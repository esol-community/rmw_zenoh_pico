// Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// Copyright(C) 2024 eSOL Co., Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>
#include <string.h>

static ZenohPicoSession _zenohSession;

ZenohPicoSession *zenoh_pico_generate_session(const z_loaned_config_t *config,
					      const char *enclave)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  ZenohPicoSession *session = &_zenohSession;
  if(_zenohSession.ref > 0){
    ZenohPicoDataRefClone(session);
    return &_zenohSession;
  }

  session = ZenohPicoDataGenerate(session);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    session,
    "failed to allocate struct for the ZenohPicoSession",
    return NULL);
  memset(session, 0, sizeof(ZenohPicoSession));

  z_config_clone(&session->config, config);

  if(enclave != NULL) {
    z_string_copy_from_str(&session->enclave, enclave);
  }

  session->graph_guard_condition.implementation_identifier = rmw_get_implementation_identifier();
  session->graph_guard_condition.data = zenoh_pico_guard_condition_data;

  session->enable_session = false;

  ZenohPicoDataRefClone(session);

  return session;
}

bool zenoh_pico_destroy_session(ZenohPicoSession *session)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  RMW_CHECK_ARGUMENT_FOR_NULL(session, false);

  if(ZenohPicoDataRelease(session)){

    // stop background zenoh task
    if(session->enable_session){
      zp_stop_read_task(z_loan_mut(session->session));
      zp_stop_lease_task(z_loan_mut(session->session));
    }

    zenoh_pico_destroy_guard_condition_data((ZenohPicoGuardConditionData *)session->graph_guard_condition.data);

    z_drop(z_move(session->config));
    z_drop(z_move(session->enclave));
    z_drop(z_move(session->session));
  }

  return true;
}

rmw_ret_t session_connect(ZenohPicoSession *session)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  RMW_CHECK_ARGUMENT_FOR_NULL(session, false);

  if(session->enable_session){
    RMW_ZENOH_LOG_DEBUG("already session open.");
    return RMW_RET_OK;
  }

  RMW_ZENOH_LOG_DEBUG("Opening session...");

  if(_Z_IS_ERR(z_open(&session->session, z_move(session->config), NULL))){
    RMW_ZENOH_LOG_ERROR("Error setting up zenoh session");
    return RMW_RET_ERROR;
  }

  if (_Z_IS_ERR(zp_start_read_task(z_loan_mut(session->session), NULL))
      || _Z_IS_ERR(zp_start_lease_task(z_loan_mut(session->session), NULL))) {
    RMW_ZENOH_LOG_ERROR("Unable to start read and lease tasks");
    z_drop(z_move(session->config));
    z_drop(z_move(session->session));
    return RMW_RET_ERROR;
  }

  session->enable_session = true;

  RMW_ZENOH_LOG_DEBUG("complite.");

  return RMW_RET_OK;
}

bool isEnableSession(ZenohPicoSession *session) {
  return session->enable_session;
}

bool declaration_liveliness(ZenohPicoSession *session,
			    const z_loaned_string_t *keyexpr,
			    z_owned_liveliness_token_t *token)
{
    z_view_keyexpr_t ke;
    z_view_keyexpr_from_substr(&ke, z_string_data(keyexpr), z_string_len(keyexpr));
    if(_Z_IS_ERR(z_liveliness_declare_token(z_loan(session->session),
					    token,
					    z_loan(ke),
					    NULL))){
      RMW_ZENOH_LOG_INFO("Unable to declare token.");
      return false;
    }
    return true;
}
