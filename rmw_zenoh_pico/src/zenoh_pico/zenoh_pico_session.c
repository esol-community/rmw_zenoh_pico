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

  // NOTE (no-Asyncify): z_open()'s own zenoh session/link-open handshake
  // can genuinely need several attempts to complete -- each recv()/send()
  // in zenoh-pico's emscripten link layer now makes exactly one
  // non-blocking attempt per call instead of internally sleeping-and-
  // retrying (see zenoh-pico's own emscripten-nonblocking-io.patch: there
  // is no way to cooperatively yield to the browser's event loop from
  // inside a single synchronous call without Asyncify). session_connect()
  // is therefore expected to be called repeatedly (from a caller-side
  // retry loop, e.g. rclpy.init() driven through a Python `await
  // asyncio.sleep()` loop) until it stops returning RMW_RET_ERROR.
  //
  // z_open() takes ownership of (z_move()s) whatever config pointer it's
  // given, whether or not it ultimately succeeds -- so retrying with
  // session->config itself (as before) would hand z_open() an
  // already-consumed, invalid config on the second and every later
  // attempt. Clone a fresh, disposable copy for each attempt instead:
  // this keeps the ORIGINAL session->config (set up once in
  // zenoh_pico_generate_session()) alive and valid across every retry.
  z_owned_config_t attempt_config;
  z_config_clone(&attempt_config, z_loan(session->config));
  if(_Z_IS_ERR(z_open(&session->session, z_move(attempt_config), NULL))){
    RMW_ZENOH_LOG_DEBUG("zenoh session not yet established, will retry");
    return RMW_RET_ERROR;
  }

#if Z_FEATURE_MULTI_THREAD == 1
  if (_Z_IS_ERR(zp_start_read_task(z_loan_mut(session->session), NULL))
      || _Z_IS_ERR(zp_start_lease_task(z_loan_mut(session->session), NULL))) {
    RMW_ZENOH_LOG_ERROR("Unable to start read and lease tasks");
    z_drop(z_move(session->config));
    z_drop(z_move(session->session));
    return RMW_RET_ERROR;
  }
#else
  // No background read/lease task to start in single-threaded mode.
  // zp_start_read_task()/zp_start_lease_task() are hard-coded to always
  // return -1 here (see zenoh-pico's own zp_start_read_task() -- it's an
  // intentional "not supported without real threads" sentinel, not a real
  // failure): rmw_wait() pumps zp_read()/zp_send_keep_alive() itself
  // instead (see this same patch's own change to rmw_wait.c).
#endif

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
