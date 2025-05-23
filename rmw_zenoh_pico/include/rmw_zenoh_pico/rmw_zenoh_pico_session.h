/*
 * Copyright(C) 2024 eSOL Co., Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RMW_ZENOH_PICO_SESSION_H
#define RMW_ZENOH_PICO_SESSION_H

#include <rmw/rmw.h>
#include <stdint.h>
#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoSession
  {
    int ref;
    z_owned_mutex_t lock;

    // Enclave, name used to find security artifacts in a sros2 keystore
    z_owned_string_t enclave;

    // configuration data array
    z_owned_config_t config;

    // An owned session.
    z_owned_session_t session;
    bool enable_session;

    // graph infomation
    rmw_guard_condition_t graph_guard_condition;

  } ZenohPicoSession;

  extern ZenohPicoSession *zenoh_pico_generate_session(const z_loaned_config_t *config,
						       const char *enclave);
  extern bool zenoh_pico_destroy_session(ZenohPicoSession *session);
  extern rmw_ret_t session_connect(ZenohPicoSession *session);

  extern bool declaration_liveliness(ZenohPicoSession *session,
				     const z_loaned_string_t *keyexpr,
				     z_owned_liveliness_token_t *token);

  extern bool isEnableSession(ZenohPicoSession *session);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
