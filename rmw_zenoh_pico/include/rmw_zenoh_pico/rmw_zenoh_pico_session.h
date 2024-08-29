/*
 * Copyright (C)
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
    int ref_;

    // Enclave, name used to find security artifacts in a sros2 keystore
    z_string_t enclave_;

    // configuration data array
    z_owned_config_t config_;

    // An owned session.
    z_owned_session_t session_;

    // graph infomation
    rmw_guard_condition_t graph_guard_condition;

  } ZenohPicoSession;

  extern ZenohPicoSession *zenoh_pico_generate_session(ZenohPicoSession *session,
						       z_owned_config_t *config,
						       const char *enclave);
  extern bool zenoh_pico_destroy_session(ZenohPicoSession *session);
  extern bool zenoh_pico_clone_session(ZenohPicoSession *dst, ZenohPicoSession *src);
  extern rmw_ret_t session_connect(ZenohPicoSession *session);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
