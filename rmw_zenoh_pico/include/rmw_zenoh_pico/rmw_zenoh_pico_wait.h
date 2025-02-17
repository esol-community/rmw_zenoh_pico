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

#ifndef RMW_ZENOH_PICO_WAIT_H
#define RMW_ZENOH_PICO_WAIT_H

#include <rmw/rmw.h>
#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoWaitSetData {
    int ref;
    z_owned_mutex_t lock;

    z_owned_condvar_t condition_variable;
    z_owned_mutex_t condition_mutex;

    bool triggered;
    rmw_context_t * context;

  } ZenohPicoWaitSetData;

  extern ZenohPicoWaitSetData * zenoh_pico_generate_wait_data(rmw_context_t * context);
  extern bool zenoh_pico_destroy_wait_data(ZenohPicoWaitSetData *wait_data);

  extern void wait_condition_lock(ZenohPicoWaitSetData * wait_set_data);
  extern void wait_condition_unlock(ZenohPicoWaitSetData * wait_set_data);
  extern void wait_condition_signal(ZenohPicoWaitSetData * wait_set_data);
  extern void wait_condition_triggered(ZenohPicoWaitSetData * wait_set_data, bool value);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
