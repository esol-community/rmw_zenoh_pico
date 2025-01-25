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

#ifndef RMW_ZENOH_PICO_GUARD_CONDITION_H
#define RMW_ZENOH_PICO_GUARD_CONDITION_H

#include <rmw/rmw.h>
#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

#include <rmw_zenoh_pico/rmw_zenoh_pico_wait.h>

  typedef struct _ZenohPicoGuardConditionData {
    z_mutex_t condition_mutex;
    bool triggered;
    ZenohPicoWaitSetData * wait_set_data;

  } ZenohPicoGuardConditionData;

  extern ZenohPicoGuardConditionData * zenoh_pico_guard_condition_data(void);
  extern bool zenoh_pico_destroy_guard_condition_data(ZenohPicoGuardConditionData *condition_data);

  // ---------

  extern void guard_condition_trigger(ZenohPicoGuardConditionData *condition_data);
  extern bool guard_condition_check_and_attach(ZenohPicoGuardConditionData *condition_data,
				    ZenohPicoWaitSetData * wait_set_data);
  extern bool guard_condition_detach_and_is_trigger_set(ZenohPicoGuardConditionData *condition_data);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
