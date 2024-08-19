// Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include "zenoh-pico/system/platform-common.h"
#include <rmw/rmw.h>
#include <rmw/allocators.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

ZenohPicoGuardConditionData * zenoh_pico_guard_condition_data(void)
{
  ZenohPicoGuardConditionData *condition_data = z_malloc(sizeof(ZenohPicoGuardConditionData));
  memset(condition_data, 0, sizeof(ZenohPicoGuardConditionData));

  z_mutex_init(&condition_data->internal_mutex_);
  condition_data->triggered_ = false;

  return condition_data;
}

bool zenoh_pico_destroy_guard_condition_data(ZenohPicoGuardConditionData *condition_data)
{
    z_free(condition_data);
    return true;
}

rmw_guard_condition_t *
rmw_create_guard_condition(
  rmw_context_t * context)
{
  rmw_guard_condition_t *guard_condition = z_malloc(sizeof(rmw_guard_condition_t));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    guard_condition,
    "unable to allocate memory for guard_condition",
    NULL);

  guard_condition->implementation_identifier = rmw_get_implementation_identifier();
  guard_condition->context = context;

  RMW_CHECK_FOR_NULL_WITH_MSG(
    guard_condition->data,
    "unable to allocate memory for guard condition data",
    NULL);

  ZenohPicoGuardConditionData *condition_data = zenoh_pico_guard_condition_data();
  guard_condition->data = (ZenohPicoGuardConditionData *)condition_data;

  return guard_condition;
}

rmw_ret_t
rmw_destroy_guard_condition(
  rmw_guard_condition_t * guard_condition)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(guard_condition, RMW_RET_INVALID_ARGUMENT);

  if(guard_condition->data) {
    ZenohPicoGuardConditionData *condition_data = (ZenohPicoGuardConditionData *)guard_condition->data;
    (void)zenoh_pico_destroy_guard_condition_data(condition_data);
    guard_condition->data = NULL;
  }

  z_free(guard_condition);

  return RMW_RET_OK;
}
