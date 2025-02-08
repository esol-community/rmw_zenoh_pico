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

#include <rmw/rmw.h>
#include <rmw/names_and_types.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

void guard_condition_trigger(ZenohPicoGuardConditionData *condition_data)
{
  z_mutex_lock(z_loan_mut(condition_data->condition_mutex));
  condition_data->triggered = true;

  if(condition_data->wait_set_data != NULL){
    ZenohPicoWaitSetData * wait_set_data = condition_data->wait_set_data;
    wait_condition_lock(wait_set_data);
    wait_set_data->triggered = true;
    wait_condition_unlock(wait_set_data);
  }

  z_mutex_unlock(z_loan_mut(condition_data->condition_mutex));
}

bool guard_condition_check_and_attach(ZenohPicoGuardConditionData *condition_data,
				      ZenohPicoWaitSetData * wait_set_data)
{
  z_mutex_lock(z_loan_mut(condition_data->condition_mutex));

  if(condition_data->triggered) {
    z_mutex_unlock(z_loan_mut(condition_data->condition_mutex));
    return true;
  }

  if(wait_set_data != NULL){
    condition_data->wait_set_data = wait_set_data;
  }

  z_mutex_unlock(z_loan_mut(condition_data->condition_mutex));

  return false;
}

bool guard_condition_detach_and_is_trigger_set(ZenohPicoGuardConditionData *condition_data)
{
  bool value;
  z_mutex_lock(z_loan_mut(condition_data->condition_mutex));

  value = condition_data->triggered;
  condition_data->wait_set_data = NULL;

  z_mutex_unlock(z_loan_mut(condition_data->condition_mutex));

  return value;
}

rmw_ret_t
rmw_trigger_guard_condition(
  const rmw_guard_condition_t * guard_condition)
{
  RMW_ZENOH_FUNC_ENTRY(guard_condition);

  RMW_CHECK_ARGUMENT_FOR_NULL(guard_condition, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    guard_condition->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  ZenohPicoGuardConditionData *condition_data = (ZenohPicoGuardConditionData *)guard_condition->data;
  RMW_CHECK_ARGUMENT_FOR_NULL(condition_data, RMW_RET_INVALID_ARGUMENT);

  guard_condition_trigger(condition_data);

  return RMW_RET_OK;
}
