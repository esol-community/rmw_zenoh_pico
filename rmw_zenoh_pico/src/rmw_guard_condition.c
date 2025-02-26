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

ZenohPicoGuardConditionData * zenoh_pico_guard_condition_data(void)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  ZenohPicoGuardConditionData *condition_data = Z_MALLOC(sizeof(ZenohPicoGuardConditionData));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    condition_data,
    "failed to allocate memory for the condition_data",
    return NULL);

  memset(condition_data, 0, sizeof(ZenohPicoGuardConditionData));

  z_mutex_init(&condition_data->condition_mutex);
  condition_data->triggered = false;

  return condition_data;
}

bool zenoh_pico_destroy_guard_condition_data(ZenohPicoGuardConditionData *condition_data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  z_mutex_drop(z_move(condition_data->condition_mutex));
  if(condition_data != NULL)
    Z_FREE(condition_data);

  return true;
}

rmw_guard_condition_t *
rmw_create_guard_condition(rmw_context_t * context)
{
  RMW_ZENOH_FUNC_ENTRY(context);

  rmw_guard_condition_t *guard_condition = Z_MALLOC(sizeof(rmw_guard_condition_t));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    guard_condition,
    "unable to allocate memory for guard_condition",
    return NULL);

  guard_condition->implementation_identifier = rmw_get_implementation_identifier();
  guard_condition->context = context;

  ZenohPicoGuardConditionData *condition_data = zenoh_pico_guard_condition_data();
  RMW_CHECK_FOR_NULL_WITH_MSG(
    condition_data,
    "unable to allocate memory for guard condition data",
    return NULL);
  guard_condition->data = (ZenohPicoGuardConditionData *)condition_data;

  return guard_condition;
}

rmw_ret_t
rmw_destroy_guard_condition(rmw_guard_condition_t * guard_condition)
{
  RMW_ZENOH_FUNC_ENTRY(guard_condition);

  RMW_CHECK_ARGUMENT_FOR_NULL(guard_condition, RMW_RET_INVALID_ARGUMENT);

  if(guard_condition->data) {
    ZenohPicoGuardConditionData *condition_data = (ZenohPicoGuardConditionData *)guard_condition->data;
    (void)zenoh_pico_destroy_guard_condition_data(condition_data);
    guard_condition->data = NULL;
  }

  Z_FREE(guard_condition);

  return RMW_RET_OK;
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
