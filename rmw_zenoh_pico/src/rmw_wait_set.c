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

z_owned_mutex_t mutex_ZenohPicoWaitSetData;

ZenohPicoWaitSetData * zenoh_pico_generate_wait_set_data(rmw_context_t * context)
{

  ZenohPicoWaitSetData *wait_data = NULL;
  ZenohPicoGenerateData(wait_data, ZenohPicoWaitSetData);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    wait_data,
    "failed to allocate struct for the ZenohPicoWaitSetData",
    return NULL);

  z_mutex_init(&wait_data->condition_mutex);
  z_condvar_init(&wait_data->condition_variable);
  wait_data->triggered = false;

  wait_data->context = context;

  return wait_data;
}

bool zenoh_pico_destroy_wait_set_data(ZenohPicoWaitSetData *wait_data)
{
  z_mutex_drop(z_move(wait_data->condition_mutex));
  z_condvar_drop(z_move(wait_data->condition_variable));

  ZenohPicoDestroyData(wait_data, ZenohPicoWaitSetData);

  return true;
}

rmw_wait_set_t *
rmw_create_wait_set(rmw_context_t * context,
		    size_t max_conditions)
{
  (void)max_conditions;

  RMW_ZENOH_FUNC_ENTRY();

  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, NULL);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context->implementation_identifier,
    return NULL);

  rmw_wait_set_t *wait_set = Z_MALLOC(sizeof(rmw_wait_set_t));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    wait_set,
    "failed to allocate wait set",
    return NULL);
  wait_set->implementation_identifier = rmw_get_implementation_identifier();

  ZenohPicoWaitSetData *wait_set_data = zenoh_pico_generate_wait_set_data(context);
  if(wait_set_data == NULL)
    return NULL;

  wait_set->data = (void *)wait_set_data;

  return wait_set;
}

rmw_ret_t
rmw_destroy_wait_set(rmw_wait_set_t * wait_set)
{
  RMW_ZENOH_FUNC_ENTRY();

  RMW_CHECK_ARGUMENT_FOR_NULL(wait_set, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(wait_set->data, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    wait_set->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  ZenohPicoWaitSetData *wait_data = (ZenohPicoWaitSetData *)wait_set->data;
  zenoh_pico_destroy_wait_set_data(wait_data);

  Z_FREE(wait_set);

  return RMW_RET_OK;
}
