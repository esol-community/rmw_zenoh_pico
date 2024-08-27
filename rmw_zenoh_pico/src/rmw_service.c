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

#ifdef HAVE_C_TYPESUPPORT
#include <rosidl_typesupport_microxrcedds_c/identifier.h>
#endif /* ifdef HAVE_C_TYPESUPPORT */
#ifdef HAVE_CPP_TYPESUPPORT
#include <rosidl_typesupport_microxrcedds_cpp/identifier.h>
#endif /* ifdef HAVE_CPP_TYPESUPPORT */

#include <rmw/rmw.h>
#include <rmw/allocators.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

rmw_service_t *
rmw_create_service(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_support,
  const char * service_name,
  const rmw_qos_profile_t * qos_policies)
{
  rmw_service_t * rmw_service = NULL;
  if (!node) {
    _Z_INFO("node handle is null");
  } else if (!type_support) {
    _Z_INFO("type support is null");
  } else if (!service_name || strlen(service_name) == 0) {
    _Z_INFO("service name is null or empty string");
  } else if (!qos_policies) {
    _Z_INFO("qos_profile is null");
  } else {
  }
  return rmw_service;
}

rmw_ret_t
rmw_destroy_service(
  rmw_node_t * node,
  rmw_service_t * service)
{
  rmw_ret_t result_ret = RMW_RET_OK;
  if (!node) {
    _Z_INFO("node handle is null");
    result_ret = RMW_RET_ERROR;
    result_ret = RMW_RET_ERROR;
  } else if (!node->data) {
    _Z_INFO("node imp is null");
    result_ret = RMW_RET_ERROR;
  } else if (!service) {
    _Z_INFO("service handle is null");
    result_ret = RMW_RET_ERROR;
    result_ret = RMW_RET_ERROR;
  } else if (!service->data) {
    _Z_INFO("service imp is null");
    result_ret = RMW_RET_ERROR;
  } else {
  }

  return result_ret;
}

rmw_ret_t
rmw_service_response_publisher_get_actual_qos(
  const rmw_service_t * service,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_service_request_subscription_get_actual_qos(
  const rmw_service_t * service,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  return RMW_RET_OK;
}
