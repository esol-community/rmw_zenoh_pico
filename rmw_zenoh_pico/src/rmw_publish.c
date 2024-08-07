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

#include <rmw/ret_types.h>
#include <rmw/rmw.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

rmw_ret_t
rmw_publish(
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)allocation;
  rmw_ret_t ret = RMW_RET_OK;
  if (!publisher) {
    RMW_UROS_TRACE_MESSAGE("publisher pointer is null")
    ret = RMW_RET_ERROR;
  } else if (!ros_message) {
    RMW_UROS_TRACE_MESSAGE("ros_message pointer is null")
    ret = RMW_RET_ERROR;
    ret = RMW_RET_ERROR;
  } else if (!publisher->data) {
    RMW_UROS_TRACE_MESSAGE("publisher imp is null");
    ret = RMW_RET_ERROR;
  } else {
    bool written = false;
  }
  return ret;
}

rmw_ret_t
rmw_publish_serialized_message(
  const rmw_publisher_t * publisher,
  const rmw_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)publisher;
  (void)serialized_message;
  (void)allocation;
  RMW_UROS_TRACE_MESSAGE("function not implemented")
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_publish_loaned_message(
  const rmw_publisher_t * publisher,
  void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)publisher;
  (void)ros_message;
  (void)allocation;

  RMW_UROS_TRACE_MESSAGE("function not implemented")
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_publisher_wait_for_all_acked(
  const rmw_publisher_t * publisher,
  rmw_time_t wait_timeout)
{
  (void)publisher;
  (void)wait_timeout;

  RMW_SET_ERROR_MSG("function not implemented");
  return RMW_RET_UNSUPPORTED;
}
