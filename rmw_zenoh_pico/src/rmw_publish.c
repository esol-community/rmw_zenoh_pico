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


rmw_ret_t
rmw_publish(const rmw_publisher_t * publisher,
	    const void * ros_message,
	    rmw_publisher_allocation_t * allocation)
{
  RMW_ZENOH_FUNC_ENTRY(publisher);

  (void)allocation;

  RMW_CHECK_FOR_NULL_WITH_MSG(
    publisher, "publisher handle is null",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    ros_message, "ros message handle is null",
    return RMW_RET_INVALID_ARGUMENT);

  ZenohPicoPubData *pub_data = (ZenohPicoPubData *)publisher->data;

  return rmw_zenoh_pico_publish(pub_data, ros_message);
}

rmw_ret_t
rmw_publish_serialized_message(const rmw_publisher_t * publisher,
			       const rmw_serialized_message_t * serialized_message,
			       rmw_publisher_allocation_t * allocation)
{
  RMW_ZENOH_FUNC_ENTRY(publisher);

  (void)publisher;
  (void)serialized_message;
  (void)allocation;
  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_publish_loaned_message(const rmw_publisher_t * publisher,
			   void * ros_message,
			   rmw_publisher_allocation_t * allocation)
{
  RMW_ZENOH_FUNC_ENTRY(publisher);

  (void)publisher;
  (void)ros_message;
  (void)allocation;

  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_publisher_wait_for_all_acked(const rmw_publisher_t * publisher,
				 rmw_time_t wait_timeout)
{
  RMW_ZENOH_FUNC_ENTRY(publisher);

  (void)publisher;
  (void)wait_timeout;

  RMW_SET_ERROR_MSG("function not implemented");
  return RMW_RET_UNSUPPORTED;
}
