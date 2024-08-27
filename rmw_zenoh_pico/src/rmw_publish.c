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

#include <ctype.h>
#include <stdbool.h>

#include "zenoh-pico/system/platform-common.h"
#include <rmw/ret_types.h>
#include <rmw/rmw.h>

#include <rmw/validate_full_topic_name.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

static void set_ros2_header(uint8_t *msg_bytes)
{
  const uint8_t _header[] = {0x00, 0x01, 0x00, 0x00};

  memcpy(msg_bytes, _header, 4);
}

rmw_ret_t
rmw_publish(
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  RMW_ZENOH_FUNC_ENTRY();

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

  size_t serialized_size = pub_data->callbacks_->get_serialized_size(ros_message);

  serialized_size += SUB_MSG_OFFSET;
  uint8_t * msg_bytes = (uint8_t *)Z_MALLOC(serialized_size);
  memset(msg_bytes, 0, serialized_size);

  ucdrBuffer temp_buffer;
  ucdr_init_buffer_origin_offset(
    &temp_buffer,
    msg_bytes,
    serialized_size,
    0,
    SUB_MSG_OFFSET);

  bool ret = pub_data->callbacks_->cdr_serialize(ros_message, &temp_buffer);
  set_ros2_header(msg_bytes);

  (void)zenoh_pico_debug_dump_msg(msg_bytes, serialized_size);

  z_publisher_put_options_t options = z_publisher_put_options_default();
  options.encoding = z_encoding(Z_ENCODING_PREFIX_TEXT_PLAIN, NULL);

  z_publisher_put(z_loan(pub_data->publisher_),
		  (const uint8_t *)msg_bytes,
		  serialized_size,
		  &options);

  Z_FREE(msg_bytes);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_publish_serialized_message(
  const rmw_publisher_t * publisher,
  const rmw_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  RMW_ZENOH_FUNC_ENTRY();

  (void)publisher;
  (void)serialized_message;
  (void)allocation;
  RMW_ZENOH_LOG_INFO("function not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_publish_loaned_message(
  const rmw_publisher_t * publisher,
  void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  RMW_ZENOH_FUNC_ENTRY();

  (void)publisher;
  (void)ros_message;
  (void)allocation;

  RMW_ZENOH_LOG_INFO("function not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_publisher_wait_for_all_acked(
  const rmw_publisher_t * publisher,
  rmw_time_t wait_timeout)
{
  RMW_ZENOH_FUNC_ENTRY();

  (void)publisher;
  (void)wait_timeout;

  RMW_SET_ERROR_MSG("function not implemented");
  return RMW_RET_UNSUPPORTED;
}
