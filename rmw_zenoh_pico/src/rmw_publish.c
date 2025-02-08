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

static void set_ros2_header(uint8_t *msg_bytes)
{
// This magic code is said to be used to evaluate endianness.
// How it is actually managed requires separate investigation.
  const uint8_t _header[] = {0x00, 0x01, 0x00, 0x00};

  memcpy(msg_bytes, _header, 4);
}

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

  z_mutex_lock(z_loan_mut(pub_data->mutex));

  size_t serialized_size = pub_data->callbacks->get_serialized_size(ros_message);

  serialized_size += SUB_MSG_OFFSET;
  uint8_t * msg_bytes = (uint8_t *)TOPIC_MALLOC(serialized_size);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    msg_bytes,
    "failed to allocate memory for the serialized",
    return RMW_RET_ERROR);
  memset(msg_bytes, 0, serialized_size);

  ucdrBuffer temp_buffer;
  ucdr_init_buffer_origin_offset(&temp_buffer,
				 msg_bytes,
				 serialized_size,
				 0,
				 SUB_MSG_OFFSET);

  bool ret = pub_data->callbacks->cdr_serialize(ros_message, &temp_buffer);

  set_ros2_header(msg_bytes);

  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    (void)zenoh_pico_debug_dump_msg(msg_bytes, serialized_size);
  }

  // set attachment to option
  z_publisher_put_options_t options;
  z_publisher_put_options_default(&options);

  // gen attachment data
  z_owned_bytes_t attachment;
  zenoh_pico_inc_sequence_num(&pub_data->attachment);
  if(_Z_IS_OK(zenoh_pico_attachment_gen(&pub_data->attachment, &attachment))){
    options.attachment = z_move(attachment);
  }

  // put publish data
  z_owned_bytes_t payload;
  z_bytes_copy_from_buf(&payload, msg_bytes, serialized_size);
  z_publisher_put(z_loan(pub_data->publisher),
		  z_move(payload),
		  &options);

  TOPIC_FREE(msg_bytes);
  z_drop(z_move(attachment));

  z_mutex_unlock(z_loan_mut(pub_data->mutex));

  return RMW_RET_OK;
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
