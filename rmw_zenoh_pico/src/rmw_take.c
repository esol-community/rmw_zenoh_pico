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

static rmw_ret_t
rmw_zenoh_pico_take_one(ZenohPicoSubData * sub_data,
		void * ros_message,
		rmw_message_info_t * message_info,
		bool * taken)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  ReceiveMessageData *msg_data = recv_msg_list_pop(&sub_data->message_queue);
  RMW_CHECK_ARGUMENT_FOR_NULL(msg_data, RMW_RET_ERROR);

  bool deserialize_rv = rmw_zenoh_pico_deserialize_topic_msg(msg_data,
						       sub_data->callbacks,
						       ros_message,
						       message_info);
  if (taken != NULL) {
    *taken = deserialize_rv;
  }

  if (!deserialize_rv) {
    RMW_SET_ERROR_MSG("Typesupport deserialize error.");
    return RMW_RET_ERROR;
  }

  ZenohPicoDataDestroy(msg_data);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_take(const rmw_subscription_t * subscription,
	 void * ros_message,
	 bool * taken,
	 rmw_subscription_allocation_t * allocation)
{
  RMW_ZENOH_FUNC_ENTRY(subscription);

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->topic_name, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->data, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  *taken = false;
  ZenohPicoSubData *sub_data = (ZenohPicoSubData *)subscription->data;
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);

  return rmw_zenoh_pico_take_one(sub_data, ros_message, NULL, taken);
}

rmw_ret_t
rmw_take_with_info(const rmw_subscription_t * subscription,
		   void * ros_message,
		   bool * taken,
		   rmw_message_info_t * message_info,
		   rmw_subscription_allocation_t * allocation)
{
  RMW_ZENOH_FUNC_ENTRY(subscription);

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->topic_name, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->data, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  *taken = false;
  ZenohPicoSubData *sub_data = (ZenohPicoSubData *)subscription->data;
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);

  return rmw_zenoh_pico_take_one(sub_data, ros_message, message_info, taken);
}

rmw_ret_t
rmw_take_sequence(const rmw_subscription_t * subscription,
		  size_t count,
		  rmw_message_sequence_t * message_sequence,
		  rmw_message_info_sequence_t * message_info_sequence,
		  size_t * taken,
		  rmw_subscription_allocation_t * allocation)
{
  RMW_ZENOH_FUNC_ENTRY(subscription);

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->topic_name, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->data, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_sequence, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info_sequence, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  if (0u == count) {
    RMW_SET_ERROR_MSG("count cannot be 0");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (count > message_sequence->capacity) {
    RMW_SET_ERROR_MSG("Insuffient capacity in message_sequence");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (count > message_info_sequence->capacity) {
    RMW_SET_ERROR_MSG("Insuffient capacity in message_info_sequence");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (count > UINT_MAX) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Cannot take %zu samples at once, limit is %u", count, UINT_MAX);
    return RMW_RET_ERROR;
  }

  *taken = 0;

  ZenohPicoSubData *sub_data = (ZenohPicoSubData *)subscription->data;
  if (sub_data == NULL){
    RMW_SET_ERROR_MSG("don't found subscription");
    return RMW_RET_INVALID_ARGUMENT;

  }

  rmw_ret_t ret = RMW_RET_ERROR;

  while (*taken < count) {
    bool one_taken = false;

    ret = rmw_zenoh_pico_take_one(
      sub_data,
      message_sequence->data[*taken],
      &message_info_sequence->data[*taken],
      &one_taken);
    if (ret != RMW_RET_OK) {
      // If we are taking a sequence and the 2nd take in the sequence failed, we'll report
      // RMW_RET_ERROR to the caller, but we will *also* tell the caller that there are valid
      // messages already taken (via the message_sequence size).  It is up to the caller to deal
      // with that situation appropriately.
      break;
    }

    if (!one_taken) {
      // No error, but there was nothing left to be taken, so break out of the loop
      break;
    }

    (*taken)++;
  }

  message_sequence->size = *taken;
  message_info_sequence->size = *taken;

  return ret;
}

rmw_ret_t
rmw_take_serialized_message(const rmw_subscription_t * subscription,
			    rmw_serialized_message_t * serialized_message,
			    bool * taken,
			    rmw_subscription_allocation_t * allocation)
{
  RMW_ZENOH_FUNC_ENTRY(subscription);

  (void)subscription;
  (void)serialized_message;
  (void)taken;
  (void)allocation;
  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_take_serialized_message_with_info(const rmw_subscription_t * subscription,
				      rmw_serialized_message_t * serialized_message,
				      bool * taken,
				      rmw_message_info_t * message_info,
				      rmw_subscription_allocation_t * allocation)
{
  RMW_ZENOH_FUNC_ENTRY(subscription);

  (void)subscription;
  (void)serialized_message;
  (void)taken;
  (void)message_info;
  (void)allocation;
  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_take_loaned_message(const rmw_subscription_t * subscription,
			void ** loaned_message,
			bool * taken,
			rmw_subscription_allocation_t * allocation)
{
  RMW_ZENOH_FUNC_ENTRY(subscription);

  (void)subscription;
  (void)loaned_message;
  (void)taken;
  (void)allocation;

  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_take_loaned_message_with_info(const rmw_subscription_t * subscription,
				  void ** loaned_message,
				  bool * taken,
				  rmw_message_info_t * message_info,
				  rmw_subscription_allocation_t * allocation)
{
  RMW_ZENOH_FUNC_ENTRY(subscription);

  (void)subscription;
  (void)loaned_message;
  (void)taken;
  (void)message_info;
  (void)allocation;

  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_return_loaned_message_from_subscription(const rmw_subscription_t * subscription,
					    void * loaned_message)
{
  RMW_ZENOH_FUNC_ENTRY(subscription);

  (void)subscription;
  (void)loaned_message;

  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}
