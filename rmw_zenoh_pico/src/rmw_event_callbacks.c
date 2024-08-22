// Copyright 2020 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
#include "zenoh-pico/system/platform/unix.h"
#include <rmw/rmw.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

void data_callback_init(DataCallbackManager *data_callback)
{
  z_mutex_init(&data_callback->mutext_);
  data_callback->callback_ = NULL;
  data_callback->user_data_ = NULL;
  data_callback->unread_count_ = 0;
}

void data_callback_set(DataCallbackManager *data_callback,
		       const void * user_data,
		       rmw_event_callback_t callback)
{
  z_mutex_lock(&data_callback->mutext_);

  if(callback != NULL){
    if(data_callback->unread_count_) {
      data_callback->callback_(data_callback->user_data_,
			       data_callback->unread_count_);
      data_callback->unread_count_ = 0;
    }
    data_callback->user_data_ = user_data;
    data_callback->callback_  = callback;

  }else{
    data_callback->user_data_ = NULL;
    data_callback->callback_  = NULL;
  }

  z_mutex_unlock(&data_callback->mutext_);
}

void data_callback_trigger(DataCallbackManager *data_callback)
{
  z_mutex_lock(&data_callback->mutext_);
  if(data_callback->callback_ != NULL){
    data_callback->callback_(data_callback->user_data_, 1);
  }else{
    data_callback->unread_count_ += 1;
  }
  z_mutex_unlock(&data_callback->mutext_);
}

rmw_ret_t
rmw_subscription_set_on_new_message_callback(
  rmw_subscription_t * subscription,
  rmw_event_callback_t callback,
  const void * user_data)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);

  ZenohPicoSubData *sub_data = (ZenohPicoSubData *)subscription->data;
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);

  data_callback_set(&sub_data->data_callback_mgr,
		    user_data,
		    callback);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_service_set_on_new_request_callback(
  rmw_service_t * service,
  rmw_event_callback_t callback,
  const void * user_data)
{
  (void) service;
  (void) callback;
  (void) user_data;

  RMW_UROS_TRACE_MESSAGE("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_client_set_on_new_response_callback(
  rmw_client_t * client,
  rmw_event_callback_t callback,
  const void * user_data)
{
  (void) client;
  (void) callback;
  (void) user_data;

  RMW_UROS_TRACE_MESSAGE("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_event_set_callback(
  rmw_event_t * event,
  rmw_event_callback_t callback,
  const void * user_data)
{
  (void) event;
  (void) callback;
  (void) user_data;

  RMW_UROS_TRACE_MESSAGE("function not implemented");
  return RMW_RET_UNSUPPORTED;
}
