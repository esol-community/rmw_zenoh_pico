// Copyright 2020 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

void data_callback_init(DataCallbackManager *data_callback)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  z_mutex_init(&data_callback->mutex);
  data_callback->callback = NULL;
  data_callback->user_data = NULL;
  data_callback->unread_count = 0;
}

void data_callback_set(DataCallbackManager *data_callback,
		       const void * user_data,
		       rmw_event_callback_t callback)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  z_mutex_lock(z_loan_mut(data_callback->mutex));

  if(callback != NULL){
    if(data_callback->unread_count) {
      data_callback->callback(data_callback->user_data,
			      data_callback->unread_count);
      data_callback->unread_count = 0;
    }
    data_callback->user_data = user_data;
    data_callback->callback  = callback;

  }else{
    data_callback->user_data = NULL;
    data_callback->callback  = NULL;
  }

  z_mutex_unlock(z_loan_mut(data_callback->mutex));
}

void data_callback_trigger(DataCallbackManager *data_callback)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  z_mutex_lock(z_loan_mut(data_callback->mutex));
  if(data_callback->callback != NULL){
    data_callback->callback(data_callback->user_data, 1);
  }else{
    data_callback->unread_count += 1;
  }
  z_mutex_unlock(z_loan_mut(data_callback->mutex));
}

rmw_ret_t
rmw_subscription_set_on_new_message_callback(rmw_subscription_t * subscription,
					     rmw_event_callback_t callback,
					     const void * user_data)
{
  RMW_ZENOH_FUNC_ENTRY(subscription);

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);

  ZenohPicoSubData *sub_data = (ZenohPicoSubData *)subscription->data;
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);

  data_callback_set(&sub_data->data_callback_mgr,
		    user_data,
		    callback);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_service_set_on_new_request_callback(rmw_service_t * service,
					rmw_event_callback_t callback,
					const void * user_data)
{
  RMW_ZENOH_FUNC_ENTRY(service);

  (void) service;
  (void) callback;
  (void) user_data;

  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_client_set_on_new_response_callback(rmw_client_t * client,
					rmw_event_callback_t callback,
					const void * user_data)
{
  RMW_ZENOH_FUNC_ENTRY(client);

  (void) client;
  (void) callback;
  (void) user_data;

  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_event_set_callback(rmw_event_t * event,
		       rmw_event_callback_t callback,
		       const void * user_data)
{
  RMW_ZENOH_FUNC_ENTRY(event);

  (void) event;
  (void) callback;
  (void) user_data;

  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}
