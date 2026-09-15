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

static int support_event_type(rmw_event_type_t type)
{
  // This used to read `#ifndef EVENT_TABLE_SIZE` -- but that macro (see
  // rmw_zenoh_pico_event.h) is unconditionally defined by the very
  // header this file includes, so the #ifndef is always false and the
  // lookup loop below was preprocessed out entirely, unconditionally.
  // support_event_type() therefore always fell straight through to
  // `return -1`, for every event type, no matter what `_support_event[]`
  // actually listed -- silently skipping every
  // add_rmw_zenoh_pico_event_total()/_current()/take_event_status() call
  // project-wide, and making rmw_publisher_event_init()/
  // rmw_subscription_event_init() always fail "unsupported" regardless
  // of `_support_event[]`'s contents.
  for(size_t index = 0; index < EVENT_TABLE_SIZE; index++){
    if(_support_event[index] == type)
      return index;
  }
  return -1;
}

static EventStatus *take_event_status(DataEventManager *event_mgr, rmw_event_type_t type)
{
  int index = support_event_type(type);

  if(index < 0)
    return NULL;

  return (event_mgr->event_status +index);
}

void add_rmw_zenoh_pico_event_total(DataEventManager *event_mgr, rmw_event_type_t type, bool change)
{
  z_mutex_lock(z_loan_mut(event_mgr->mutex));

  int index = support_event_type(type);
  EventStatus *status = take_event_status(event_mgr, type);
  if(status != NULL){
    status->total_count += 1;
    if(change)
      status->total_count_change += 1;
    status->changed = true;
  }

  z_mutex_unlock(z_loan_mut(event_mgr->mutex));

  return;
}

void add_rmw_zenoh_pico_event_current(DataEventManager *event_mgr, rmw_event_type_t type, bool change)
{

  z_mutex_lock(z_loan_mut(event_mgr->mutex));

  int index = support_event_type(type);
  EventStatus *status = take_event_status(event_mgr, type);
  if(status != NULL){
    status->current_count += 1;
    if(change)
      status->current_count_change += 1;
    status->changed = true;
  }

  z_mutex_unlock(z_loan_mut(event_mgr->mutex));

  return;
}

// Counterpart to add_rmw_zenoh_pico_event_current() for event types whose
// current_count can genuinely decrease (RMW_EVENT_*_MATCHED: how many
// peers are matched *right now*, as opposed to the monotonic total_count
// lifetime counter -- see rmw_matched_status_t's own documented
// semantics). Floors at 0 rather than underflowing.
void sub_rmw_zenoh_pico_event_current(DataEventManager *event_mgr, rmw_event_type_t type, bool change)
{
  z_mutex_lock(z_loan_mut(event_mgr->mutex));

  EventStatus *status = take_event_status(event_mgr, type);
  if(status != NULL && status->current_count > 0){
    status->current_count -= 1;
    if(change)
      status->current_count_change += 1;
    status->changed = true;
  }

  z_mutex_unlock(z_loan_mut(event_mgr->mutex));

  return;
}

// Sets which QoS policy caused the most recent
// REQUESTED/OFFERED_QOS_INCOMPATIBLE incident (rmw_*_qos_incompatible_
// event_status_t's own documented last_policy_kind field).
void set_rmw_zenoh_pico_event_last_policy_kind(DataEventManager *event_mgr, rmw_event_type_t type, rmw_qos_policy_kind_t policy_kind)
{
  z_mutex_lock(z_loan_mut(event_mgr->mutex));

  EventStatus *status = take_event_status(event_mgr, type);
  if(status != NULL){
    status->last_policy_kind = policy_kind;
  }

  z_mutex_unlock(z_loan_mut(event_mgr->mutex));
}

// Reads back an event's live current_count (e.g. RMW_EVENT_*_MATCHED)
// without consuming/clearing its "changed" flag -- used by
// rmw_publisher_count_matched_subscriptions()/
// rmw_subscription_count_matched_publishers(), which report a live
// snapshot rather than a one-shot event.
size_t get_rmw_zenoh_pico_event_current_count(DataEventManager *event_mgr, rmw_event_type_t type)
{
  z_mutex_lock(z_loan_mut(event_mgr->mutex));

  size_t count = 0;
  EventStatus *status = take_event_status(event_mgr, type);
  if(status != NULL)
    count = status->current_count;

  z_mutex_unlock(z_loan_mut(event_mgr->mutex));

  return count;
}

bool is_rmw_zenoh_pico_event_changed(DataEventManager *event_mgr, rmw_event_type_t type)
{
  z_mutex_lock(z_loan_mut(event_mgr->mutex));

  bool ret = false;
  EventStatus *status = take_event_status(event_mgr, type);

  if(status != NULL){
    if(status->changed){
      status->changed = false;
      ret = true;
    }
  }

  z_mutex_unlock(z_loan_mut(event_mgr->mutex));
  return ret;
}

static bool event_condition_check(DataEventManager *event_mgr)
{
  // Same broken `#ifndef EVENT_TABLE_SIZE` issue as support_event_type()
  // above -- always false, so this loop was always skipped and
  // event_condition_check() always returned false. Meant rmw_wait()
  // could never be woken by a genuine event-status change reaching
  // event_condition_check_and_attach()/
  // event_condition_detach_and_queue_is_empty() through this path.
  for(size_t index = 0; index < EVENT_TABLE_SIZE; index++){
    rmw_event_type_t type = _support_event[index];

    if(is_rmw_zenoh_pico_event_changed(event_mgr, type))
      return true;
  }

  return false;
}

void event_condition_trigger(DataEventManager *event_mgr)
{
  ZenohPicoWaitCondition cond;
  cond.condition_mutex		= z_loan_mut(event_mgr->condition_mutex);
  cond.msg_queue		= NULL;
  cond.wait_set_data_ptr	= &event_mgr->wait_set_data;

  zenoh_pico_condition_trigger(&cond);
}

bool event_condition_check_and_attach(DataEventManager *event_mgr,
				      ZenohPicoWaitSetData *wait_set_mgr)
{
  z_mutex_lock(z_loan_mut(event_mgr->condition_mutex));

  if(event_condition_check(event_mgr)){
    z_mutex_unlock(z_loan_mut(event_mgr->condition_mutex));
    return true;
  }

  event_mgr->wait_set_data = wait_set_mgr;

  z_mutex_unlock(z_loan_mut(event_mgr->condition_mutex));

  return false;
}

bool event_condition_detach_and_queue_is_empty(DataEventManager *event_mgr)
{
  bool ret;

  z_mutex_lock(z_loan_mut(event_mgr->condition_mutex));

  event_mgr->wait_set_data = NULL;
  ret = !event_condition_check(event_mgr);

  z_mutex_unlock(z_loan_mut(event_mgr->condition_mutex));

  return ret;
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

  data_callback_set(&sub_data->data_event_mgr,
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

  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service->data, RMW_RET_INVALID_ARGUMENT);

  ZenohPicoServiceData *service_data = (ZenohPicoServiceData *)service->data;
  RMW_CHECK_ARGUMENT_FOR_NULL(service_data, RMW_RET_INVALID_ARGUMENT);

  data_callback_set(&service_data->data_event_mgr,
		    user_data,
		    callback);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_client_set_on_new_response_callback(rmw_client_t * client,
					rmw_event_callback_t callback,
					const void * user_data)
{
  RMW_ZENOH_FUNC_ENTRY(client);

  ZenohPicoServiceData * client_data = (ZenohPicoServiceData * )client->data;

  RMW_CHECK_ARGUMENT_FOR_NULL(client_data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client->data, RMW_RET_INVALID_ARGUMENT);

  data_callback_set(&client_data->data_event_mgr,
		    user_data,
		    callback);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_event_set_callback(rmw_event_t * event,
		       rmw_event_callback_t callback,
		       const void * user_data)
{
  RMW_ZENOH_FUNC_ENTRY(event);

  RMW_CHECK_ARGUMENT_FOR_NULL(event, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(event->data, RMW_RET_INVALID_ARGUMENT);

  DataEventManager * event_mgr = (DataEventManager *)event->data;

  RMW_CHECK_ARGUMENT_FOR_NULL(event_mgr, RMW_RET_INVALID_ARGUMENT);

  data_callback_set(event_mgr,
		    user_data,
		    callback);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_publisher_event_init(
  rmw_event_t * rmw_event,
  const rmw_publisher_t * publisher,
  rmw_event_type_t event_type)
{
  RMW_ZENOH_FUNC_ENTRY(rmw_event);

  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_event, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher->implementation_identifier, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher->data, RMW_RET_INVALID_ARGUMENT);

  if(support_event_type(event_type) < 0){
    RMW_SET_ERROR_MSG("Publisher implementation identifier not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  ZenohPicoPubData *pub_data = (ZenohPicoPubData *)publisher->data;
  RMW_CHECK_ARGUMENT_FOR_NULL(pub_data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rmw_event->implementation_identifier	= rmw_get_implementation_identifier();
  rmw_event->data			= (void *)&pub_data->data_event_mgr;
  rmw_event->event_type			= event_type;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_subscription_event_init(
  rmw_event_t * rmw_event,
  const rmw_subscription_t * subscription,
  rmw_event_type_t event_type)
{
  RMW_ZENOH_FUNC_ENTRY(rmw_event);

  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_event, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->implementation_identifier, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->data, RMW_RET_INVALID_ARGUMENT);

  if(support_event_type(event_type) < 0){
    RMW_SET_ERROR_MSG("Publisher implementation identifier not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  ZenohPicoSubData *pub_data = (ZenohPicoSubData *)subscription->data;
  RMW_CHECK_ARGUMENT_FOR_NULL(pub_data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rmw_event->implementation_identifier	= rmw_get_implementation_identifier();
  rmw_event->data			= (void *)&pub_data->data_event_mgr;
  rmw_event->event_type			= event_type;

  return RMW_RET_OK;
}

void data_callback_init(DataEventManager *data_callback)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  z_mutex_init(&data_callback->mutex);
  data_callback->callback = NULL;
  data_callback->user_data = NULL;
  data_callback->unread_count = 0;
  // sizeof(EventStatus) (one entry) used to be passed here instead of
  // sizeof(data_callback->event_status) (the whole EVENT_TABLE_SIZE-entry
  // array) -- only event_status[0] ever got zeroed, every other slot
  // kept whatever garbage was already in this struct's memory.
  memset(data_callback->event_status, 0, sizeof(data_callback->event_status));
  // A zeroed last_policy_kind is not a valid rmw_qos_policy_kind_t
  // (every real value in qos_policy_kind.h is a nonzero bit flag) --
  // rclpy's default incompatible-QoS callback reads it unconditionally
  // once such an event is taken, and raises ValueError on a raw 0
  // instead of a real value. RMW_QOS_POLICY_INVALID is qos_policy_kind.h's
  // own designated "no specific policy" value -- a safe, real default.
  for (size_t index = 0; index < EVENT_TABLE_SIZE; index++) {
    data_callback->event_status[index].last_policy_kind = RMW_QOS_POLICY_INVALID;
  }
}

void data_callback_set(DataEventManager *data_callback,
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
      memset(data_callback->event_status, 0, sizeof(data_callback->event_status));
    }
    data_callback->user_data = user_data;
    data_callback->callback  = callback;

  }else{
    data_callback->user_data = NULL;
    data_callback->callback  = NULL;
  }

  z_mutex_unlock(z_loan_mut(data_callback->mutex));
}

void data_callback_trigger(DataEventManager *data_callback)
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

bool
rmw_event_type_is_supported(rmw_event_type_t rmw_event_type)
{
  return support_event_type(rmw_event_type) == -1 ? false : true;
}

rmw_ret_t
rmw_take_event(const rmw_event_t * event_handle,
	       void * event_info,
	       bool * taken)
{
  RMW_ZENOH_FUNC_ENTRY(event_handle);

  RMW_CHECK_ARGUMENT_FOR_NULL(event_handle, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(event_handle->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(event_info, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  *taken = false;

  rmw_event_type_t event_type = event_handle->event_type;
  DataEventManager *event_mgr = (DataEventManager *)event_handle->data;
  EventStatus *st = take_event_status(event_mgr, event_type);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    st,
    "Publisher implementation identifier not from this implementation",
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // event_condition_check() (used by rmw_wait() to decide whether a
  // wait-set entry is "ready") checks whether *any* of this shared
  // DataEventManager's event types changed, not specifically the one
  // `event_handle` represents here -- a publisher/subscription with more
  // than one registered rmw_event_t (e.g. a default incompatible-QoS
  // handler alongside a real MATCHED status change) had every sibling
  // handle spuriously reported "ready" whenever *any* of them changed.
  // This function used to compound that by never checking its own
  // type's `changed` flag either, so a spurious wakeup for -- say --
  // OFFERED_QOS_INCOMPATIBLE would still unconditionally report
  // total_count=0/last_policy_kind=RMW_QOS_POLICY_INVALID with
  // *taken=true, firing the caller's incompatible-QoS callback for an
  // incident that never happened. Gate on this event's own `changed`
  // flag first: a genuine change still reports normally below (and the
  // switch cases each clear `changed` once taken); nothing to report
  // otherwise -- standard rmw_take_event() semantics.
  z_mutex_lock(z_loan_mut(event_mgr->mutex));
  bool has_changed = st->changed;
  z_mutex_unlock(z_loan_mut(event_mgr->mutex));
  if (!has_changed) {
    return RMW_RET_OK;
  }

  switch (event_type) {
  case RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE:
  {
    rmw_requested_qos_incompatible_event_status_t *ei = (rmw_requested_qos_incompatible_event_status_t *)event_info;
    ei->total_count = st->total_count;
    ei->total_count_change = st->total_count_change;
    // last_policy_kind used to be left unset here -- rclpy's own default
    // incompatible-QoS callback (event_handler.py's
    // _default_incompatible_qos_callback -> qos.py's
    // qos_policy_kind_from_kind()) reads it unconditionally once this
    // event is taken, and raised ValueError on whatever uninitialized
    // garbage happened to be there instead of a real
    // rmw_qos_policy_kind_t value.
    ei->last_policy_kind = st->last_policy_kind;
    st->changed = false;
    *taken = true;
    return RMW_RET_OK;
  }

  case RMW_EVENT_OFFERED_QOS_INCOMPATIBLE:
  {
    rmw_requested_qos_incompatible_event_status_t *ei = (rmw_requested_qos_incompatible_event_status_t *)event_info;
    ei->total_count = st->total_count;
    ei->total_count_change = st->total_count_change;
    ei->last_policy_kind = st->last_policy_kind;
    st->changed = false;
    *taken = true;
    return RMW_RET_OK;
  }

  case RMW_EVENT_MESSAGE_LOST:
  {
    rmw_message_lost_status_t *ei = (rmw_message_lost_status_t *)event_info;
    ei->total_count = st->total_count;
    ei->total_count_change = st->total_count_change;
    st->changed = false;
    *taken = true;
    return RMW_RET_OK;
  }
  case RMW_EVENT_SUBSCRIPTION_MATCHED:
  case RMW_EVENT_PUBLICATION_MATCHED:
  {
    rmw_matched_status_t *ei = (rmw_matched_status_t *)event_info;
    ei->total_count = st->total_count;
    ei->total_count_change = st->total_count_change;
    ei->current_count = st->current_count;
    ei->current_count_change = st->current_count_change;
    st->changed = false;
    *taken = true;
    return RMW_RET_OK;
  }

  case RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE:
  case RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE:
  {
    rmw_incompatible_type_status_t *ei = (rmw_incompatible_type_status_t *)event_info;
    ei->total_count = st->total_count;
    ei->total_count_change = st->total_count_change;
    st->changed = false;
    *taken = true;
    return RMW_RET_OK;
  }
  case RMW_EVENT_LIVELINESS_LOST:
  case RMW_EVENT_LIVELINESS_CHANGED:
  case RMW_EVENT_REQUESTED_DEADLINE_MISSED:
  case RMW_EVENT_OFFERED_DEADLINE_MISSED:
  case RMW_EVENT_INVALID:
    break;
  }

  return RMW_RET_ERROR;
}
