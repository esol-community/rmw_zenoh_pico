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

void wait_condition_lock(ZenohPicoWaitSetData * wait_set_data)
{
  z_mutex_lock(z_loan_mut(wait_set_data->condition_mutex));
}

void wait_condition_unlock(ZenohPicoWaitSetData * wait_set_data)
{
  z_mutex_unlock(z_loan_mut(wait_set_data->condition_mutex));
}

void wait_condition_signal(ZenohPicoWaitSetData * wait_set_data)
{
  z_condvar_signal(z_loan_mut(wait_set_data->condition_variable));
}

void wait_condition_triggered(ZenohPicoWaitSetData * wait_set_data, bool value)
{
  wait_set_data->triggered = value;
}

static bool _check_and_attach_condition(const rmw_subscriptions_t * const subscriptions,
					const rmw_guard_conditions_t * const guard_conditions,
					const rmw_services_t * const services,
					const rmw_clients_t * const clients,
					const rmw_events_t * const events,
					ZenohPicoWaitSetData * wait_set_data)
{
  if(guard_conditions){
    for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
      ZenohPicoGuardConditionData *gc = (ZenohPicoGuardConditionData *)guard_conditions->guard_conditions[i];
      if(gc == NULL){
	continue;
      }

      if(guard_condition_check_and_attach(gc, wait_set_data)) {
	// RMW_ZENOH_LOG_INFO("found attach guard_conditions");
	return true;
      }
    }
  }

  if(events) {
    for (size_t i = 0; i < events->event_count; ++i) {
      DataEventManager *event_mgr = (DataEventManager *)events->events[i];
      if (event_mgr == NULL) {
        continue;
      }

      if(event_condition_check_and_attach(event_mgr, wait_set_data)) {
	// RMW_ZENOH_LOG_INFO("found attach event");
	return true;
      }
    }
  }

  if (subscriptions) {
    for (size_t i = 0; i < subscriptions->subscriber_count; ++i) {
      ZenohPicoSubData *sub_data = (ZenohPicoSubData *)subscriptions->subscribers[i];
      if (sub_data == NULL) {
        continue;
      }

      if(subscription_condition_check_and_attach(sub_data, wait_set_data)) {
	// RMW_ZENOH_LOG_INFO("found attach subscriptions");
	return true;
      }
    }
  }

  if (services) {
    for (size_t i = 0; i < services->service_count; ++i) {
      ZenohPicoServiceData *service_data = (ZenohPicoServiceData *) services->services[i];
      if (service_data == NULL) {
        continue;
      }

      if(service_condition_check_and_attach(service_data, wait_set_data)) {
	// RMW_ZENOH_LOG_INFO("found attach services");
	return true;
      }
    }
  }

  if (clients) {
    for (size_t i = 0; i < clients->client_count; ++i) {
      ZenohPicoServiceData *client_data = (ZenohPicoServiceData *)clients->clients[i];
      if (client_data == NULL) {
        continue;
      }

      if(client_condition_check_and_attach(client_data, wait_set_data)) {
	// RMW_ZENOH_LOG_INFO("found attach clients");
	return true;
      }
    }
  }

  return false;
}

rmw_ret_t
rmw_wait(rmw_subscriptions_t * subscriptions,
	 rmw_guard_conditions_t * guard_conditions,
	 rmw_services_t * services,
	 rmw_clients_t * clients,
	 rmw_events_t * events,
	 rmw_wait_set_t * wait_set,
	 const rmw_time_t * wait_timeout)
{
  // RMW_ZENOH_FUNC_ENTRY(NULL);

  RMW_CHECK_ARGUMENT_FOR_NULL(wait_set, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    wait_set->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  ZenohPicoWaitSetData *wait_set_data = (ZenohPicoWaitSetData *)wait_set->data;
  RMW_CHECK_FOR_NULL_WITH_MSG(
    wait_set_data,
    "waitset data struct is null",
    return RMW_RET_ERROR);

#if Z_FEATURE_MULTI_THREAD == 0
  // _check_and_attach_condition() below can find a guard condition/timer
  // already ready (e.g. rclpy's executor attaches one whenever a periodic
  // timer is pending) and return true *before* the network-pumping loop
  // further down ever runs -- that loop lives entirely inside the
  // `if(!skip_wait)` branch below, so on every rmw_wait() call where
  // something is already ready, zp_read()/zp_send_keep_alive() never get
  // called at all. With no background thread (Z_FEATURE_MULTI_THREAD==0)
  // nothing else ever pumps the session, so a talker whose only waitable is
  // a repeating timer (the common case -- confirmed via rclpy's talker,
  // where this starved the session of any traffic until the router's
  // 10-second lease expired and force-closed the transport) can go
  // completely silent on the wire forever, well before any real timeout.
  // Pumping once here, unconditionally, guarantees the network gets
  // serviced on every single call regardless of skip_wait.
  ZenohPicoSession *__pump_session = (ZenohPicoSession *)wait_set_data->context->impl;
  const z_loaned_session_t *__pump_zsession = z_loan(__pump_session->session);
  (void)zp_read(__pump_zsession, NULL);
  (void)zp_send_keep_alive(__pump_zsession, NULL);
#endif

  bool skip_wait = _check_and_attach_condition(
    subscriptions, guard_conditions, services, clients, events, wait_set_data);

  if(!skip_wait){
#if Z_FEATURE_MULTI_THREAD == 0
    // No background read thread exists to receive data and signal this
    // wait-set (that's a real OS thread in the Z_FEATURE_MULTI_THREAD==1
    // build, started automatically by zenoh-pico's session init) -- so
    // nothing will ever make wait_set_data->triggered become true unless
    // *we* pump the session ourselves. zp_read()/zp_send_keep_alive() are
    // the documented single-threaded-mode replacement for that background
    // thread (see zenoh-pico's "Single Thread helpers").
    //
    // A synchronous call on this platform can never usefully block longer
    // than a single attempt: there's no way to cooperatively yield back to
    // the browser's event loop from inside one call without Asyncify, so a
    // "sleep between polls" loop for a nonzero requested timeout would
    // just spin with no real delay and no yield -- freezing the whole
    // page for up to that timeout, or forever if wait_timeout is NULL.
    // So this always polls exactly once and returns immediately,
    // regardless of what timeout was requested -- the caller is expected
    // to retry across separate top-level calls, with a real yield in
    // between (e.g. a JS-driven tick or Python's own `await
    // asyncio.sleep()`), the same way session_connect() below already
    // has to.
    ZenohPicoSession *rmw_session = (ZenohPicoSession *)wait_set_data->context->impl;
    const z_loaned_session_t *zsession = z_loan(rmw_session->session);

    (void)zp_read(zsession, NULL);
    (void)zp_send_keep_alive(zsession, NULL);

    wait_set_data->triggered = false;
#else
    z_loaned_mutex_t *lock = z_loan_mut(wait_set_data->condition_mutex);
    z_loaned_condvar_t *cv = z_loan_mut(wait_set_data->condition_variable);

    z_mutex_lock(lock);

    if (wait_timeout == NULL) {
      while(!wait_set_data->triggered)
	z_condvar_wait(cv, lock);

    }else{
      if (wait_timeout->sec != 0 || wait_timeout->nsec != 0) {
	struct timespec timeout;

	memset(&timeout, 0, sizeof(struct timespec));
	timeout.tv_sec = wait_timeout->sec;
	timeout.tv_nsec = wait_timeout->nsec;

	(void)z_condvar_timewait(cv, lock, &timeout);
      }
    }

    /* RMW_ZENOH_LOG_DEBUG("wakeup from wait condition...."); */

    wait_set_data->triggered = false;

    z_mutex_unlock(lock);
#endif  // Z_FEATURE_MULTI_THREAD == 0
  }

  bool wait_result = false;

  if(guard_conditions){
    for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
      ZenohPicoGuardConditionData *gc = (ZenohPicoGuardConditionData *)guard_conditions->guard_conditions[i];
      if(gc == NULL){
	continue;
      }

      if(!guard_condition_detach_and_is_trigger_set(gc)){
	guard_conditions->guard_conditions[i] = NULL;
      }else{
	wait_result = true;
      }
    }
  }

  if(events) {
    for (size_t i = 0; i < events->event_count; ++i) {
      for (size_t i = 0; i < events->event_count; ++i) {
	DataEventManager *event_mgr = (DataEventManager *)events->events[i];
	if (event_mgr == NULL) {
	  continue;
	}

	if(event_condition_detach_and_queue_is_empty(event_mgr)) {
	  // RMW_ZENOH_LOG_INFO("found attach event");
	  return true;
	}
      }
    }
  }

  if (subscriptions) {
    for (size_t i = 0; i < subscriptions->subscriber_count; ++i) {
      ZenohPicoSubData *sub_data = (ZenohPicoSubData *)subscriptions->subscribers[i];
      if (sub_data == NULL) {
	continue;
      }

      if(subscription_condition_detach_and_queue_is_empty(sub_data)){
	// Setting to NULL lets rcl know that this subscription is not ready
        subscriptions->subscribers[i] = NULL;
      }else{
        wait_result = true;
      }
    }
  }

  if (services) {
    for (size_t i = 0; i < services->service_count; ++i) {
      ZenohPicoServiceData *service_data = (ZenohPicoServiceData *)services->services[i];
      if (service_data == NULL) {
	continue;
      }

      if(service_condition_detach_and_queue_is_empty(service_data)){
	// Setting to NULL lets rcl know that this client is not ready
        services->services[i] = NULL;
      }else{
        wait_result = true;
      }
    }
  }

  if (clients) {
    for (size_t i = 0; i < clients->client_count; ++i) {
      ZenohPicoServiceData *client_data = (ZenohPicoServiceData *)clients->clients[i];
      if (client_data == NULL) {
	continue;
      }

      if(client_condition_detach_and_queue_is_empty(client_data)){
	// Setting to NULL lets rcl know that this client is not ready
        clients->clients[i] = NULL;
      }else{
        wait_result = true;
      }
    }
  }

  return wait_result ? RMW_RET_OK : RMW_RET_TIMEOUT;
}

ZenohPicoWaitSetData * zenoh_pico_generate_wait_set_data(rmw_context_t * context)
{
  RMW_ZENOH_FUNC_ENTRY(context);

  ZenohPicoWaitSetData *wait_data = NULL;
  wait_data = ZenohPicoDataGenerate(wait_data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    wait_data,
    "failed to allocate struct for the ZenohPicoWaitSetData",
    return NULL);

  z_mutex_init(&wait_data->condition_mutex);
  (void)z_condvar_init_with_attr(&wait_data->condition_variable, NULL);

  wait_data->triggered = false;

  wait_data->context = context;

  return wait_data;
}

bool zenoh_pico_destroy_wait_set_data(ZenohPicoWaitSetData *wait_data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  if(ZenohPicoDataRelease(wait_data)){
    z_mutex_drop(z_move(wait_data->condition_mutex));
    z_condvar_drop(z_move(wait_data->condition_variable));

    ZenohPicoDataDestroy(wait_data);
  }

  return true;
}

rmw_wait_set_t *
rmw_create_wait_set(rmw_context_t * context, size_t max_conditions)
{
  (void)max_conditions;

  RMW_ZENOH_FUNC_ENTRY(context);

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
  RMW_ZENOH_FUNC_ENTRY(wait_set);

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
