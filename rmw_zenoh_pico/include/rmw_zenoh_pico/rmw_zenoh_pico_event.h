/*
 * Copyright(C) 2024 eSOL Co., Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RMW_ZENOH_PICO_EVENT_H
#define RMW_ZENOH_PICO_EVENT_H

#include <rmw/rmw.h>
#include <stdint.h>
#include <zenoh-pico.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico_rosMessage.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_wait.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  // Note :
  // the current implement of rmw_zenoh_pico does not support event.
  // because, the gid_cache is not implemented on rmw_zenoh_pico.
  //
  static rmw_event_type_t _support_event[] = {
    // RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE,
    // RMW_EVENT_OFFERED_QOS_INCOMPATIBLE,
    // RMW_EVENT_MESSAGE_LOST,
    // RMW_EVENT_SUBSCRIPTION_MATCHED,
    // RMW_EVENT_PUBLICATION_MATCHED,
    // RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE,
    // RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE,
  };

  #define EVENT_TABLE_SIZE (sizeof(_support_event)/sizeof(rmw_event_type_t))

  typedef struct _EventStatus {
    size_t total_count;
    size_t total_count_change;
    size_t current_count;
    size_t current_count_change;
    bool changed;
  } EventStatus;

  typedef struct _DataEventManager {
    z_owned_mutex_t mutex;

    // User callback that can be set via set_callback().
    rmw_event_callback_t callback;

    // User data that should be passed to the user callback.
    const void * user_data;

    // number of trigger requests made before the callback was set.
    size_t unread_count;

    // event status
    EventStatus event_status[EVENT_TABLE_SIZE];

    // rmw_wait condition
    z_owned_mutex_t condition_mutex;
    ZenohPicoWaitSetData * wait_set_data;

  } DataEventManager;

  extern void add_rmw_zenoh_pico_event_total(DataEventManager *event_mgr, rmw_event_type_t type, bool change);
  extern void add_rmw_zenoh_pico_event_current(DataEventManager *event_mgr, rmw_event_type_t type, bool change);
  extern bool is_rmw_zenoh_pico_event_changed(DataEventManager *event_mgr, rmw_event_type_t type);

  extern bool event_condition_check_and_attach(DataEventManager *event_data,
					ZenohPicoWaitSetData *wait_set_data);
  extern bool event_condition_detach_and_queue_is_empty(DataEventManager *event_data);

  extern void data_callback_init(DataEventManager *data_callback);
  extern void data_callback_set(DataEventManager *data_callback,
				const void * user_data,
				rmw_event_callback_t callback);

  extern void data_callback_trigger(DataEventManager *data_callback);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
