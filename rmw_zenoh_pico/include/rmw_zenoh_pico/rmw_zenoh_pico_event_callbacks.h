/*
 * Copyright (C)
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

#ifndef RMW_ZENOH_PICO_EVENT_CALLBACKS_H
#define RMW_ZENOH_PICO_EVENT_CALLBACKS_H

#include <rmw/rmw.h>
#include <zenoh-pico.h>

typedef struct _DataCallbackManager {
  z_mutex_t mutext_;

  /// User callback that can be set via set_callback().
  rmw_event_callback_t callback_;

  /// User data that should be passed to the user callback.
  const void * user_data_;

  /// number of trigger requests made before the callback was set.
  size_t unread_count_;

} DataCallbackManager;

extern void data_callback_init(DataCallbackManager *data_callback);
extern void data_callback_set(DataCallbackManager *data_callback,
			      const void * user_data,
			      rmw_event_callback_t callback);
extern void data_callback_trigger(DataCallbackManager *data_callback);

#endif
