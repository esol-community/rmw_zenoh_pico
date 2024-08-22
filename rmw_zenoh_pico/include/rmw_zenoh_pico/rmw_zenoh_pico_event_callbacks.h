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
