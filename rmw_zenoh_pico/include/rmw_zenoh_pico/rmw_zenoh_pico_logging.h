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

#ifndef LOGGING_H
#define LOGGING_H

#include <stddef.h>
#include <string.h>
#include <unistd.h>

#include <rmw/rmw.h>
#include <rmw/ret_types.h>
#include <rmw/init_options.h>

#include <zenoh-pico.h>

extern void z_log_prefix(const char *prefix, const char *func_name);

// Logging values
#define _Z_LOG_LVL_ERROR 1
#define _Z_LOG_LVL_INFO  2
#define _Z_LOG_LVL_DEBUG 3

extern void rmw_zenoh_pico_debug_level_init(void);
extern int  rmw_zenoh_pico_debug_level_get(void);

extern bool rmw_zenoh_pico_check_validate_name(const char * name);

extern void rmw_zenoh_pico_log_init(void);
extern void rmw_zenoh_pico_log_lock(void);
extern void rmw_zenoh_pico_log_unlock(void);

// Logging macros

#ifdef _Z_LOG_PREFIX
#undef _Z_LOG_PREFIX
#endif

// Ignore print only if log deactivated and build is release

#ifdef _Z_DEBUG
#undef _Z_DEBUG
#endif

#ifdef _Z_INFO
#undef _Z_INFO
#endif

#ifdef _Z_ERROR
#undef _Z_ERROR
#endif

#ifndef ZENOH_DEBUG_ENABLE

#define _Z_DEBUG(...) (void)(0)
#define _Z_INFO(...) (void)(0)
#define _Z_ERROR(...) (void)(0)

#define RMW_ZENOH_LOG_DEBUG (void)(0)
#define RMW_ZENOH_LOG_INFO (void)(0)
#define RMW_ZENOH_LOG_ERROR (void)(0)

#define RMW_ZENOH_FUNC_ENTRY() (void)(0)

#else

#define _Z_DEBUG(f, ...)					\
  do {								\
    if (rmw_zenoh_pico_debug_level_get() >= _Z_LOG_LVL_DEBUG) {	\
      rmw_zenoh_pico_log_lock();				\
      z_log_prefix("DEBUG", f);					\
      printf(__VA_ARGS__);					\
      printf("\r\n");						\
      rmw_zenoh_pico_log_unlock();				\
    }								\
  } while (false)

#define _Z_INFO(f, ...)						\
  do {								\
    if (rmw_zenoh_pico_debug_level_get() >= _Z_LOG_LVL_INFO) {	\
      rmw_zenoh_pico_log_lock();				\
      z_log_prefix("INFO", f);					\
      printf(__VA_ARGS__);					\
      printf("\r\n");						\
      rmw_zenoh_pico_log_unlock();				\
    }								\
  } while (false)

#define _Z_ERROR(f, ...)					\
  do {								\
    if (rmw_zenoh_pico_debug_level_get() >= _Z_LOG_LVL_ERROR) {	\
      rmw_zenoh_pico_log_lock();				\
      z_log_prefix("ERROR", f);					\
      printf(__VA_ARGS__);					\
      printf("\r\n");						\
      rmw_zenoh_pico_log_unlock();				\
    }								\
  } while (false)

#define RMW_ZENOH_LOG_DEBUG(...) _Z_DEBUG(__func__, __VA_ARGS__)
#define RMW_ZENOH_LOG_INFO(...)  _Z_INFO(__func__, __VA_ARGS__)
#define RMW_ZENOH_LOG_ERROR(...) _Z_ERROR(__func__, __VA_ARGS__)

static inline void __entry_log_ptr(const void * v, const char *func) {
  if(v == NULL)
    _Z_DEBUG(func, "start()");
  else
    _Z_DEBUG(func, "start(%p)", v);
}

static inline void __entry_log_char(const char * v, const char *func) {
  _Z_DEBUG(func,"start(%s)", v);
}

static inline void __entry_log_context(const rmw_context_t * v, const char *func) {
  _Z_DEBUG(func,"start(%p)", v);
}

static inline void __entry_log_node(const rmw_node_t * v, const char *func) {
  _Z_DEBUG(func,"start(%p)", v->context);
  _Z_INFO(func, "node->identifier = %s", v->implementation_identifier);
  _Z_INFO(func, "node->namespace  = %s", v->namespace_);
  _Z_INFO(func, "node->data       = %p", v->data);
}

static inline void __entry_log_subscription(const rmw_subscription_t * v, const char *func) {
  _Z_DEBUG(func,"start(%s)", v->topic_name);
}

static inline void __entry_log_publisher(const rmw_publisher_t * v, const char *func) {
  _Z_DEBUG(func,"start(%s)", v->topic_name);
}

static inline void __entry_log_event(const rmw_event_t * v, const char *func) {
  _Z_DEBUG(func,"start(%s)", v->implementation_identifier);
}

static inline void __entry_log_service(const rmw_service_t * v, const char *func) {
  _Z_DEBUG(func,"start(%s)", v->service_name);
}//

static inline void __entry_log_client(const rmw_client_t * v, const char *func) {
  _Z_DEBUG(func,"start(%s)", v->service_name);
}

static inline void __entry_log_guard_condition(const rmw_guard_condition_t * v, const char *func) {
  _Z_DEBUG(func,"start(%s)", v->implementation_identifier);
}

static inline void __entry_log_init_options(const rmw_init_options_t * v, const char *func) {
  _Z_DEBUG(func,"start(%s)", v->implementation_identifier);
}

static inline void __entry_log_wait_set(const rmw_wait_set_t * v, const char *func) {
  _Z_DEBUG(func,"start(%s)", v->implementation_identifier);
}

static inline void __entry_log_loaned_string(const z_loaned_string_t * v, const char *func) {
  char _work[64];
  memset(_work, 0, sizeof(_work));
  size_t _len = z_string_len(v) >= sizeof(_work) -1 ? sizeof(_work) -1 : z_string_len(v);
  strncpy(_work, z_string_data(v), _len);

  _Z_DEBUG(func,"start(%s)", _work);
}

static inline void __entry_log_loaned_sample(const z_loaned_sample_t * v, const char *func) {
  const _z_keyexpr_t *key = &v->keyexpr;

  char _work[64];
  memset(_work, 0, sizeof(_work));
  size_t _len = _z_string_len(&key->_suffix) >= sizeof(_work) -1 ? sizeof(_work) -1 : _z_string_len(&key->_suffix);
  strncpy(_work, _z_string_data(&key->_suffix), _len);

  _Z_DEBUG(func,"start(%s)", _work);
}

#define ZENOH_DEBUG_FUNC_ENTRY_ENABLE
#ifdef ZENOH_DEBUG_FUNC_ENTRY_ENABLE
#define RMW_ZENOH_FUNC_ENTRY(v)						\
  _Generic((v),								\
	   const char *				: __entry_log_char,	\
	   const rmw_context_t *		: __entry_log_context,	\
	   const rmw_node_t *			: __entry_log_node,	\
	   const rmw_subscription_t *		: __entry_log_subscription, \
	   const rmw_publisher_t *		: __entry_log_publisher, \
	   const rmw_event_t *			: __entry_log_event,	\
	   const rmw_service_t *		: __entry_log_service,	\
	   const rmw_client_t *			: __entry_log_client,	\
	   const rmw_guard_condition_t *	: __entry_log_guard_condition, \
	   const rmw_init_options_t *		: __entry_log_init_options, \
	   const z_loaned_string_t *		: __entry_log_loaned_string, \
	   const z_loaned_sample_t *		: __entry_log_loaned_sample, \
	   default				: __entry_log_ptr	\
    )(v, __func__)

#else
#define RMW_ZENOH_FUNC_ENTRY() (void)(0)
#endif


#endif /* ! ZENOH_DEBUG_ENABLE */

#endif
