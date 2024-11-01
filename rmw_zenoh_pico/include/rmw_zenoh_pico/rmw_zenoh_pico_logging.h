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
#include <unistd.h>

#include <rmw/rmw.h>
#include <rmw/ret_types.h>
#include <rmw/init_options.h>

#include <zenoh-pico.h>

// Timestamp function
static inline void __z_log_prefix(const char *prefix, const char *func_name) {
  char time_stamp[64];

  z_time_t tv = z_time_now();
  snprintf(time_stamp, sizeof(time_stamp), "%ld.%09ld", tv.tv_sec, tv.tv_usec);

  printf("[%s] [%s] [%s]:",  prefix, time_stamp, func_name);
}

// Logging values
#define _Z_LOG_LVL_ERROR 1
#define _Z_LOG_LVL_INFO  2
#define _Z_LOG_LVL_DEBUG 3

extern void rmw_zenoh_pico_debug_level_inir(void);
extern int rmw_zenoh_pico_debug_level_get(void);

// Logging macros
#define _Z_LOG_PREFIX(prefix)    __z_log_prefix(#prefix, __func__)

// Ignore print only if log deactivated and build is release
#ifndef ZENOH_DEBUG_ENABLE

#define _Z_DEBUG(...) (void)(0)
#define _Z_INFO(...) (void)(0)
#define _Z_ERROR(...) (void)(0)

#define RMW_ZENOH_LOG_DEBUG (void)(0)
#define RMW_ZENOH_LOG_INFO (void)(0)
#define RMW_ZENOH_LOG_ERROR (void)(0)

#define RMW_ZENOH_FUNC_ENTRY() (void)(0)

#else

#define _Z_DEBUG(...)						\
  do {								\
    if (rmw_zenoh_pico_debug_level_get() >= _Z_LOG_LVL_DEBUG) {	\
      _Z_LOG_PREFIX(DEBUG);					\
      printf(__VA_ARGS__);					\
      printf("\r\n");						\
    }								\
  } while (false)

#define _Z_INFO(...)						\
  do {								\
    if (rmw_zenoh_pico_debug_level_get() >= _Z_LOG_LVL_INFO) {	\
      _Z_LOG_PREFIX(INFO);					\
      printf(__VA_ARGS__);					\
      printf("\r\n");						\
    }								\
  } while (false)

#define _Z_ERROR(...)						\
  do {								\
    if (rmw_zenoh_pico_debug_level_get() >= _Z_LOG_LVL_ERROR) {	\
      _Z_LOG_PREFIX(ERROR);					\
      printf(__VA_ARGS__);					\
      printf("\r\n");						\
    }								\
  } while (false)

#define RMW_ZENOH_LOG_DEBUG(...) _Z_DEBUG(__VA_ARGS__)
#define RMW_ZENOH_LOG_INFO(...)  _Z_INFO(__VA_ARGS__)
#define RMW_ZENOH_LOG_ERROR(...) _Z_ERROR(__VA_ARGS__)

#define RMW_ZENOH_FUNC_ENTRY() RMW_ZENOH_LOG_DEBUG("start()")

#endif /* ! ZENOH_DEBUG_ENABLE */

#endif
