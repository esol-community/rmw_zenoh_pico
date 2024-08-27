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

#define ZENOH_DEBUG _Z_LOG_LVL_DEBUG

// Logging macros
#define _Z_LOG_PREFIX(prefix)    __z_log_prefix(#prefix, __func__)

// Ignore print only if log deactivated and build is release
#if ZENOH_DEBUG == 0

#define _Z_DEBUG(...) (void)(0)
#define _Z_INFO(...) (void)(0)
#define _Z_ERROR(...) (void)(0)

#define RMW_ZENOH_LOG_DEBUG_NAMED (void)(0)
#define RMW_ZENOH_LOG_INFO_NAMED (void)(0)
#define RMW_ZENOH_LOG_ERROR_NAMED (void)(0)

#define RMW_UROS_TRACE_MESSAGE(...) (void)(0)

#else  // ZENOH_DEBUG != 0 || defined(Z_BUILD_DEBUG)

#define RMW_UROS_TRACE_MESSAGE(...) (void)(0)

#define _Z_DEBUG(...)				\
  do {						\
    if (ZENOH_DEBUG >= _Z_LOG_LVL_DEBUG) {	\
      _Z_LOG_PREFIX(DEBUG);			\
      printf(__VA_ARGS__);			\
      printf("\r\n");				\
    }						\
  } while (false)

#define _Z_INFO(...)				\
  do {						\
    if (ZENOH_DEBUG >= _Z_LOG_LVL_INFO) {	\
      _Z_LOG_PREFIX(INFO);			\
      printf(__VA_ARGS__);			\
      printf("\r\n");				\
    }						\
  } while (false)

#define _Z_ERROR(...)				\
  do {						\
    if (ZENOH_DEBUG >= _Z_LOG_LVL_ERROR) {	\
      _Z_LOG_PREFIX(ERROR);			\
      printf(__VA_ARGS__);			\
      printf("\r\n");				\
    }						\
  } while (false)

#define RMW_ZENOH_LOG_DEBUG_NAMED(tag, ...)	\
  do {						\
    if (ZENOH_DEBUG >= _Z_LOG_LVL_ERROR) {	\
      _Z_LOG_PREFIX(DEBUG);			\
      printf("%s : ", tag);			\
      printf(__VA_ARGS__);			\
      printf("\r\n");				\
    }						\
  } while (false)

#define RMW_ZENOH_LOG_INFO_NAMED(tag, ...)	\
  do {						\
    if (ZENOH_DEBUG >= _Z_LOG_LVL_ERROR) {	\
      _Z_LOG_PREFIX(INFO);			\
      printf("%s : ", tag);			\
      printf(__VA_ARGS__);			\
      printf("\r\n");				\
    }						\
  } while (false)

#define RMW_ZENOH_LOG_ERROR_NAMED(tag, ...)	\
  do {						\
    if (ZENOH_DEBUG >= _Z_LOG_LVL_ERROR) {	\
      _Z_LOG_PREFIX(ERROR);			\
      printf("%s : ", tag);			\
      printf(__VA_ARGS__);			\
      printf("\r\n");				\
    }						\
  } while (false)

#endif /* ZENOH_DEBUG */



#endif
