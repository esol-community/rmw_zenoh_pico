#ifndef RMW_INIT_H
#define RMW_INIT_H

#include <stdio.h>
#include <stddef.h>
#include <unistd.h>

#include <zenoh-pico.h>

#include <rmw_microros/rmw_microros.h>


struct rmw_context_impl_s
{
  rmw_zenoh_pico_transport_params_t transport_param;
};
typedef struct rmw_context_impl_s rmw_context_impl_t;

struct rmw_zenoh_pico_session_s
{
  z_owned_config_t config;
  z_owned_session_t zid;
};
typedef struct rmw_zenoh_pico_session_s rmw_zenoh_pico_session_t;

// ----------------------

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

#else  // ZENOH_DEBUG != 0 || defined(Z_BUILD_DEBUG)

#define _Z_DEBUG(...)                          \
    do {                                       \
        if (ZENOH_DEBUG >= _Z_LOG_LVL_DEBUG) { \
            _Z_LOG_PREFIX(DEBUG);              \
            printf(__VA_ARGS__);               \
            printf("\r\n");                    \
        }                                      \
    } while (false)

#define _Z_INFO(...)                          \
    do {                                      \
        if (ZENOH_DEBUG >= _Z_LOG_LVL_INFO) { \
            _Z_LOG_PREFIX(INFO);              \
            printf(__VA_ARGS__);              \
            printf("\r\n");                   \
        }                                     \
    } while (false)

#define _Z_ERROR(...)                          \
    do {                                       \
        if (ZENOH_DEBUG >= _Z_LOG_LVL_ERROR) { \
            _Z_LOG_PREFIX(ERROR);              \
            printf(__VA_ARGS__);               \
            printf("\r\n");                    \
        }                                      \
    } while (false)

#endif /* ZENOH_DEBUG */
#endif /* RMW_INIT_H*/
