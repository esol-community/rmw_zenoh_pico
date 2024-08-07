#ifndef RMW_ZENOH_PICO_DATA_TYPES_H
#define RMW_ZENOH_PICO_DATA_TYPES_H

#include <rmw_zenoh_pico/config.h>

#include <stddef.h>
#include <unistd.h>

#include <rmw/rmw.h>
#include <rmw/ret_types.h>
#include <rmw/init_options.h>

#include <zenoh-pico.h>

#ifdef RMW_UROS_ERROR_HANDLING
#include <rmw_microros/error_handling.h>
#endif  // RMW_UROS_ERROR_HANDLING

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

#define MAX_INET_DEVICE      50
#define MAX_SERIAL_DEVICE    50

  struct rmw_zenoh_pico_transport_params_s
  {
    const char *mode;

#if defined(RMW_ZENOH_PICO_TRANSPORT_UNICAST)
    char connect_addr[MAX_INET_DEVICE];
    char listen_addr[MAX_INET_DEVICE];

#elif defined (RMW_ZENOH_PICO_TRANSPORT_MCAST)
    char locator_addr[MAX_INET_DEVICE];
    char listen_addr[MAX_INET_DEVICE];

#elif defined (RMW_ZENOH_PICO_TRANSPORT_SERIAL)
    char serial_device[MAX_SERIAL_DEVICE];
#endif
  };
  typedef struct rmw_zenoh_pico_transport_params_s rmw_zenoh_pico_transport_params_t;

  struct rmw_context_impl_s
  {
    rmw_zenoh_pico_transport_params_t transport_param;
  };
  typedef struct rmw_context_impl_s rmw_context_impl_t;

  struct rmw_zenoh_pico_session_s
  {
    // configuration data array
    z_owned_config_t config;

    // An owned session.
    z_owned_session_t session;

  };
  typedef struct rmw_zenoh_pico_session_s rmw_zenoh_pico_session_t;

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif /* RMW_ZENOH_PICO_DATA_TYPES_H */
