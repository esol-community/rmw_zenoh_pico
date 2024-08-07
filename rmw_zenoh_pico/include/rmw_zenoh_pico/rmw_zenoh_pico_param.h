#ifndef RMW_ZENOH_PICO_PARAM_H
#define RMW_ZENOH_PICO_PARAM_H

#include <rmw_zenoh_pico/config.h>

#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

#define MAX_INET_DEVICE      50
#define MAX_SERIAL_DEVICE    50

  typedef struct _ZenohPicoTransportParams
  {
    bool alloc_;
    const char *mode_;

#if defined(RMW_ZENOH_PICO_TRANSPORT_UNICAST)
    char connect_addr_[MAX_INET_DEVICE];
    char listen_addr_[MAX_INET_DEVICE];

#elif defined (RMW_ZENOH_PICO_TRANSPORT_MCAST)
    char locator_addr_[MAX_INET_DEVICE];
    char listen_addr_[MAX_INET_DEVICE];

#elif defined (RMW_ZENOH_PICO_TRANSPORT_SERIAL)
    char serial_device_[MAX_SERIAL_DEVICE];
#endif
  } ZenohPicoTransportParams;

  extern ZenohPicoTransportParams *zenoh_pico_generate_param(ZenohPicoTransportParams *param);
  extern bool zenoh_pico_destroy_param(ZenohPicoTransportParams *param);

  extern bool zenoh_pico_clone_param(ZenohPicoTransportParams *dst, ZenohPicoTransportParams *src);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
