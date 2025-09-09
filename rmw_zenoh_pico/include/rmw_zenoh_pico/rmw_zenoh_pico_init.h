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

#ifndef RMW_ZENOH_PICO_INIT_H
#define RMW_ZENOH_PICO_INIT_H

#include <rmw_zenoh_pico/config.h>

#include <stdint.h>
#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

#define MAX_INET_DEVICE      50
#define MAX_SERIAL_DEVICE    50

  typedef struct _ZenohPicoTransportParams
  {
    int ref;
    z_owned_mutex_t lock;

    const char *mode;

#if defined(RMW_ZENOH_PICO_TRANSPORT_UNICAST)
    char connect_addr[MAX_INET_DEVICE];
    char listen_addr[MAX_INET_DEVICE];

#elif defined (RMW_ZENOH_PICO_TRANSPORT_MULTICAST)
    char mcast_addr[MAX_INET_DEVICE];

#elif defined (RMW_ZENOH_PICO_TRANSPORT_SERIAL)
    char serial_device[MAX_SERIAL_DEVICE];
#endif
  } ZenohPicoTransportParams;

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
