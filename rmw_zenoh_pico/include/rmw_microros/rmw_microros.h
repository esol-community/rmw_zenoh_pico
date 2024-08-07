// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RMW_MICROROS__RMW_MICROROS_H_
#define RMW_MICROROS__RMW_MICROROS_H_

#include <rmw_zenoh_pico/config.h>

#include <rmw/rmw.h>
#include <rmw/ret_types.h>
#include <rmw/init_options.h>

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

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif  // RMW_MICROROS__RMW_MICROROS_H_
