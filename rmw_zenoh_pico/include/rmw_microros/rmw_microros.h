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

#ifdef RMW_UXRCE_TRANSPORT_IPV4
  #define MAX_IP_LEN         16
#elif defined(RMW_UXRCE_TRANSPORT_IPV6)
  #define MAX_IP_LEN         39
#endif  // ifdef RMW_UXRCE_TRANSPORT_IPV4
#define MAX_PORT_LEN         5
#define MAX_SERIAL_DEVICE    50

typedef struct rmw_zenoh_pico_transport_params_t
{
#if defined(RMW_ZENOH_PICO_TRANSPORT_SERIAL)
  char serial_device[MAX_SERIAL_DEVICE];
#elif defined(RMW_ZENOH_PICO_TRANSPORT_TCP)
  char session_address[MAX_IP_LEN];
  char session_port[MAX_PORT_LEN];
  bool enable_mcast;
#elif defined(RMW_RMW_ZENOH_PICO_TRANSPORT_CUSTOM)
  bool framing;
  void * args;
  open_custom_func open_cb;
  close_custom_func close_cb;
  write_custom_func write_cb;
  read_custom_func read_cb;
#endif  // if defined(RMW_RMW_ZENOH_PICO_TRANSPORT_SERIAL)
  uint32_t client_key;
} rmw_zenoh_pico_transport_params_t;

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif  // RMW_MICROROS__RMW_MICROROS_H_
