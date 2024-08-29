/*
 * Copyright (C)
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

#ifndef RMW_ZEONH_PICO_H
#define RMW_ZEONH_PICO_H

#include <rmw_zenoh_pico/config.h>

#include <stddef.h>
#include <unistd.h>
#include <ctype.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>
#include <time.h>

#include <rmw/rmw.h>
#include <rmw/ret_types.h>
#include <rmw/init_options.h>

#include <ucdr/microcdr.h>

#include "zenoh-pico/system/platform-common.h"

// utility
#include <rmw_zenoh_pico/rmw_zenoh_pico_macros.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_logging.h>

// internal data
#include <rmw_zenoh_pico/liveliness/rmw_zenoh_pico_nodeInfo.h>
#include <rmw_zenoh_pico/liveliness/rmw_zenoh_pico_topicInfo.h>
#include <rmw_zenoh_pico/liveliness/rmw_zenoh_pico_entity.h>
#include <rmw_zenoh_pico/liveliness/rmw_zenoh_pico_liveliness.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico_receiveMessage.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_messageType.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico_identifiers.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_param.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_session.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico_node.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_subscription.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_wait.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_guard_condition.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_event_callbacks.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_publisher.h>

// expand zenoh-pico api
extern int8_t z_condvar_wait_time(z_condvar_t *cv, z_mutex_t *m, struct timespec *wait_timeout);
extern void test_qos_profile(rmw_qos_profile_t *qos);

#endif
