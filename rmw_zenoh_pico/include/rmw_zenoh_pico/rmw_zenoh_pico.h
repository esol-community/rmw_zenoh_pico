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

#ifndef RMW_ZEONH_PICO_H
#define RMW_ZEONH_PICO_H

#include <rmw_zenoh_pico/config.h>

#include <stddef.h>
#include <unistd.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include <time.h>
#include <string.h>

#include <rmw/rmw.h>
#include <rmw/ret_types.h>
#include <rmw/allocators.h>
#include <rmw/init_options.h>
#include <rmw/error_handling.h>
#include <rmw/validate_full_topic_name.h>
#include <rmw/names_and_types.h>
#include <rmw/get_network_flow_endpoints.h>
#include <rmw/get_topic_endpoint_info.h>

#include <rosidl_runtime_c/message_type_support_struct.h>

#include <rosidl_typesupport_microxrcedds_c/identifier.h>
#include <rosidl_typesupport_microxrcedds_c/message_type_support.h>

#include <ucdr/microcdr.h>

#include <zenoh-pico/system/platform.h>

#include <zenoh-pico.h>
#include <zenoh-pico/api/macros.h>
#include <zenoh-pico/api/primitives.h>
#include <zenoh-pico/api/types.h>

#include <zenoh-pico/collections/string.h>
#include <zenoh-pico/system/common/platform.h>
#include <zenoh-pico/system/platform.h>
#include <zenoh-pico/utils/result.h>

// utility
#include <rmw_zenoh_pico/rmw_zenoh_pico_macros.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_logging.h>

// internal data
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_attach.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_condition.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_data.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_entity.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_gid.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_liveliness.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_nodeInfo.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_string.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_topicInfo.h>

// rmw data
#include <rmw_zenoh_pico/rmw_zenoh_pico_event.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_guard_condition.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_identifiers.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_init.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_logging.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_macros.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_messageType.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_node.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_publisher.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_rosMessage.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_service.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_session.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_subscription.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_wait.h>

// expand zenoh-pico api
extern void test_qos_profile(rmw_qos_profile_t *qos);

#endif
