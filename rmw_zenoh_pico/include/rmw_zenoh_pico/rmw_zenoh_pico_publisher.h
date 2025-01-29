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

#ifndef RMW_ZENOH_PICO_PUBLISHER_H
#define RMW_ZENOH_PICO_PUBLISHER_H

#include <rmw/rmw.h>
#include <stdint.h>
#include <zenoh-pico.h>

#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_nodeInfo.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_entity.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_liveliness.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_attach.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico_node.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoPubData {
    int ref;

    size_t id;

    // Liveliness key for the publisher.
    z_owned_string_t topic_key;
    z_owned_string_t token_key;

    // Liveliness token for the publisher.
    z_owned_publisher_t publisher;
    z_owned_subscriber_t token;

    // this node
    ZenohPicoNodeData *node;

    // this subscribe entity
    ZenohPicoEntity *entity;

    // CDR callback
    const message_type_support_callbacks_t *callbacks;

    // Store the actual QoS profile used to configure this publisher.
    rmw_qos_profile_t adapted_qos_profile;

    // publish mutex
    z_owned_mutex_t mutex;

    // attachment data [sequence_num, last timestamp, topic gid]
    zenoh_pico_attachemt_data attachment;

  } ZenohPicoPubData;

  extern ZenohPicoPubData * zenoh_pico_generate_publisher_data(
    size_t pub_id,
    ZenohPicoNodeData *node,
    ZenohPicoEntity *entity,
    const rmw_qos_profile_t *qos_profile,
    const rosidl_message_type_support_t * type_support,
    const message_type_support_callbacks_t *callbacks);
  extern bool zenoh_pico_destroy_publisher_data(ZenohPicoPubData *pub_data);
  extern void zenoh_pico_debug_publisher_data(ZenohPicoPubData *pub_data);
  extern bool declaration_publisher_data(ZenohPicoPubData *pub_data);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
