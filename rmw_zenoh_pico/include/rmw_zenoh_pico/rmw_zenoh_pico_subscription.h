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

#ifndef RMW_ZENOH_PICO_SUBSCRIPTION_H
#define RMW_ZENOH_PICO_SUBSCRIPTION_H

#include "zenoh-pico/api/types.h"
#include <rmw/rmw.h>
#include <zenoh-pico.h>

#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_nodeInfo.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_entity.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_liveliness.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico_node.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_rosMessage.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_event_callbacks.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_wait.h>

#define ROS2_MSG_OFFSET 4

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoSubData {
    int ref;

    size_t id;

    // Liveliness token for the subscriber.
    z_owned_string_t token_key;
    z_owned_liveliness_token_t token;

    // the subscriber topic.
    z_owned_string_t topic_key;
    z_owned_subscriber_t subscriber;

    // this node
    ZenohPicoNodeData *node;

    // this subscribe entity
    ZenohPicoEntity *entity;

    // Store the actual QoS profile used to configure this subscription.
    rmw_qos_profile_t adapted_qos_profile;

    // CDR callback
    const message_type_support_callbacks_t *callbacks;

    // recived message list
    ReceiveMessageDataList message_queue;

    // data callback on new message
    DataCallbackManager data_callback_mgr;

    // rmw_wait condition
    z_owned_mutex_t condition_mutex;
    ZenohPicoWaitSetData * wait_set_data;

  } ZenohPicoSubData;

  extern bool subscription_condition_check_and_attach(ZenohPicoSubData *sub_data,
						      ZenohPicoWaitSetData * wait_set_data);
  extern bool subscription_condition_detach_and_queue_is_empty(ZenohPicoSubData *sub_data);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
