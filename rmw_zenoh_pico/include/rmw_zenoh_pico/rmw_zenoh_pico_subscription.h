/*
 * Copyright (C) 2024 eSOL Co., Ltd.
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

#include <rmw/rmw.h>
#include <zenoh-pico.h>

#include <rmw_zenoh_pico/liveliness/rmw_zenoh_pico_nodeInfo.h>
#include <rmw_zenoh_pico/liveliness/rmw_zenoh_pico_entity.h>
#include <rmw_zenoh_pico/liveliness/rmw_zenoh_pico_liveliness.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico_node.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_receiveMessage.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_event_callbacks.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_wait.h>

#define SUB_MSG_OFFSET 4

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoSubData {
    int ref_;

    size_t id_;

    // Liveliness key for the subscriber.
    _z_string_t topic_key_;
    _z_string_t token_key_;

    // Liveliness token for the subscriber.
    z_owned_subscriber_t subscriber_;
    z_owned_subscriber_t token_;

    // this node
    ZenohPicoNodeData *node_;

    // this subscribe entity
    ZenohPicoEntity *entity_;

    // Store the actual QoS profile used to configure this subscription.
    rmw_qos_profile_t adapted_qos_profile_;

    // CDR callback
    const message_type_support_callbacks_t *callbacks_;

    // recived message list
    ReceiveMessageDataList message_queue_;

    // data callback on new message
    DataCallbackManager data_callback_mgr;

    // rmw_wait condition
    z_mutex_t condition_mutex;
    ZenohPicoWaitSetData * wait_set_data_;

  } ZenohPicoSubData;

  extern ZenohPicoSubData * zenoh_pico_generate_subscription_data(
    size_t sub_id,
    ZenohPicoNodeData *node,
    ZenohPicoEntity *entity,
    const rosidl_message_type_support_t * type_support,
    const message_type_support_callbacks_t *callbacks,
    rmw_qos_profile_t *qos_profile);

  extern bool zenoh_pico_destroy_subscription_data(ZenohPicoSubData *sub_data);
  extern void zenoh_pico_debug_subscription_data(ZenohPicoSubData *sub_data);

  extern bool declaration_subscription_data(ZenohPicoSubData *sub_data);
  extern bool undeclaration_subscription_data(ZenohPicoSubData *sub_data);

  extern void subscription_condition_trigger(ZenohPicoSubData *sub_data);
  extern bool subscription_condition_check_and_attach(ZenohPicoSubData *sub_data,
						      ZenohPicoWaitSetData * wait_set_data);
  extern bool subscription_condition_detach_and_queue_is_empty(ZenohPicoSubData *sub_data);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
