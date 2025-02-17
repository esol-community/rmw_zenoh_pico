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

#ifndef RMW_ZENOH_PICO_CLIENT_H
#define RMW_ZENOH_PICO_CLIENT_H

#include <rmw/rmw.h>
#include <zenoh-pico.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico_node.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_rosMessage.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_event_callbacks.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_wait.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoClientData {
    int ref;

    size_t id;

    // Liveliness token for the client.
    z_owned_string_t liveliness_key;
    z_owned_string_t topic_key;

    z_owned_liveliness_token_t token;

    // this node
    ZenohPicoNodeData *node;

    // this client entity
    ZenohPicoEntity *entity;

    // CDR callback
    const message_type_support_callbacks_t *request_callback;
    const message_type_support_callbacks_t *response_callback;

    // Deque to store the replies in the order they arrive.
    ReceiveMessageDataList reply_queue;

    // data callback on new message
    DataCallbackManager data_callback_mgr;

    // Wait set data/rmw_wait condition
    z_owned_mutex_t condition_mutex;
    ZenohPicoWaitSetData * wait_set_data;

    // attachment data [sequence_num, last timestamp, topic gid]
    zenoh_pico_attachemt_data attachment;

    // value of qos
    rmw_qos_profile_t qos_profile;

    // Internal mutex.
    z_owned_mutex_t mutex;

  } ZenohPicoClientData;

  extern bool client_condition_check_and_attach(ZenohPicoClientData *sub_data,
						ZenohPicoWaitSetData * wait_set_data);
  extern bool client_condition_detach_and_queue_is_empty(ZenohPicoClientData *sub_data);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
