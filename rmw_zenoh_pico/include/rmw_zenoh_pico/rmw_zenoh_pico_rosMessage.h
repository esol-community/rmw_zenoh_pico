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

#ifndef RMW_ZENOH_PICO_ROS_MESSAGE_H
#define RMW_ZENOH_PICO_ROS_MESSAGE_H

#include "zenoh-pico/api/types.h"
#include <rmw/rmw.h>
#include <zenoh-pico.h>

#include <rosidl_typesupport_microxrcedds_c/identifier.h>
#include <rosidl_typesupport_microxrcedds_c/message_type_support.h>

#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_attach.h>

typedef struct _ZenohPicoPubData ZenohPicoPubData;
typedef struct _ZenohPicoSubData ZenohPicoSubData;

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef enum _ZenohPicoMessageType {
    Sample,
    Query
  } ZenohPicoMessageType;

  typedef struct _ReceiveMessageData {
    struct _ReceiveMessageData *next;

    int ref;
    z_owned_mutex_t lock;

    void *payload_start;
    size_t payload_size;

    int64_t recv_timestamp;
    size_t sequence_num;

    // attachment data [sequence_num, last timestamp, topic gid]
    zenoh_pico_attachemt_data attachment;

    // type of my message
    ZenohPicoMessageType type;

    // used by rmw_take_request() and rmw_take_request()
    _z_query_rc_t      query;
    z_owned_keyexpr_t  keyexpr;

  } ReceiveMessageData;

  typedef struct _ReceiveMessageDataList {
    z_owned_mutex_t mutex;

    ReceiveMessageData *que_top;
    ReceiveMessageData *que_bottom;
    int count;
  } ReceiveMessageDataList;

  extern ReceiveMessageData * rmw_zenoh_pico_generate_recv_sample_msg_data(
    const z_loaned_sample_t *sample,
    uint64_t recv_ts);
  extern ReceiveMessageData * rmw_zenoh_pico_generate_recv_query_msg_data(
    const z_loaned_query_t *query,
    uint64_t recv_ts);

  extern bool zenoh_pico_delete_recv_msg_data(ReceiveMessageData * recv_data);

  extern void zenoh_pico_debug_dump_msg(const uint8_t *start, size_t size);
  extern void zenoh_pico_debug_recv_msg_data(ReceiveMessageData * recv_data);

  extern void rmw_zenoh_pico_debug_recv_msg_data(ReceiveMessageData * recv_data);

  extern void recv_msg_list_init(ReceiveMessageDataList *msg_list);
  extern ReceiveMessageData *recv_msg_list_append(ReceiveMessageDataList *msg_list,
						ReceiveMessageData *recv_data);
  extern ReceiveMessageData *recv_msg_list_pop(ReceiveMessageDataList *msg_list);
  extern ReceiveMessageData *recv_msg_list_pickup(ReceiveMessageDataList *msg_list,
						  bool (*func)(ReceiveMessageData *, const void *),
						  const void *data);
  extern int recv_msg_list_count(ReceiveMessageDataList *msg_list);
  extern bool recv_msg_list_empty(ReceiveMessageDataList *msg_list);
  extern void recv_msg_list_debug(ReceiveMessageDataList *msg_list);

  extern uint8_t * rmw_zenoh_pico_serialize(const message_type_support_callbacks_t *callbacks,
					    const void * ros_message, size_t *size);

  extern bool rmw_zenoh_pico_deserialize(void * payload_start,
					 size_t payload_size,
					 const message_type_support_callbacks_t *callbacks,
					 void * ros_message);

  extern rmw_ret_t rmw_zenoh_pico_publish(ZenohPicoPubData *pub_data,
					  const void * ros_message);

  extern bool rmw_zenoh_pico_deserialize_topic_msg(ReceiveMessageData *msg_data,
						   const message_type_support_callbacks_t *callbacks,
						   void * ros_message,
						   rmw_message_info_t * message_info);

  extern bool rmw_zenoh_pico_deserialize_response_msg(ReceiveMessageData *msg_data,
						      const message_type_support_callbacks_t *callbacks,
						      void * ros_message,
						      rmw_service_info_t * message_info);

  extern bool rmw_zenoh_pico_deserialize_request_msg(ReceiveMessageData *msg_data,
						     const message_type_support_callbacks_t *callbacks,
						     void * ros_request,
						     rmw_service_info_t * request_header);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
