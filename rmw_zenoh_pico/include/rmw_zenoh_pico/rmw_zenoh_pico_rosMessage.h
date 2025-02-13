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

#include <rmw/rmw.h>
#include <zenoh-pico.h>

#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_attach.h>

typedef struct _ZenohPicoPubData ZenohPicoPubData;
typedef struct _ZenohPicoSubData ZenohPicoSubData;

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ReceiveMessageData {
    struct _ReceiveMessageData *next;

    int ref;

    void *payload_start;
    size_t payload_size;

    int64_t recv_timestamp;

    // attachment data [sequence_num, last timestamp, topic gid]
    zenoh_pico_attachemt_data attachment;

  } ReceiveMessageData;

  typedef struct _ReceiveMessageDataList {
    z_owned_mutex_t mutex;

    ReceiveMessageData *que_top;
    ReceiveMessageData *que_bottom;
    int count;
  } ReceiveMessageDataList;

  extern ReceiveMessageData * zenoh_pico_generate_recv_msg_data(
    const z_loaned_sample_t *sample,
    time_t recv_ts);
  extern bool zenoh_pico_delete_recv_msg_data(ReceiveMessageData * recv_data);
  extern void zenoh_pico_debug_dump_msg(const uint8_t *start, size_t size);
  extern void zenoh_pico_debug_recv_msg_data(ReceiveMessageData * recv_data);

  extern void recv_msg_list_init(ReceiveMessageDataList *msg_list);
  extern ReceiveMessageData *recv_msg_list_push(ReceiveMessageDataList *msg_list,
						ReceiveMessageData *recv_data);
  extern ReceiveMessageData *recv_msg_list_pop(ReceiveMessageDataList *msg_list);
  extern int recv_msg_list_count(ReceiveMessageDataList *msg_list);
  extern bool recv_msg_list_empty(ReceiveMessageDataList *msg_list);
  extern void recv_msg_list_debug(ReceiveMessageDataList *msg_list);

  extern void set_ros2_header(uint8_t *msg_bytes);
  extern rmw_ret_t zenoh_pico_publish(ZenohPicoPubData *pub_data,
				      const void * ros_message);

  extern rmw_ret_t zenoh_pico_take(ZenohPicoSubData * sub_data,
				   void * ros_message,
				   rmw_message_info_t * message_info,
				   bool * taken);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
