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

#ifndef RMW_ZENOH_PICO_RECEIVE_MESSAGE_H
#define RMW_ZENOH_PICO_RECEIVE_MESSAGE_H

#include <rmw/rmw.h>
#include <zenoh-pico.h>

typedef struct _ReceiveMessageData {
  struct _ReceiveMessageData *next_;

  int ref_;

  void *payload_start;
  size_t payload_size;

  uint64_t recv_timestamp_;
  uint8_t publisher_gid_[RMW_GID_STORAGE_SIZE];
  int64_t sequence_number_;
  int64_t source_timestamp_;

} ReceiveMessageData;

typedef struct _ReceiveMessageDataList {
  z_mutex_t mutex;

  ReceiveMessageData *que_top;
  ReceiveMessageData *que_bottom;
  int count;
} ReceiveMessageDataList;

extern ReceiveMessageData * zenoh_pico_generate_recv_msg_data(const z_sample_t *sample,
							      uint64_t recv_ts,
							      const uint8_t pub_gid[RMW_GID_STORAGE_SIZE],
							      int64_t seqnum,
							      int64_t source_ts);
extern bool zenoh_pico_delete_recv_msg_data(ReceiveMessageData * recv_data);
extern void zenoh_pico_debug_dump_msg(uint8_t *start, size_t size);
extern void zenoh_pico_debug_recv_msg_data(ReceiveMessageData * recv_data);

extern void recv_msg_list_init(ReceiveMessageDataList *msg_list);
extern ReceiveMessageData *recv_msg_list_push(ReceiveMessageDataList *msg_list,
					      ReceiveMessageData *recv_data);
extern ReceiveMessageData *recv_msg_list_pop(ReceiveMessageDataList *msg_list);
extern int recv_msg_list_count(ReceiveMessageDataList *msg_list);
extern bool recv_msg_list_empty(ReceiveMessageDataList *msg_list);
extern void recv_msg_list_debug(ReceiveMessageDataList *msg_list);

#endif
