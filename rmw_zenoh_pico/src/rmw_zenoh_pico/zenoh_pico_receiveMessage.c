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

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

z_mutex_t mutex_ReceiveMessageData;

ReceiveMessageData * zenoh_pico_generate_recv_msg_data(const z_sample_t *sample,
						       uint64_t recv_ts,
						       const uint8_t pub_gid[RMW_GID_STORAGE_SIZE],
						       int64_t seqnum,
						       int64_t source_ts)
{
  ReceiveMessageData * recv_data = NULL;
  ZenohPicoGenerateData(recv_data, ReceiveMessageData);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    recv_data,
    "failed to allocate struct for the ReceiveMessageData",
    return NULL);

  recv_data->payload_start = (void *)Z_MALLOC(sample->payload.len);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    recv_data->payload_start,
    "failed to allocate memory for the payload",
    return NULL);

  recv_data->payload_size  = sample->payload.len;
  memcpy(recv_data->payload_start, sample->payload.start, sample->payload.len);

  memcpy(recv_data->publisher_gid_, pub_gid, RMW_GID_STORAGE_SIZE);
  recv_data->recv_timestamp_	= recv_ts;
  recv_data->sequence_number_	= seqnum;
  recv_data->source_timestamp_	= source_ts;

  return recv_data;
}

bool zenoh_pico_delete_recv_msg_data(ReceiveMessageData * recv_data)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(recv_data, false);

  if(recv_data->payload_start != NULL)
    Z_FREE(recv_data->payload_start);

  ZenohPicoDestroyData(recv_data, ReceiveMessageData);

  return true;
}

#define PAYLOAD_DUMP_MAX 32
void zenoh_pico_debug_dump_msg(uint8_t *start, size_t size)
{
  printf("size = [%d]\n", (int)size);
  for(size_t count = 0; count < size  && count <= PAYLOAD_DUMP_MAX; count += 4){
    uint8_t *ptr = start + count;

    if((size -count)>= 4){
      printf("%02x %02x %02x %02x\t%c %c %c %c",
	     *(ptr +0), *(ptr +1),
	     *(ptr +2), *(ptr +3),
	     isascii(*(ptr +0)) ? *(ptr +0) : '.' ,
	     isascii(*(ptr +1)) ? *(ptr +1) : '.' ,
	     isascii(*(ptr +2)) ? *(ptr +2) : '.' ,
	     isascii(*(ptr +3)) ? *(ptr +3) : '.'
	);

    } else if((size -count) >= 3) {
      printf("%02x %02x %02x     \t%c %c %c",
	     *(ptr +0), *(ptr +1),
	     *(ptr +2),
	     isascii(*(ptr +0)) ? *(ptr +0) : '.' ,
	     isascii(*(ptr +1)) ? *(ptr +1) : '.' ,
	     isascii(*(ptr +2)) ? *(ptr +2) : '.'
	);

    } else if((size -count) >= 2) {
      printf("%02x %02x          \t%c %c",
	     *(ptr +0), *(ptr +1),
	     isascii(*(ptr +0)) ? *(ptr +0) : '.' ,
	     isascii(*(ptr +1)) ? *(ptr +1) : '.'
	);

    } else if((size -count) >= 1) {
      printf("%02x               \t%c",
	     *(ptr +0),
	     isascii(*(ptr +0)) ? *(ptr +0) : '.'
	);
    }
    printf("\n");
  }

  return;
}
void zenoh_pico_debug_recv_msg_data(ReceiveMessageData * recv_data)
{
  printf("--------- recv msg data ----------\n");
  printf("ref              = %d\n", recv_data->ref_);
  printf("recv_timestamp   = %d\n", (int)recv_data->recv_timestamp_);
  printf("publisher_gid_   = [");
  for(size_t count = 0; count < RMW_GID_STORAGE_SIZE && count < 16; count++){
    printf("%02x ", recv_data->publisher_gid_[count]);
  }
  printf("]\n");
  printf("sequence_number  = [%d]\n", (int)recv_data->sequence_number_);
  printf("source_timestamp = [%d]\n", (int)recv_data->source_timestamp_);

  printf("--------- recv simple data ----------\n");
  zenoh_pico_debug_dump_msg(recv_data->payload_start, recv_data->payload_size);
}

void recv_msg_list_init(ReceiveMessageDataList *msg_list)
{
  msg_list->que_top    = (ReceiveMessageData *)NULL;
  msg_list->que_bottom = (ReceiveMessageData *)NULL;
  msg_list->count      = 0;

  z_mutex_init(&msg_list->mutex);
}

void recv_msg_list_destroy(ReceiveMessageDataList *msg_list)
{
  if(msg_list == NULL){
    return;
  }

  z_mutex_lock(&msg_list->mutex);
  ReceiveMessageData * msg_data = msg_list->que_top;

  for(size_t count = 0; msg_data != NULL; count++){
    (void)zenoh_pico_delete_recv_msg_data(msg_data);
    msg_data = msg_data->next_;
  }
  z_mutex_unlock(&msg_list->mutex);

  z_mutex_free(&msg_list->mutex);
}

ReceiveMessageData *recv_msg_list_push(ReceiveMessageDataList *msg_list,
				       ReceiveMessageData *recv_data)
{
  if((msg_list == NULL) || (recv_data == NULL))
    return NULL;

  z_mutex_lock(&msg_list->mutex);

  ReceiveMessageData *bottom_msg = msg_list->que_bottom;

  recv_data->next_ = NULL;

  if(bottom_msg != NULL){
    bottom_msg->next_ = recv_data;
  }

  if(msg_list->que_top == NULL)
    msg_list->que_top = recv_data;

  msg_list->que_bottom = recv_data;
  msg_list->count += 1;

  z_mutex_unlock(&msg_list->mutex);

  return(recv_data);
}

ReceiveMessageData *recv_msg_list_pop(ReceiveMessageDataList *msg_list)
{

  if(msg_list == NULL)
    return NULL;

  z_mutex_lock(&msg_list->mutex);

  ReceiveMessageData *bottom_msg = msg_list->que_top;
  if(bottom_msg == NULL){
    z_mutex_unlock(&msg_list->mutex);
    return NULL;
  }

  msg_list->que_top = bottom_msg->next_;
  bottom_msg->next_  = NULL;

  msg_list->count -= 1;
  if(msg_list->count <= 0)
    msg_list->count = 0;

  z_mutex_unlock(&msg_list->mutex);

  return bottom_msg;
}

int recv_msg_list_count(ReceiveMessageDataList *msg_list)
{
  int ret;

  z_mutex_lock(&msg_list->mutex);
  ret = msg_list->count;
  z_mutex_unlock(&msg_list->mutex);

  return ret;
}

bool recv_msg_list_empty(ReceiveMessageDataList *msg_list)
{
  return recv_msg_list_count(msg_list) == 0 ? true : false;
}

void recv_msg_list_debug(ReceiveMessageDataList *msg_list)
{
  if(msg_list == NULL){
    printf("msg_list is NULL\n");
    return;
  }

  printf("data dump start... \n");

  z_mutex_lock(&msg_list->mutex);
  ReceiveMessageData * msg_data = msg_list->que_top;

  for(size_t count = 0; msg_data != NULL; count++){
    printf("[%d]\n", (int)count);
    zenoh_pico_debug_recv_msg_data(msg_data);
    msg_data = msg_data->next_;
  }
  z_mutex_unlock(&msg_list->mutex);

  printf("data dump end  ... \n");
}
