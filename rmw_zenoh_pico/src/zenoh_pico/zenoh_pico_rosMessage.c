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

#include "zenoh-pico/api/macros.h"
#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

z_owned_mutex_t mutex_ReceiveMessageData;

void set_ros2_header(uint8_t *msg_bytes)
{
// This magic code is said to be used to evaluate endianness.
// How it is actually managed requires separate investigation.
  const uint8_t _header[] = {0x00, 0x01, 0x00, 0x00};

  memcpy(msg_bytes, _header, 4);
}

static bool
rmw_zenoh_pico_deserialize(ReceiveMessageData *msg_data,
			   const message_type_support_callbacks_t *callbacks,
			   void * ros_message)
{
  ucdrBuffer temp_buffer;

  ucdr_init_buffer(&temp_buffer,
		   msg_data->payload_start +SUB_MSG_OFFSET,
		   msg_data->payload_size -SUB_MSG_OFFSET);

  bool ret = callbacks->cdr_deserialize(
    &temp_buffer,
    ros_message);

  return ret;
}

rmw_ret_t zenoh_pico_publish(ZenohPicoPubData *pub_data,
			     const void * ros_message)
{
  z_mutex_lock(z_loan_mut(pub_data->mutex));

  size_t serialized_size = pub_data->callbacks->get_serialized_size(ros_message);

  uint8_t * msg_bytes = (uint8_t *)TOPIC_MALLOC(serialized_size +SUB_MSG_OFFSET);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    msg_bytes,
    "failed to allocate memory for the serialized",
    return RMW_RET_ERROR);
  memset(msg_bytes, 0, serialized_size +SUB_MSG_OFFSET);

  ucdrBuffer temp_buffer;
  ucdr_init_buffer(&temp_buffer,
		   msg_bytes +SUB_MSG_OFFSET,
		   serialized_size);

  bool ret = pub_data->callbacks->cdr_serialize(ros_message, &temp_buffer);

  set_ros2_header(msg_bytes);

  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    (void)zenoh_pico_debug_dump_msg(msg_bytes, serialized_size +SUB_MSG_OFFSET);
  }

  // set attachment to option
  z_publisher_put_options_t options;
  z_publisher_put_options_default(&options);

  // gen attachment data
  z_owned_bytes_t attachment;
  zenoh_pico_inc_sequence_num(&pub_data->attachment);
  if(_Z_IS_OK(zenoh_pico_attachment_gen(&pub_data->attachment, &attachment))){
    options.attachment = z_move(attachment);
  }

  // put publish data
  z_owned_bytes_t payload;
  z_bytes_copy_from_buf(&payload, msg_bytes, serialized_size + SUB_MSG_OFFSET);
  z_publisher_put(z_loan(pub_data->publisher),
		  z_move(payload),
		  &options);

  TOPIC_FREE(msg_bytes);
  z_drop(z_move(attachment));

  z_mutex_unlock(z_loan_mut(pub_data->mutex));

  return RMW_RET_OK;
}

rmw_ret_t
zenoh_pico_take(ZenohPicoSubData * sub_data,
		void * ros_message,
		rmw_message_info_t * message_info,
		bool * taken)
{
  *taken = false;

  ReceiveMessageData *msg_data = recv_msg_list_pop(&sub_data->message_queue);

  const message_type_support_callbacks_t *callbacks = sub_data->callbacks;

  bool deserialize_rv = rmw_zenoh_pico_deserialize(msg_data, callbacks, ros_message);

  if (message_info != NULL) {
    message_info->source_timestamp		= msg_data->attachment.timestamp;
    message_info->received_timestamp		= msg_data->recv_timestamp;
    message_info->publication_sequence_number	= msg_data->attachment.sequence_num;
    message_info->reception_sequence_number	= 0;

    message_info->publisher_gid.implementation_identifier = rmw_get_implementation_identifier();

    const uint8_t *gid_ptr = z_slice_data(z_loan(msg_data->attachment.gid));
    size_t gid_len = z_slice_len(z_loan(msg_data->attachment.gid));
    memcpy(message_info->publisher_gid.data, gid_ptr, gid_len);

    message_info->from_intra_process = false;
  }

  if (taken != NULL) {
    *taken = deserialize_rv;
  }

  if (!deserialize_rv) {
    RMW_SET_ERROR_MSG("Typesupport deserialize error.");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

ReceiveMessageData * zenoh_pico_generate_recv_msg_data(const z_loaned_sample_t *sample,
						       time_t recv_ts)
{
  ReceiveMessageData * recv_data = NULL;
  ZenohPicoGenerateData(recv_data, ReceiveMessageData);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    recv_data,
    "failed to allocate struct for the ReceiveMessageData",
    return NULL);

  const z_loaned_bytes_t *payload = z_sample_payload(sample);
  recv_data->payload_start = (void *)TOPIC_MALLOC(z_bytes_len(payload));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    recv_data->payload_start,
    "failed to allocate memory for the payload",
    return NULL);

  recv_data->payload_size  = z_bytes_len(payload);

  _z_bytes_to_buf(payload, recv_data->payload_start, z_bytes_len(payload));

  recv_data->recv_timestamp	= recv_ts;

  zenoh_pico_attachemt_data data;
  if(_Z_IS_ERR(zenoh_pico_attachment_data_get(sample, &recv_data->attachment))) {
    RMW_ZENOH_LOG_ERROR("unable to receive attachment data");
  }

  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    zenoh_pico_debug_attachment(&recv_data->attachment);
  }

  return recv_data;
}

bool zenoh_pico_delete_recv_msg_data(ReceiveMessageData * recv_data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  RMW_CHECK_ARGUMENT_FOR_NULL(recv_data, false);

  if(recv_data->payload_start != NULL)
    TOPIC_FREE(recv_data->payload_start);

  zenoh_pico_destroy_attachment(&recv_data->attachment);

  ZenohPicoDestroyData(recv_data, ReceiveMessageData);

  return true;
}

#define PAYLOAD_DUMP_MAX 32
void zenoh_pico_debug_dump_msg(const uint8_t *start, size_t size)
{
  printf("size = [%d]\n", (int)size);
  for(size_t count = 0; count < size  && count <= PAYLOAD_DUMP_MAX; count += 4){
    const uint8_t *ptr = start + count;

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
  printf("ref              = %d\n", recv_data->ref);
  printf("recv_timestamp   = [%d]\n", (int)recv_data->recv_timestamp);

  // debug attachment
  zenoh_pico_debug_attachment(&recv_data->attachment);

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

  z_loaned_mutex_t *msg_mutex = z_loan_mut(msg_list->mutex);

  z_mutex_lock(msg_mutex);
  ReceiveMessageData * msg_data = msg_list->que_top;

  for(size_t count = 0; msg_data != NULL; count++){
    (void)zenoh_pico_delete_recv_msg_data(msg_data);
    msg_data = msg_data->next;
  }
  z_mutex_unlock(msg_mutex);

  z_mutex_drop(z_move(msg_list->mutex));
}

ReceiveMessageData *recv_msg_list_push(ReceiveMessageDataList *msg_list,
				       ReceiveMessageData *recv_data)
{
  if((msg_list == NULL) || (recv_data == NULL))
    return NULL;

  z_loaned_mutex_t *msg_mutex = z_loan_mut(msg_list->mutex);

  z_mutex_lock(msg_mutex);

  ReceiveMessageData *bottom_msg = msg_list->que_bottom;

  recv_data->next = NULL;

  if(bottom_msg != NULL){
    bottom_msg->next = recv_data;
  }

  if(msg_list->que_top == NULL)
    msg_list->que_top = recv_data;

  msg_list->que_bottom = recv_data;
  msg_list->count += 1;

  z_mutex_unlock(msg_mutex);

  // RMW_ZENOH_LOG_INFO("message_queue size is %d", recv_msg_list_count(msg_list));

  return(recv_data);
}

ReceiveMessageData *recv_msg_list_pop(ReceiveMessageDataList *msg_list)
{

  if(msg_list == NULL)
    return NULL;

  z_loaned_mutex_t *msg_mutex = z_loan_mut(msg_list->mutex);

  z_mutex_lock(msg_mutex);

  ReceiveMessageData *bottom_msg = msg_list->que_top;
  if(bottom_msg == NULL){
    z_mutex_unlock(msg_mutex);
    return NULL;
  }

  msg_list->que_top = bottom_msg->next;
  bottom_msg->next  = NULL;

  msg_list->count -= 1;
  if(msg_list->count <= 0)
    msg_list->count = 0;

  z_mutex_unlock(msg_mutex);

  return bottom_msg;
}

int recv_msg_list_count(ReceiveMessageDataList *msg_list)
{
  int ret;

  z_loaned_mutex_t *msg_mutex = z_loan_mut(msg_list->mutex);

  z_mutex_lock(msg_mutex);
  ret = msg_list->count;
  z_mutex_unlock(msg_mutex);

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

  z_loaned_mutex_t *msg_mutex = z_loan_mut(msg_list->mutex);

  z_mutex_lock(msg_mutex);
  ReceiveMessageData * msg_data = msg_list->que_top;

  for(size_t count = 0; msg_data != NULL; count++){
    printf("[%d]\n", (int)count);
    zenoh_pico_debug_recv_msg_data(msg_data);
    msg_data = msg_data->next;
  }
  z_mutex_unlock(msg_mutex);

  printf("data dump end  ... \n");
}
