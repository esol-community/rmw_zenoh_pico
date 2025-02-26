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

z_owned_mutex_t mutex_ReceiveMessageData;

static void set_ros2_header(uint8_t *msg_bytes)
{
// This magic code is said to be used to evaluate endianness.
// How it is actually managed requires separate investigation.
  const uint8_t _header[] = {0x00, 0x01, 0x00, 0x00};

  memcpy(msg_bytes, _header, 4);
}

uint8_t * rmw_zenoh_pico_serialize(const message_type_support_callbacks_t *callbacks,
				   const void * ros_message,
				   size_t *size)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  size_t serialized_size = callbacks->get_serialized_size(ros_message);
  size_t alloc_size = serialized_size +ROS2_MSG_OFFSET;

  uint8_t * msg_bytes = (uint8_t *)TOPIC_MALLOC(alloc_size);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    msg_bytes,
    "failed to allocate memory for the serialized",
    return NULL);
  memset(msg_bytes, 0, alloc_size);

  ucdrBuffer temp_buffer;
  ucdr_init_buffer(&temp_buffer,
		   msg_bytes +ROS2_MSG_OFFSET,
		   serialized_size);

  bool ret = callbacks->cdr_serialize(ros_message, &temp_buffer);
  set_ros2_header(msg_bytes);

  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    (void)zenoh_pico_debug_dump_msg(msg_bytes, alloc_size);
  }

  *size = alloc_size;

  return msg_bytes;
}

bool rmw_zenoh_pico_deserialize(void * payload_start,
				size_t payload_size,
				const message_type_support_callbacks_t *callbacks,
				void * ros_message)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);
  ucdrBuffer temp_buffer;

  if(payload_start != NULL){
    ucdr_init_buffer(&temp_buffer,
		     payload_start + ROS2_MSG_OFFSET,
		     payload_size  - ROS2_MSG_OFFSET);

    return callbacks->cdr_deserialize(&temp_buffer, ros_message);
  }

  return false;
}

ReceiveMessageData *
rmw_zenoh_pico_generate_recv_sample_msg_data(const z_loaned_sample_t *sample,
					     time_t recv_ts)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  ReceiveMessageData * recv_data = NULL;
  recv_data = ZenohPicoDataGenerate(recv_data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    recv_data,
    "failed to allocate struct for the ReceiveMessageData",
    return NULL);

  recv_data->type = Sample;

  const z_loaned_bytes_t *payload = z_sample_payload(sample);
  recv_data->payload_start = (void *)TOPIC_MALLOC(z_bytes_len(payload));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    recv_data->payload_start,
    "failed to allocate memory for the payload",
    return NULL);

  recv_data->payload_size  = z_bytes_len(payload);
  _z_bytes_to_buf(payload, recv_data->payload_start, z_bytes_len(payload));

  recv_data->recv_timestamp = recv_ts;

  zenoh_pico_attachemt_data data;
  const z_loaned_bytes_t *attachment = z_sample_attachment(sample);
  if(_Z_IS_ERR(attachment_data_get(attachment, &recv_data->attachment))) {
    RMW_ZENOH_LOG_ERROR("unable to receive attachment data");

    TOPIC_FREE(recv_data->payload_start);

    return NULL;
  }

  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    attachment_debug(&recv_data->attachment);
  }

  return recv_data;
}

ReceiveMessageData *
rmw_zenoh_pico_generate_recv_query_msg_data(const z_loaned_query_t *query,
					    time_t recv_ts)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  ReceiveMessageData * recv_data = NULL;
  recv_data = ZenohPicoDataGenerate(recv_data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    recv_data,
    "failed to allocate struct for the ReceiveMessageData",
    return NULL);

  recv_data->type = Query;

  // duplicate payload
  const z_loaned_bytes_t *payload = z_query_payload(query);
  size_t payload_size = z_bytes_len(payload);

  recv_data->payload_start = (void *)TOPIC_MALLOC(z_bytes_len(payload));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    recv_data->payload_start,
    "failed to allocate memory for the payload",
    return NULL);

  recv_data->payload_size = payload_size;
  _z_bytes_to_buf(payload, recv_data->payload_start, z_bytes_len(payload));

  // clone query with ref counter.
  recv_data->query = _z_query_rc_clone(query);
  _z_query_t *_val = recv_data->query._val;

  // WORKAROUND:
  // when the query message by z_query_reply() on other places of callback
  // function on the original zenoh-pico implementation, their function dont send
  // message for remote zenoh.
  //
  // This problem occurs because the original structure(qle_infos.ke_out)
  // of the aliased keyexpr member in query is cleared by the _z_keyexpr_clear() function.
  //
  // Therefore, the keyexpr is alias information which is clone from original query data
  // is redefined as a member of the query information.

  z_keyexpr_clone(&recv_data->keyexpr, z_query_keyexpr(query));
  _val->_key = _z_keyexpr_alias(z_loan(recv_data->keyexpr));

  // set receive timestamp
  recv_data->recv_timestamp = recv_ts;

  // copy attachmet. (this data is used by rmw_send_response())
  zenoh_pico_attachemt_data data;
  const z_loaned_bytes_t *attachment = z_query_attachment(query);
  if(_Z_IS_ERR(attachment_data_get(attachment, &recv_data->attachment))) {
    RMW_ZENOH_LOG_ERROR("unable to receive attachment data");

    TOPIC_FREE(recv_data->payload_start);
    z_drop(z_move(recv_data->keyexpr));
    _z_query_rc_drop(&recv_data->query);

    return NULL;
  }

  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    attachment_debug(&recv_data->attachment);
  }

  return recv_data;
}

bool zenoh_pico_delete_recv_msg_data(ReceiveMessageData * recv_data)
{
  RMW_ZENOH_FUNC_ENTRY(recv_data);

  RMW_CHECK_ARGUMENT_FOR_NULL(recv_data, false);

  if(ZenohPicoDataRelease(recv_data)){

    if(recv_data->payload_start != NULL)
      TOPIC_FREE(recv_data->payload_start);

    attachment_destroy(&recv_data->attachment);

    if(recv_data->type == Query){
      z_drop(z_move(recv_data->keyexpr));
      _z_query_rc_drop(&recv_data->query);
    }

    ZenohPicoDataDestroy(recv_data);
  }

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
	     isalpha(*(ptr +0)) != 0 ? *(ptr +0) : '.' ,
	     isalpha(*(ptr +1)) != 0 ? *(ptr +1) : '.' ,
	     isalpha(*(ptr +2)) != 0 ? *(ptr +2) : '.' ,
	     isalpha(*(ptr +3)) != 0 ? *(ptr +3) : '.'
	);

    } else if((size -count) >= 3) {
      printf("%02x %02x %02x     \t%c %c %c",
	     *(ptr +0), *(ptr +1),
	     *(ptr +2),
	     isalpha(*(ptr +0)) != 0 ? *(ptr +0) : '.' ,
	     isalpha(*(ptr +1)) != 0 ? *(ptr +1) : '.' ,
	     isalpha(*(ptr +2)) != 0 ? *(ptr +2) : '.'
	);

    } else if((size -count) >= 2) {
      printf("%02x %02x          \t%c %c",
	     *(ptr +0), *(ptr +1),
	     isalpha(*(ptr +0)) != 0 ? *(ptr +0) : '.' ,
	     isalpha(*(ptr +1)) != 0 ? *(ptr +1) : '.'
	);

    } else if((size -count) >= 1) {
      printf("%02x               \t%c",
	     *(ptr +0),
	     isalpha(*(ptr +0)) != 0 ? *(ptr +0) : '.'
	);
    }
    printf("\n");
  }

  return;
}
void rmw_zenoh_pico_debug_recv_msg_data(ReceiveMessageData * recv_data)
{
  printf("--------- recv msg data ----------\n");
  printf("ref              = %d\n", recv_data->ref);
  printf("recv_timestamp   = [%ld]\n", recv_data->recv_timestamp);
  printf("query ref        = [%ld]\n", _z_simple_rc_strong_count(recv_data->query._cnt));

  // debug attachment
  attachment_debug(&recv_data->attachment);

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

ReceiveMessageData *recv_msg_list_append(ReceiveMessageDataList *msg_list,
					 ReceiveMessageData *recv_data)
{
  if((msg_list == NULL) || (recv_data == NULL))
    return NULL;

  z_loaned_mutex_t *msg_mutex = z_loan_mut(msg_list->mutex);

  z_mutex_lock(msg_mutex);

  recv_data->next = NULL;

  ReceiveMessageData *bottom_msg = msg_list->que_bottom;
  if(bottom_msg != NULL){
    bottom_msg->next = recv_data;
  }
  msg_list->que_bottom = recv_data;
  msg_list->count += 1;

  if(msg_list->que_top == NULL)
    msg_list->que_top = recv_data;

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

  ReceiveMessageData *top_msg = msg_list->que_top;
  if(top_msg == NULL){
    z_mutex_unlock(msg_mutex);
    return NULL;
  }
  msg_list->que_top = top_msg->next;
  msg_list->count -= 1;

  if(msg_list->que_bottom == top_msg)
    msg_list->que_bottom = NULL;

  top_msg->next  = NULL;

  z_mutex_unlock(msg_mutex);

  return top_msg;
}

ReceiveMessageData *recv_msg_list_pickup(
  ReceiveMessageDataList *msg_list,
  bool (*func)(ReceiveMessageData *, const void *),
  const void *data)
{
  if(msg_list == NULL)
    return NULL;

  z_loaned_mutex_t *msg_mutex = z_loan_mut(msg_list->mutex);

  z_mutex_lock(msg_mutex);

  ReceiveMessageData *pickup_msg = NULL;
  ReceiveMessageData *current_msg = msg_list->que_top;
  if(current_msg == NULL){
    z_mutex_unlock(msg_mutex);
    return NULL;
  }

  // 1st message data
  if(func(current_msg, data)){
    pickup_msg = current_msg;
    msg_list->que_top = current_msg->next;
    msg_list->count -= 1;

    if(msg_list->que_bottom == pickup_msg)
      msg_list->que_bottom = NULL;

    pickup_msg->next = NULL;

  }else{
    // until 2nd message data
    ReceiveMessageData *before_msg = current_msg;
    while(current_msg->next != NULL){
      current_msg = current_msg->next;

      if(func(current_msg, data)){
	pickup_msg = current_msg;
	before_msg->next = current_msg->next;
	msg_list->count -= 1;

	if(msg_list->que_bottom == pickup_msg)
	  msg_list->que_bottom = before_msg;

	pickup_msg->next = NULL;

	break;
      }
      before_msg = current_msg;
    }
  }

  z_mutex_unlock(msg_mutex);

  return pickup_msg;
}

int recv_msg_list_count(ReceiveMessageDataList *msg_list)
{
  int ret;

  if(msg_list == NULL)
    return 0;

  z_loaned_mutex_t *msg_mutex = z_loan_mut(msg_list->mutex);

  z_mutex_lock(msg_mutex);
  ret = msg_list->count;
  z_mutex_unlock(msg_mutex);

  return ret;
}

bool recv_msg_list_empty(ReceiveMessageDataList *msg_list)
{
  if(msg_list == NULL)
    return false;

  return recv_msg_list_count(msg_list) == 0 ? true : false;
}

void recv_msg_list_debug(ReceiveMessageDataList *msg_list)
{
  if(msg_list == NULL){
    printf("msg_list is NULL\n");
    return;
  }

  z_loaned_mutex_t *msg_mutex = z_loan_mut(msg_list->mutex);

  printf("data dump start [%d]... \n", msg_list->count);

  printf("que top    = [%p]\n", msg_list->que_top);
  printf("que bottom = [%p]\n", msg_list->que_bottom);

  z_mutex_lock(msg_mutex);
  ReceiveMessageData * msg_data = msg_list->que_top;

  for(size_t count = 0; msg_data != NULL; count++){
    printf("[%d]\n", (int)count);
    rmw_zenoh_pico_debug_recv_msg_data(msg_data);
    msg_data = msg_data->next;
  }
  z_mutex_unlock(msg_mutex);

  printf("data dump end  ... \n");
}

// ----------------------------

rmw_ret_t
rmw_zenoh_pico_publish(ZenohPicoPubData *pub_data,
		       const void * ros_message)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  size_t data_length;
  uint8_t * msg_bytes = rmw_zenoh_pico_serialize(pub_data->callbacks,
						 ros_message,
						 &data_length);
  if(msg_bytes == NULL)
    return RMW_RET_ERROR;

  z_mutex_lock(z_loan_mut(pub_data->mutex));

  // set attachment to option
  z_publisher_put_options_t options;
  z_publisher_put_options_default(&options);

  // gen attachment data
  z_owned_bytes_t attachment;
  attachment_sequence_num_inc(&pub_data->attachment);
  if(_Z_IS_ERR(attachment_gen(&pub_data->attachment, &attachment))){
    z_mutex_unlock(z_loan_mut(pub_data->mutex));
    return RMW_RET_ERROR;
  }
  options.attachment = z_move(attachment);

  // put publish data
  z_owned_bytes_t payload;
  z_bytes_copy_from_buf(&payload, msg_bytes, data_length);
  // z_bytes_from_static_buf(&payload, msg_bytes, data_length);

  z_result_t ret = z_publisher_put(z_loan(pub_data->topic),
				   z_move(payload),
				   &options);
  TOPIC_FREE(msg_bytes);
  z_drop(z_move(attachment));

  z_mutex_unlock(z_loan_mut(pub_data->mutex));

  return ret == _Z_RES_OK ?  RMW_RET_OK : RMW_RET_ERROR;
}

bool
rmw_zenoh_pico_deserialize_topic_msg(
  ReceiveMessageData *msg_data,
  const message_type_support_callbacks_t *callbacks,
  void * ros_message,
  rmw_message_info_t * message_info)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  if(!rmw_zenoh_pico_deserialize(msg_data->payload_start,
				 msg_data->payload_size,
				 callbacks,
				 ros_message))
    return false;

  if (message_info != NULL) {
    message_info->source_timestamp		= msg_data->attachment.timestamp;
    message_info->received_timestamp		= msg_data->recv_timestamp;
    message_info->publication_sequence_number	= msg_data->attachment.sequence_num;
    message_info->reception_sequence_number	= msg_data->sequence_num;

    message_info->publisher_gid.implementation_identifier = rmw_get_implementation_identifier();

    const uint8_t *gid_ptr = z_slice_data(z_loan(msg_data->attachment.gid));
    size_t gid_len = z_slice_len(z_loan(msg_data->attachment.gid));
    if(gid_len > sizeof(message_info->publisher_gid.data))
      memcpy(message_info->publisher_gid.data, gid_ptr, sizeof(message_info->publisher_gid.data));
    else
      memcpy(message_info->publisher_gid.data, gid_ptr, gid_len);

    message_info->from_intra_process = false;
  }

  return true;
}

static void rmw_zneoh_fill_request_header(rmw_service_info_t *request_header,
					  ReceiveMessageData *msg_data)
{
  request_header->source_timestamp		= msg_data->attachment.timestamp;
  request_header->received_timestamp		= msg_data->recv_timestamp;
  request_header->request_id.sequence_number	= msg_data->attachment.sequence_num;

  const uint8_t *gid_ptr = z_slice_data(z_loan(msg_data->attachment.gid));
  size_t gid_len = z_slice_len(z_loan(msg_data->attachment.gid));
  if(gid_len > sizeof(request_header->request_id.writer_guid))
    memcpy(request_header->request_id.writer_guid, gid_ptr, sizeof(request_header->request_id.writer_guid));
  else
    memcpy(request_header->request_id.writer_guid, gid_ptr, gid_len);
}

bool
rmw_zenoh_pico_deserialize_response_msg(ReceiveMessageData *msg_data,
					const message_type_support_callbacks_t *callbacks,
					void * ros_response,
					rmw_service_info_t *request_header)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  if(!rmw_zenoh_pico_deserialize(msg_data->payload_start,
				 msg_data->payload_size,
				 callbacks,
				 ros_response))
    return false;

  if(request_header != NULL)
    rmw_zneoh_fill_request_header(request_header, msg_data);

  return true;
}

bool
rmw_zenoh_pico_deserialize_request_msg(ReceiveMessageData *msg_data,
				       const message_type_support_callbacks_t *callbacks,
				       void * ros_request,
				       rmw_service_info_t * request_header)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  if(!rmw_zenoh_pico_deserialize(msg_data->payload_start,
				 msg_data->payload_size,
				 callbacks,
				 ros_request))
    return false;

  if(request_header != NULL)
    rmw_zneoh_fill_request_header(request_header, msg_data);

  return true;
}
