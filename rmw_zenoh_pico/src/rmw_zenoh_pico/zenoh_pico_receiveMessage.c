
#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_receiveMessage.h"
#include <stdbool.h>

static void _z_recv_sample_clone(z_sample_t *dst, const z_sample_t *src)
{
  dst->keyexpr._id = src->keyexpr._id;
  z_string_t keyexpr_data = z_string_make(src->keyexpr._suffix);
  dst->keyexpr._suffix = keyexpr_data.val;

  _z_bytes_copy(&dst->payload, &src->payload);

  dst->encoding.prefix = src->encoding.prefix;
  _z_bytes_copy(&dst->encoding.suffix, &src->encoding.suffix);

  dst->timestamp.time = src->timestamp.time;
  dst->timestamp.id = src->timestamp.id;
}

static void _z_recv_sample_clear(z_sample_t *sample)
{
  z_string_t keyexpr;
  keyexpr.val = sample->keyexpr._suffix;
  keyexpr.len = strlen(sample->keyexpr._suffix);

  _z_string_clear(&keyexpr);
}

ReceiveMessageData * zenoh_pico_generate_recv_msg_data(const z_sample_t *sample,
						       uint64_t recv_ts,
						       const uint8_t pub_gid[RMW_GID_STORAGE_SIZE],
						       int64_t seqnum,
						       int64_t source_ts)
{
  ReceiveMessageData * recv_data = NULL;
  ZenohPicoGenerateData(recv_data, ReceiveMessageData);
  if(recv_data == NULL)
    return NULL;

  _z_recv_sample_clone(&recv_data->sample_, sample);

  memcpy(recv_data->publisher_gid_, pub_gid, RMW_GID_STORAGE_SIZE);
  recv_data->recv_timestamp_	= recv_ts;
  recv_data->sequence_number_	= seqnum;
  recv_data->source_timestamp_	= source_ts;

  return recv_data;
}

void zenoh_pico_delete_recv_msg_data(ReceiveMessageData * recv_data)
{
  _z_recv_sample_clear(&recv_data->sample_);

  ZenohPicoDestroyData(recv_data);
}

void zenoh_pico_debug_recv_msg_data(ReceiveMessageData * recv_data)
{
  printf("--------- recv msg data ----------\n");
  printf("ref              = %d\n", recv_data->ref_);
  printf("recv_timestamp   = %ld\n", recv_data->recv_timestamp_);
  printf("publisher_gid_   = [");
  for(size_t count = 0; count < RMW_GID_STORAGE_SIZE && count < 16; count++){
    printf("%02x ", recv_data->publisher_gid_[count]);
  }
  printf("]\n");
  printf("sequence_number  = [%ld]\n", recv_data->sequence_number_);
  printf("source_timestamp = [%ld]\n", recv_data->source_timestamp_);

  {
    printf("--------- recv simple data ----------\n");
    z_sample_t *sample = &recv_data->sample_;

    z_owned_str_t keystr = z_keyexpr_to_string(sample->keyexpr);
    printf("keystr = [%s]\n", keystr._value);
    z_drop(z_move(keystr));

    printf("payload[%ld]   = [%s]\n", sample->payload.len, sample->payload.start +8);
    printf("timestamp      = [%ld]\n", sample->timestamp.time);
    printf("encoding[%d]   = [%p][%ld]\n",
	   sample->encoding.prefix,
	   sample->encoding.suffix.start,
	   sample->encoding.suffix.len);
    printf("kind           = [%d]\n", sample->kind);
  }
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
    zenoh_pico_delete_recv_msg_data(msg_data);
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
    printf("[%ld]\n", count);
    zenoh_pico_debug_recv_msg_data(msg_data);
    msg_data = msg_data->next_;
  }
  z_mutex_unlock(&msg_list->mutex);

  printf("data dump end  ... \n");
}
