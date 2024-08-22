#ifndef RMW_ZENOH_PICO_RECEIVE_MESSAGE_H
#define RMW_ZENOH_PICO_RECEIVE_MESSAGE_H

#include <rmw/rmw.h>
#include <zenoh-pico.h>

typedef struct _ReceiveMessageData {
  struct _ReceiveMessageData *next_;

  int ref_;

  z_sample_t sample_;
  uint64_t recv_timestamp_;
  uint8_t publisher_gid_[RMW_GID_STORAGE_SIZE];
  int64_t sequence_number_;
  int64_t source_timestamp_;

} ReceiveMessageData;

typedef struct _ReceiveMessageDataList {
  ReceiveMessageData *que_top;
  ReceiveMessageData *que_bottom;
  int count;
} ReceiveMessageDataList;

extern ReceiveMessageData * zenoh_pico_generate_recv_msg_data(const z_sample_t *sample,
							      uint64_t recv_ts,
							      const uint8_t pub_gid[RMW_GID_STORAGE_SIZE],
							      int64_t seqnum,
							      int64_t source_ts);
extern void zenoh_pico_delete_recv_msg_data(ReceiveMessageData * recv_data);
extern void zenoh_pico_debug_recv_msg_data(ReceiveMessageData * recv_data);

extern void recv_msg_list_init(ReceiveMessageDataList *msg_list);
extern ReceiveMessageData *recv_msg_list_push(ReceiveMessageDataList *msg_list,
					      ReceiveMessageData *recv_data);
extern ReceiveMessageData *recv_msg_list_pop(ReceiveMessageDataList *msg_list);
extern int recv_msg_list_count(ReceiveMessageDataList *msg_list);
extern void recv_msg_list_debug(ReceiveMessageDataList *msg_list);

#endif
