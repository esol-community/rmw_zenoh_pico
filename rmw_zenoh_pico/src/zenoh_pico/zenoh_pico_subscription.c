#include <rmw_zenoh_pico/config.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <rmw/allocators.h>
#include <rmw/rmw.h>

#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_nodeInfo.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_logging.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_node.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_subscription.h"
#include "zenoh-pico/api/macros.h"
#include "zenoh-pico/api/types.h"
#include "zenoh-pico/protocol/core.h"

ZenohPicoSubData * zenoh_pico_generate_sub_data(size_t sub_id,
						ZenohPicoNodeData *node,
						ZenohPicoEntity *entity,
						const rosidl_message_type_support_t * type_support,
						rmw_qos_profile_t *qos_profile)
{
  if((node == NULL) || (entity == NULL))
    return NULL;

  ZenohPicoSubData *sub_data = NULL;
  ZenohPicoGenerateData(sub_data, ZenohPicoSubData);
  if(sub_data == NULL)
    return NULL;

  sub_data->node_	= node;
  sub_data->entity_	= entity;
  sub_data->id_		= sub_id;

  sub_data->adapted_qos_profile_ = *qos_profile;

  ZenohPicoNodeInfo_t  *node_info  = entity->node_info_;
  ZenohPicoTopicInfo_t *topic_info = entity->topic_info_;

  // generate key from entity data
  sub_data->token_key_ = generate_liveliness(entity);

  // generate topic key
  sub_data->topic_key_ = ros_topic_name_to_zenoh_key(Z_STRING_VAL(node_info->domain_),
						     Z_STRING_VAL(topic_info->name_),
						     Z_STRING_VAL(topic_info->type_),
						     Z_STRING_VAL(topic_info->hash_));

  return sub_data;
}

bool zenoh_pico_destroy_sub_data(ZenohPicoSubData *sub_data)
{
  _Z_DEBUG("%s : start", __func__);

  undeclaration_sub_data(sub_data);

  Z_STRING_FREE(sub_data->token_key_);
  Z_STRING_FREE(sub_data->topic_key_);

  // disable until implement of ref counter.
  // if(sub_data->node_ != NULL){
  //   zenoh_pico_destroy_node_data(sub_data->node_);
  //   sub_data->node_ = NULL;
  // }

  if(sub_data->entity_ != NULL){
    zenoh_pico_destroy_entitiy(sub_data->entity_);
    sub_data->entity_ = NULL;
  }

  ZenohPicoDestroyData(sub_data);

  return true;
}

void zenoh_pico_debug_sub_data(ZenohPicoSubData *sub_data)
{
  printf("--------- subscription data ----------\n");
  printf("ref = %d\n", sub_data->ref_);

  Z_STRING_PRINTF(sub_data->token_key_, token_key);
  Z_STRING_PRINTF(sub_data->topic_key_, topic_key);

  // debug node member
  // zenoh_pico_debug_node_data(sub_data->node_);

  // debug entity member
  zenoh_pico_debug_entitiy(sub_data->entity_);
}

// --------------------------


bool get_gid_from_attachment(
  const z_attachment_t *attachment, uint8_t gid[RMW_GID_STORAGE_SIZE])
{
  if (!z_attachment_check(attachment)) {
    return false;
  }

  z_bytes_t index = z_attachment_get(*attachment, z_bytes_from_str("source_gid"));
  if (!z_check(index)) {
    return false;
  }

  if (index.len != RMW_GID_STORAGE_SIZE) {
    return false;
  }

  memcpy(gid, index.start, index.len);

  return true;
}

int64_t get_int64_from_attachment(const z_attachment_t * const attachment, char * name)
{
  if (!z_attachment_check(attachment)) {
    // A valid request must have had an attachment
    return -1;
  }

  z_bytes_t index = z_attachment_get(*attachment, z_bytes_from_str(name));
  if (!z_check(index)) {
    return -1;
  }

  if (index.len < 1) {
    return -1;
  }

  if (index.len > 19) {
    // The number was larger than we expected
    return -1;
  }

  // The largest possible int64_t number is INT64_MAX, i.e. 9223372036854775807.
  // That is 19 characters long, plus one for the trailing \0, means we need 20 bytes.
  char int64_str[20];

  memcpy(int64_str, index.start, index.len);
  int64_str[index.len] = '\0';

  errno = 0;
  char * endptr;
  int64_t num = strtol(int64_str, &endptr, 10);
  if (num == 0) {
    // This is an error regardless; the client should never send this
    return -1;
  } else if (endptr == int64_str) {
    // No values were converted, this is an error
    return -1;
  } else if (*endptr != '\0') {
    // There was junk after the number
    return -1;
  } else if (errno != 0) {
    // Some other error occurred, which may include overflow or underflow
    return -1;
  }

  return num;
}

void sub_data_handler(const z_sample_t *sample, void *ctx) {
  _Z_DEBUG("%s : start", __func__);

  z_owned_str_t keystr = z_keyexpr_to_string(sample->keyexpr);
  _Z_INFO("%s : keystr is %s ", __func__, keystr._value);

  ZenohPicoSubData *sub_data = (ZenohPicoSubData *)ctx;
  if (sub_data == NULL) {
    _Z_ERROR(
      "%s: Unable to obtain rmw_subscription_data_t from data for subscription for %s",
      __func__,keystr._value);
    return;
  }

  uint8_t pub_gid[RMW_GID_STORAGE_SIZE];
  if (!get_gid_from_attachment(&sample->attachment, pub_gid)) {
    // We failed to get the GID from the attachment.  While this isn't fatal,
    // it is unusual and so we should report it.
    memset(pub_gid, 0, RMW_GID_STORAGE_SIZE);
    _Z_ERROR("Unable to obtain publisher GID from the attachment.");
  }

  int64_t sequence_number = get_int64_from_attachment(&sample->attachment, "sequence_number");
  if (sequence_number < 0) {
    // We failed to get the sequence number from the attachment.  While this
    // isn't fatal, it is unusual and so we should report it.
    sequence_number = 0;
    _Z_ERROR("Unable to obtain sequence number from the attachment.");
  }

  int64_t source_timestamp = get_int64_from_attachment(&sample->attachment, "source_timestamp");
  if (source_timestamp < 0) {
    // We failed to get the source timestamp from the attachment.  While this
    // isn't fatal, it is unusual and so we should report it.
    source_timestamp = 0;
    _Z_ERROR("Unable to obtain sequence number from the attachment.");
  }

  _Z_DEBUG(">> [Subscriber] Received (%s) [%s]",
	   z_loan(keystr),
	   sample->payload.start + 8);

}

// callback: the typical ``callback`` function. ``context`` will be passed as its last argument.
// dropper: allows the callback's state to be freed. ``context`` will be passed as its last argument.
// context: a pointer to an arbitrary state.

bool declaration_sub_data(ZenohPicoSubData *sub_data)
{
  printf("init sub_data = %p\n", sub_data);

  ZenohPicoSession *session = sub_data->node_->session_;

  // declare subscriber
  z_subscriber_options_t sub_options = z_subscriber_options_default();
  z_owned_closure_sample_t callback_ = z_closure(sub_data_handler, 0, (void *)sub_data);
  sub_data->subscriber_ = z_declare_subscriber(z_loan(session->session_),
					       z_keyexpr(sub_data->topic_key_.val),
					       z_move(callback_),
					       &sub_options);
  if (!z_check(sub_data->subscriber_)) {
    _Z_DEBUG("Unable to declare subscriber.");
    return false;
  }

  // liveliness tokendeclare
  const char *keyexpr = Z_STRING_VAL(sub_data->token_key_);

  _Z_DEBUG("Declaring subscriber key expression '%s'...", keyexpr);
  sub_data->token_ = z_declare_keyexpr(z_loan(session->session_), z_keyexpr(keyexpr));
  if (!z_check(sub_data->token_)) {
    _Z_DEBUG("Unable to declare talken.");
    return false;
  }

  return true;
}

bool undeclaration_sub_data(ZenohPicoSubData *sub_data)
{
  ZenohPicoSession *session = sub_data->node_->session_;

  if (z_check(sub_data->subscriber_)) {
    z_undeclare_subscriber(z_move(sub_data->subscriber_));
  }

  if (z_check(sub_data->token_)) {
    z_undeclare_keyexpr(z_loan(session->session_), &sub_data->token_);
  }

  return true;
}
