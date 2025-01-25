// Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// Copyright(C) 2024 eSOL Co., Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rmw_zenoh_pico/rmw_zenoh_pico_logging.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_session.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_wait.h"
#include "zenoh-pico/api/primitives.h"
#include "zenoh-pico/api/types.h"
#include "zenoh-pico/collections/bytes.h"
#include "zenoh-pico/collections/string.h"
#include "zenoh-pico/system/platform-common.h"
#include <rmw_zenoh_pico/config.h>

#include <rosidl_typesupport_microxrcedds_c/identifier.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <rmw/rmw.h>
#include <rmw/types.h>
#include <rmw/allocators.h>
#include <rmw/error_handling.h>

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_typesupport_microxrcedds_c/message_type_support.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

z_mutex_t mutex_ZenohPicoSubData;

ZenohPicoSubData * zenoh_pico_generate_subscription_data(
  size_t sub_id,
  ZenohPicoNodeData *node,
  ZenohPicoEntity *entity,
  const rosidl_message_type_support_t * type_support,
  const message_type_support_callbacks_t *callbacks,
  rmw_qos_profile_t *qos_profile)
{
  if((node == NULL) || (entity == NULL))
    return NULL;

  ZenohPicoSubData *sub_data = NULL;
  ZenohPicoGenerateData(sub_data, ZenohPicoSubData);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    sub_data,
    "failed to allocate struct for the ZenohPicoSubData",
    return NULL);

  sub_data->id		= sub_id;
  sub_data->node	= node;
  sub_data->entity	= entity;

  sub_data->callbacks  = callbacks;
  sub_data->adapted_qos_profile = *qos_profile;

  ZenohPicoNodeInfo  *node_info  = entity->node_info;
  ZenohPicoTopicInfo *topic_info = entity->topic_info;

  // generate key from entity data
  sub_data->token_key = generate_liveliness(entity);

  // generate topic key
  sub_data->topic_key = ros_topic_name_to_zenoh_key(Z_STRING_VAL(node_info->domain),
						     Z_STRING_VAL(topic_info->name),
						     Z_STRING_VAL(topic_info->type),
						     Z_STRING_VAL(topic_info->hash));
  // init receive message list
  recv_msg_list_init(&sub_data->message_queue);

  // init data callback manager
  data_callback_init(&sub_data->data_callback_mgr);

  // init rmw_wait condition
  z_mutex_init(&sub_data->condition_mutex);
  sub_data->wait_set_data = NULL;

  return sub_data;
}

bool zenoh_pico_destroy_subscription_data(ZenohPicoSubData *sub_data)
{
  RMW_ZENOH_FUNC_ENTRY();

  (void)undeclaration_subscription_data(sub_data);

  Z_STRING_FREE(sub_data->token_key);
  Z_STRING_FREE(sub_data->topic_key);

  if(sub_data->node != NULL){
    zenoh_pico_destroy_node_data(sub_data->node);
    sub_data->node = NULL;
  }

  if(sub_data->entity != NULL){
    zenoh_pico_destroy_entity(sub_data->entity);
    sub_data->entity = NULL;
  }

  // free receive message list
  recv_msg_list_debug(&sub_data->message_queue);

  if(recv_msg_list_count(&sub_data->message_queue) > 0){
    while(true){
      ReceiveMessageData * recv_data = recv_msg_list_pop(&sub_data->message_queue);

      if(recv_data == NULL)
	break;

      zenoh_pico_delete_recv_msg_data(recv_data);
    }
  }

  z_mutex_free(&sub_data->condition_mutex);
  sub_data->wait_set_data = NULL;

  ZenohPicoDestroyData(sub_data, ZenohPicoSubData);

  return true;
}

void zenoh_pico_debug_subscription_data(ZenohPicoSubData *sub_data)
{
  printf("--------- subscription data ----------\n");
  printf("ref = %d\n", sub_data->ref);

  Z_STRING_PRINTF(sub_data->token_key, token_key);
  Z_STRING_PRINTF(sub_data->topic_key, topic_key);

  // debug node member
  zenoh_pico_debug_node_data(sub_data->node);

  // debug entity member
  zenoh_pico_debug_entity(sub_data->entity);
}

#if Z_FEATURE_ATTACHMENT == 1

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
#endif

void add_new_message(ZenohPicoSubData *sub_data, ReceiveMessageData *recv_data)
{
  // zenoh_pico_debug_recv_msg_data(recv_data);

  (void)recv_msg_list_push(&sub_data->message_queue, recv_data);
  RMW_ZENOH_LOG_INFO("message_queue size is %d",
		     recv_msg_list_count(&sub_data->message_queue));

  (void)data_callback_trigger(&sub_data->data_callback_mgr);

  (void)subscription_condition_trigger(sub_data);

  return;
}

static void _sub_data_handler(const z_sample_t *sample, void *ctx) {
  RMW_ZENOH_FUNC_ENTRY();

  ZenohPicoSubData *sub_data = (ZenohPicoSubData *)ctx;
  if (sub_data == NULL) {
    z_owned_str_t keystr = z_keyexpr_to_string(sample->keyexpr);

    RMW_ZENOH_LOG_INFO("%s : keystr is %s ", __func__, keystr._value);
    RMW_ZENOH_LOG_ERROR("Unable to obtain rmw_subscription_data_t from data for "
			"subscription for %s",
			z_loan(keystr));
    z_drop(z_move(keystr));

    return;
  }

  uint8_t pub_gid[RMW_GID_STORAGE_SIZE];
  int64_t sequence_number  = 0;
  int64_t source_timestamp = 0;

#if Z_FEATURE_ATTACHMENT == 1
  if (!get_gid_from_attachment(&sample->attachment, pub_gid)) {
    // by rmw_zenoh_c
    // We failed to get the GID from the attachment.  While this isn't fatal,
    // it is unusual and so we should report it.
    memset(pub_gid, 0, RMW_GID_STORAGE_SIZE);
    RMW_ZENOH_LOG_INFO("Unable to obtain publisher GID from the attachment.");
  }

  sequence_number = get_int64_from_attachment(&sample->attachment, "sequence_number");
  if (sequence_number < 0) {
    // by rmw_zenoh_c
    // We failed to get the sequence number from the attachment.  While this
    // isn't fatal, it is unusual and so we should report it.
    sequence_number = 0;
    RMW_ZENOH_LOG_INFO("Unable to obtain sequence number from the attachment.");
  }

  source_timestamp = get_int64_from_attachment(&sample->attachment, "source_timestamp");
  if (source_timestamp < 0) {
    // by rmw_zenoh_c
    // We failed to get the source timestamp from the attachment.  While this
    // isn't fatal, it is unusual and so we should report it.
    source_timestamp = 0;
    RMW_ZENOH_LOG_INFO("Unable to obtain source timestamp from the attachment.");
  }
#endif

  ReceiveMessageData * recv_data = zenoh_pico_generate_recv_msg_data(sample,
								     sample->timestamp.time,
								     pub_gid,
								     sequence_number,
								     source_timestamp);
  if(recv_data == NULL) return;

  (void)add_new_message(sub_data, recv_data);
}

static void _token_handler(const z_sample_t *sample, void *ctx) {
  RMW_ZENOH_FUNC_ENTRY();

  ZenohPicoSubData *sub_data = (ZenohPicoSubData *)ctx;

  return;
}

// callback: the typical ``callback`` function. ``context`` will be passed as its last argument.
// dropper: allows the callback's state to be freed. ``context`` will be passed as its last argument.
// context: a pointer to an arbitrary state.

bool declaration_subscription_data(ZenohPicoSubData *sub_data)
{
  RMW_ZENOH_FUNC_ENTRY();

  ZenohPicoSession *session = sub_data->node->session;

  // declare subscriber
  z_subscriber_options_t options = z_subscriber_options_default();
  z_owned_closure_sample_t sub_callback_ = z_closure(_sub_data_handler, 0, (void *)sub_data);
  sub_data->subscriber = z_declare_subscriber(z_loan(session->session),
					       z_keyexpr(sub_data->topic_key.val),
					       z_move(sub_callback_),
					       &options);
  if (!z_check(sub_data->subscriber)) {
    RMW_ZENOH_LOG_DEBUG("Unable to declare subscriber.");
    return false;
  }

  // liveliness tokendeclare
  const char *keyexpr = Z_STRING_VAL(sub_data->token_key);
  RMW_ZENOH_LOG_DEBUG("Declaring subscriber key expression '%s'...", keyexpr);

  z_owned_closure_sample_t token_callback_ = z_closure(_token_handler, 0, (void *)sub_data);
  sub_data->token = z_declare_subscriber(z_loan(session->session),
					 z_keyexpr(keyexpr),
					 z_move(token_callback_),
					 NULL);
  if (!z_check(sub_data->token)) {
    RMW_ZENOH_LOG_DEBUG("Unable to declare token.");
    return false;
  }

  return true;
}

bool undeclaration_subscription_data(ZenohPicoSubData *sub_data)
{
  ZenohPicoSession *session = sub_data->node->session;

  if (z_check(sub_data->token)) {
    z_undeclare_subscriber(z_move(sub_data->token));
  }


  if (z_check(sub_data->subscriber)) {
    z_undeclare_subscriber(z_move(sub_data->subscriber));
  }

  return true;
}

void subscription_condition_trigger(ZenohPicoSubData *sub_data)
{
  z_mutex_lock(&sub_data->condition_mutex);

  if(sub_data->wait_set_data != NULL) {
    ZenohPicoWaitSetData * wait_set_data = sub_data->wait_set_data;

    wait_condition_lock(wait_set_data);

    wait_condition_triggered(wait_set_data, true);
    wait_condition_signal(wait_set_data);

    wait_condition_unlock(wait_set_data);
  }

  z_mutex_unlock(&sub_data->condition_mutex);
}

bool subscription_condition_check_and_attach(ZenohPicoSubData *sub_data,
					     ZenohPicoWaitSetData * wait_set_data)
{
  bool ret;

  z_mutex_lock(&sub_data->condition_mutex);

  if(!recv_msg_list_empty(&sub_data->message_queue)){
    RMW_ZENOH_LOG_INFO("queue_has_data_and_attach_condition_if_notmessage_queue size is %d",
		       recv_msg_list_count(&sub_data->message_queue));
    z_mutex_unlock(&sub_data->condition_mutex);
    return true;
  }

  sub_data->wait_set_data = wait_set_data;

  z_mutex_unlock(&sub_data->condition_mutex);

  return false;
}

bool subscription_condition_detach_and_queue_is_empty(ZenohPicoSubData *sub_data)
{
  bool ret;

  z_mutex_lock(&sub_data->condition_mutex);

  sub_data->wait_set_data = NULL;
  ret = recv_msg_list_empty(&sub_data->message_queue);

  z_mutex_unlock(&sub_data->condition_mutex);

  return ret;
}

static rmw_subscription_t * _rmw_subscription_generate(rmw_context_t *context,
						       ZenohPicoSubData *sub_data,
						       const rmw_subscription_options_t *options)
{
  RMW_ZENOH_FUNC_ENTRY();
  (void)context;

  rmw_subscription_t * rmw_subscription = Z_MALLOC(sizeof(rmw_subscription_t));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_subscription,
    "failed to allocate memory for the subscription",
    return NULL);

  rmw_subscription->implementation_identifier	= rmw_get_implementation_identifier();
  rmw_subscription->topic_name			= Z_STRING_VAL(sub_data->entity->topic_info->name);
  rmw_subscription->can_loan_messages		= false;
  rmw_subscription->is_cft_enabled		= false;
  rmw_subscription->data                        = (void *)sub_data;

  memcpy(&rmw_subscription->options, options, sizeof(rmw_subscription_options_t));

  return rmw_subscription;
}

static rmw_ret_t _rmw_subscription_destroy(rmw_subscription_t * sub)
{
  RMW_ZENOH_FUNC_ENTRY();

  RMW_CHECK_ARGUMENT_FOR_NULL(sub, RMW_RET_INVALID_ARGUMENT);

  ZenohPicoSubData *sub_data = (ZenohPicoSubData *)sub->data;

  if(sub_data != NULL){
    undeclaration_subscription_data(sub_data);

    zenoh_pico_destroy_subscription_data(sub_data);
  }

  Z_FREE(sub);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_subscription_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_subscription_allocation_t * allocation)
{
  RMW_ZENOH_FUNC_ENTRY();

  (void)type_support;
  (void)message_bounds;
  (void)allocation;
  RMW_SET_ERROR_MSG("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_fini_subscription_allocation(
  rmw_subscription_allocation_t * allocation)
{
  RMW_ZENOH_FUNC_ENTRY();
  (void)allocation;
  RMW_SET_ERROR_MSG("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_subscription_t *
rmw_create_subscription(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_subscription_options_t * subscription_options)
{
  RMW_ZENOH_FUNC_ENTRY();
  RMW_ZENOH_LOG_INFO("%s : topic_name = %s", __func__, topic_name);

  RMW_CHECK_ARGUMENT_FOR_NULL(node, NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, NULL);
  if (topic_name[0] == '\0') {
    RMW_SET_ERROR_MSG("topic_name argument is an empty string");
    return NULL;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_options, NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->data, NULL);

  ZenohPicoNodeData *node_data = (ZenohPicoNodeData *)node->data;
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node_data, "unable to create subscription as node_data is invalid.",
    return NULL);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context,
    "expected initialized context",
    return NULL);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context->impl,
    "expected initialized context impl",
    return NULL);

  // scan hash data by type_support
  const rosidl_message_type_support_t * type_support = find_message_type_support(type_supports);
  if (type_support == NULL) {
    // error was already set by find_message_type_support
    RMW_ZENOH_LOG_INFO("type_support is null");
    return NULL;
  }
  RMW_ZENOH_LOG_INFO("typesupport_identifier = [%s]", type_support->typesupport_identifier);

  // get hash data
  const rosidl_type_hash_t * type_hash = type_support->get_type_hash_func(type_support);

  // convert hash
  _z_string_t _hash_data = convert_hash(type_hash);
  RMW_ZENOH_LOG_INFO("%s : hash = [%s][%s]", __func__, topic_name, _hash_data.val);

  // generate message type
  const message_type_support_callbacks_t *callbacks
    = (const message_type_support_callbacks_t *)(type_support->data);

  z_string_t _type_name = convert_message_type(callbacks);
  RMW_ZENOH_LOG_INFO("%s : type name = [%s]", __func__, _type_name.val);

  // generate Qos
  // rmw_qos_profile_t _qos_profile = *qos_profile;
  rmw_qos_profile_t _qos_profile;
  memset(&_qos_profile, 0, sizeof(_qos_profile));
  test_qos_profile(&_qos_profile);

  z_string_t qos_key = qos_to_keyexpr(&_qos_profile);
  RMW_ZENOH_LOG_INFO("%s : qos = [%s]", __func__, qos_key.val);

  _z_string_t _topic_name = _z_string_make(topic_name);
  ZenohPicoTopicInfo *_topic_info = zenoh_pico_generate_topic_info(&_topic_name,
								   &_type_name,
								   &_hash_data,
								   &qos_key);
  // clone node_info
  ZenohPicoNodeInfo *_node_info = zenoh_pico_clone_node_info(node_data->entity->node_info);

  // generate entity data
  size_t _entity_id = zenoh_pico_get_next_entity_id();
  ZenohPicoSession *_session = node_data->session;
  ZenohPicoEntity *_entity = zenoh_pico_generate_entity( z_info_zid(z_loan(_session->session)),
							 _entity_id,
							 node_data->id,
							 Subscription,
							 _node_info,
							 _topic_info);
  ZenohPicoNodeData *_node = zenoh_pico_loan_node_data(node_data);
  ZenohPicoSubData *_sub_data = zenoh_pico_generate_subscription_data(_entity_id,
								      _node,
								      _entity,
								      type_support,
								      callbacks,
								      &_qos_profile);
  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    zenoh_pico_debug_subscription_data(_sub_data);
  }

  rmw_subscription_t * rmw_subscription = _rmw_subscription_generate(node->context,
								     _sub_data,
								     subscription_options);
  declaration_subscription_data(_sub_data);

  return rmw_subscription;
}

rmw_ret_t
rmw_destroy_subscription(
  rmw_node_t * node,
  rmw_subscription_t * subscription)
{
  RMW_ZENOH_FUNC_ENTRY();

  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  return _rmw_subscription_destroy(subscription);
}

rmw_ret_t
rmw_subscription_count_matched_publishers(
  const rmw_subscription_t * subscription,
  size_t * publisher_count)
{
  RMW_ZENOH_FUNC_ENTRY();
  (void)subscription;
  (void)publisher_count;
  RMW_ZENOH_LOG_INFO(
    "Function not available; enable RMW_UXRCE_GRAPH configuration profile before using");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_subscription_get_actual_qos(
  const rmw_subscription_t * subscription,
  rmw_qos_profile_t * qos)
{
  RMW_ZENOH_FUNC_ENTRY();
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_subscription_set_content_filter(
  rmw_subscription_t * subscription,
  const rmw_subscription_content_filter_options_t * options)
{
  RMW_ZENOH_FUNC_ENTRY();
  (void) subscription;
  (void) options;

  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_subscription_get_content_filter(
  const rmw_subscription_t * subscription,
  rcutils_allocator_t * allocator,
  rmw_subscription_content_filter_options_t * options)
{
  RMW_ZENOH_FUNC_ENTRY();
  (void) subscription;
  (void) allocator;
  (void) options;

  return RMW_RET_UNSUPPORTED;
}
