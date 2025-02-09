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
#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_session.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_wait.h"
#include "zenoh-pico/api/macros.h"
#include "zenoh-pico/api/primitives.h"
#include "zenoh-pico/api/types.h"
#include "zenoh-pico/system/common/platform.h"

#include <rmw_zenoh_pico/config.h>

#include <rosidl_typesupport_microxrcedds_c/identifier.h>

#include <stdio.h>
#include <string.h>

#include <rmw/rmw.h>
#include <rmw/types.h>
#include <rmw/allocators.h>
#include <rmw/error_handling.h>

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_typesupport_microxrcedds_c/message_type_support.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

z_owned_mutex_t mutex_ZenohPicoSubData;

ZenohPicoSubData * zenoh_pico_generate_subscription_data(
  size_t sub_id,
  ZenohPicoNodeData *node,
  ZenohPicoEntity *entity,
  const rmw_qos_profile_t *qos_profile,
  const rosidl_message_type_support_t * type_support,
  const message_type_support_callbacks_t *callbacks)
{
  RMW_ZENOH_FUNC_ENTRY(node);

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
  if(_Z_IS_ERR(generate_liveliness(entity, &sub_data->token_key))){
    RMW_SET_ERROR_MSG("failed generate_liveliness()");
    return NULL;
  }

  // generate topic key
  if(_Z_IS_ERR(ros_topic_name_to_zenoh_key(z_loan(node_info->domain),
					   z_loan(topic_info->name),
					   z_loan(topic_info->type),
					   z_loan(topic_info->hash),
					   &sub_data->topic_key))){
    RMW_SET_ERROR_MSG("failed ros_topic_name_to_zenoh_key()");
    return NULL;
  }

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
  RMW_ZENOH_FUNC_ENTRY(NULL);

  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, false);

  (void)undeclaration_subscription_data(sub_data);

  z_drop(z_move(sub_data->token_key));
  z_drop(z_move(sub_data->topic_key));

  z_drop(z_move(sub_data->subscriber));
  z_drop(z_move(sub_data->token));

  if(sub_data->node != NULL){
    (void)zenoh_pico_destroy_node_data(sub_data->node);
    sub_data->node = NULL;
  }

  if(sub_data->entity != NULL){
    (void)zenoh_pico_destroy_entity(sub_data->entity);
    sub_data->entity = NULL;
  }

  // free receive message list
  // recv_msg_list_debug(&sub_data->message_queue);

  if(recv_msg_list_count(&sub_data->message_queue) > 0){
    while(true){
      ReceiveMessageData * recv_data = recv_msg_list_pop(&sub_data->message_queue);

      if(recv_data == NULL)
	break;

      (void)zenoh_pico_delete_recv_msg_data(recv_data);
    }
  }

  z_mutex_drop(z_move(sub_data->condition_mutex));
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

  printf("message_queue = %d\n", recv_msg_list_count(&sub_data->message_queue));

  // debug node member
  zenoh_pico_debug_node_data(sub_data->node);

  // debug entity member
  zenoh_pico_debug_entity(sub_data->entity);
}

void add_new_message(ZenohPicoSubData *sub_data, ReceiveMessageData *recv_data)
{
  // zenoh_pico_debug_recv_msg_data(recv_data);

  (void)recv_msg_list_push(&sub_data->message_queue, recv_data);

  (void)data_callback_trigger(&sub_data->data_callback_mgr);

  (void)subscription_condition_trigger(sub_data);

  return;
}

static void _sub_data_handler(z_loaned_sample_t *sample, void *ctx) {
  RMW_ZENOH_FUNC_ENTRY(sample);

  ZenohPicoSubData *sub_data = (ZenohPicoSubData *)ctx;
  if (sub_data == NULL) {
    z_view_string_t keystr;
    z_keyexpr_as_view_string(z_sample_keyexpr(sample), &keystr);

    RMW_ZENOH_LOG_INFO("keystr is %s ", z_string_data(z_loan(keystr)));
    RMW_ZENOH_LOG_ERROR("Unable to obtain rmw_subscription_data_t from data for "
			"subscription for %s",
			z_string_data(z_loan(keystr)));
    return;
  }

  ReceiveMessageData * recv_data;
  if((recv_data = zenoh_pico_generate_recv_msg_data(sample, zenoh_pico_gen_timestamp())) == NULL) {
    RMW_ZENOH_LOG_ERROR("unable to generate_recv_msg_data");
    return;
  }

  (void)add_new_message(sub_data, recv_data);
}

// callback: the typical ``callback`` function. ``context`` will be passed as its last argument.
// dropper: allows the callback's state to be freed. ``context`` will be passed as its last argument.
// context: a pointer to an arbitrary state.

bool declaration_subscription_data(ZenohPicoSubData *sub_data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  ZenohPicoSession *session = sub_data->node->session;

  {
    // declare subscriber
    z_subscriber_options_t options;
    z_subscriber_options_default(&options);

    z_owned_closure_sample_t sub_callback;
    z_closure(&sub_callback, _sub_data_handler, 0, (void *)sub_data);

    z_view_keyexpr_t ke;
    const z_loaned_string_t *keyexpr = z_loan(sub_data->topic_key);
    z_view_keyexpr_from_substr(&ke, z_string_data(keyexpr), z_string_len(keyexpr));
    if(_Z_IS_ERR(z_declare_subscriber(z_loan(session->session),
				      &sub_data->subscriber,
				      z_loan(ke),
				      z_move(sub_callback),
				      &options))){
      RMW_ZENOH_LOG_INFO("Unable to declare subscriber.");
      return false;
    }
  }

  {
    // liveliness tokendeclare
    z_view_keyexpr_t ke;
    const z_loaned_string_t *keyexpr = z_loan(sub_data->topic_key);
    z_view_keyexpr_from_substr(&ke, z_string_data(keyexpr), z_string_len(keyexpr));
    if(_Z_IS_ERR(z_liveliness_declare_token(z_loan(session->session),
					    &sub_data->token,
					    z_loan(ke),
					    NULL))){
      RMW_ZENOH_LOG_INFO("Unable to declare token.");
      return false;
    }
  }

  return true;
}

bool undeclaration_subscription_data(ZenohPicoSubData *sub_data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  z_undeclare_subscriber(z_move(sub_data->subscriber));

  z_liveliness_undeclare_token(z_move(sub_data->token));

  return true;
}

void subscription_condition_trigger(ZenohPicoSubData *sub_data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  z_mutex_lock(z_loan_mut(sub_data->condition_mutex));

  if(sub_data->wait_set_data != NULL) {
    ZenohPicoWaitSetData * wait_set_data = sub_data->wait_set_data;

    wait_condition_lock(wait_set_data);

    wait_condition_triggered(wait_set_data, true);
    wait_condition_signal(wait_set_data);

    wait_condition_unlock(wait_set_data);
  }

  z_mutex_unlock(z_loan_mut(sub_data->condition_mutex));
}

bool subscription_condition_check_and_attach(ZenohPicoSubData *sub_data,
					     ZenohPicoWaitSetData * wait_set_data)
{
  bool ret;

  RMW_ZENOH_FUNC_ENTRY(NULL);

  z_mutex_lock(z_loan_mut(sub_data->condition_mutex));

  if(!recv_msg_list_empty(&sub_data->message_queue)){
    RMW_ZENOH_LOG_INFO("queue_has_data_and_attach_condition_if_notmessage_queue size is %d",
		       recv_msg_list_count(&sub_data->message_queue));
    z_mutex_unlock(z_loan_mut(sub_data->condition_mutex));
    return true;
  }

  sub_data->wait_set_data = wait_set_data;

  z_mutex_unlock(z_loan_mut(sub_data->condition_mutex));

  return false;
}

bool subscription_condition_detach_and_queue_is_empty(ZenohPicoSubData *sub_data)
{
  bool ret;

  RMW_ZENOH_FUNC_ENTRY(NULL);

  z_mutex_lock(z_loan_mut(sub_data->condition_mutex));

  sub_data->wait_set_data = NULL;
  ret = recv_msg_list_empty(&sub_data->message_queue);

  z_mutex_unlock(z_loan_mut(sub_data->condition_mutex));

  return ret;
}

static rmw_subscription_t * _rmw_subscription_generate(rmw_context_t *context,
						       ZenohPicoSubData *sub_data,
						       const rmw_subscription_options_t *options)
{
  RMW_ZENOH_FUNC_ENTRY(context);

  (void)context;

  rmw_subscription_t * rmw_subscription = Z_MALLOC(sizeof(rmw_subscription_t));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_subscription,
    "failed to allocate memory for the subscription",
    return NULL);

  const z_loaned_string_t *_name = z_loan(sub_data->entity->topic_info->name);

  rmw_subscription->implementation_identifier	= rmw_get_implementation_identifier();
  rmw_subscription->topic_name			= zenoh_pico_string_clone(_name);
  rmw_subscription->can_loan_messages		= false;
  rmw_subscription->is_cft_enabled		= false;
  rmw_subscription->data                        = (void *)sub_data;

  memcpy(&rmw_subscription->options, options, sizeof(rmw_subscription_options_t));

  return rmw_subscription;
}

static rmw_ret_t _rmw_subscription_destroy(rmw_subscription_t * sub)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  RMW_CHECK_ARGUMENT_FOR_NULL(sub, RMW_RET_INVALID_ARGUMENT);

  ZenohPicoSubData *sub_data = (ZenohPicoSubData *)sub->data;

  if(sub_data != NULL){
    undeclaration_subscription_data(sub_data);
    zenoh_pico_destroy_subscription_data(sub_data);
  }

  Z_FREE(sub->topic_name);
  Z_FREE(sub);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_subscription_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_subscription_allocation_t * allocation)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

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
  RMW_ZENOH_FUNC_ENTRY(NULL);
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
  RMW_ZENOH_FUNC_ENTRY(topic_name);

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
  z_owned_string_t _hash_data;
  convert_hash(type_hash, &_hash_data);
  RMW_ZENOH_LOG_INFO("hash[%s] = [%*s][%d]", topic_name,
		     Z_STRING_LEN(_hash_data),
		     Z_STRING_VAL(_hash_data),
		     Z_STRING_LEN(_hash_data));

  // generate message type
  const message_type_support_callbacks_t *callbacks
    = (const message_type_support_callbacks_t *)(type_support->data);

  z_owned_string_t _type_name;
  convert_message_type(callbacks, &_type_name);
  RMW_ZENOH_LOG_INFO("type_name = [%.*s][%d]",
		     Z_STRING_LEN(_type_name),
		     Z_STRING_VAL(_type_name),
		     Z_STRING_LEN(_type_name));

  ZenohPicoTopicInfo *_topic_info	= NULL;
  ZenohPicoNodeInfo *_node_info		= NULL;
  ZenohPicoEntity *_entity		= NULL;
  ZenohPicoSubData *_sub_data		= NULL;
  size_t _entity_id			= 0;

  {
    _topic_info = zenoh_pico_generate_topic_info(topic_name,
						 qos_profile,
						 z_loan(_type_name),
						 z_loan(_hash_data));
    if (_topic_info == NULL)
      goto error;
  }

  // clone node_info
  {
    _node_info = zenoh_pico_clone_node_info(node_data->entity->node_info);
    if(_node_info == NULL)
      goto error;
  }

  // generate entity data
  {
    _entity_id = zenoh_pico_get_next_entity_id();
    ZenohPicoSession *_session = node_data->session;
    z_id_t _zid = z_info_zid(z_loan(_session->session));
    _entity = zenoh_pico_generate_entity( &_zid,
					  _entity_id,
					  node_data->id,
					  Subscription,
					  _node_info,
					  _topic_info);
    if(_entity == NULL)
      goto error;
  }

  // generate subscriber data
  {
    ZenohPicoNodeData *_node = zenoh_pico_loan_node_data(node_data);
    _sub_data = zenoh_pico_generate_subscription_data(_entity_id,
						      _node,
						      _entity,
						      qos_profile,
						      type_support,
						      callbacks);
    if(_sub_data == NULL) goto error;
  }

  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    zenoh_pico_debug_subscription_data(_sub_data);
  }

  rmw_subscription_t * rmw_subscription = _rmw_subscription_generate(node->context,
								     _sub_data,
								     subscription_options);
  z_drop(z_move(_hash_data));
  z_drop(z_move(_type_name));

  if(rmw_subscription == NULL)
    goto error;

  if(!declaration_subscription_data(_sub_data))
    goto error;

  return rmw_subscription;

  error:
  if(_topic_info != NULL)
    zenoh_pico_destroy_topic_info(_topic_info);

  if(_node_info != NULL)
    zenoh_pico_destroy_node_info(_node_info);

  if(_entity != NULL)
    zenoh_pico_destroy_entity(_entity);

  if(_sub_data != NULL)
    zenoh_pico_destroy_subscription_data(_sub_data);

  return NULL;
}

rmw_ret_t
rmw_destroy_subscription(
  rmw_node_t * node,
  rmw_subscription_t * subscription)
{
  RMW_ZENOH_FUNC_ENTRY(node);

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
  RMW_ZENOH_FUNC_ENTRY(subscription);
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
  RMW_ZENOH_FUNC_ENTRY(subscription);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_subscription_set_content_filter(
  rmw_subscription_t * subscription,
  const rmw_subscription_content_filter_options_t * options)
{
  RMW_ZENOH_FUNC_ENTRY(subscription);
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
  RMW_ZENOH_FUNC_ENTRY(subscription);
  (void) subscription;
  (void) allocator;
  (void) options;

  return RMW_RET_UNSUPPORTED;
}
