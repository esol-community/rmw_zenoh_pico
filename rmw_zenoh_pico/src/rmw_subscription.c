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

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

static bool declaration_subscription_data(ZenohPicoSubData *sub_data);
static bool undeclaration_subscription_data(ZenohPicoSubData *sub_data);

static void add_new_message(ZenohPicoSubData *sub_data, ReceiveMessageData *recv_data);
static void subscription_condition_trigger(ZenohPicoSubData *sub_data);

static ZenohPicoSubData * zenoh_pico_generate_subscription_data(
  ZenohPicoNodeData *node,
  const char * topic_name,
  const rosidl_message_type_support_t * type_support,
  const rmw_qos_profile_t *qos_profile)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  RMW_CHECK_ARGUMENT_FOR_NULL(node, NULL);

  ZenohPicoEntity *entity      = NULL;
  ZenohPicoNodeInfo *node_info = NULL;

  // generate entity data
  ZenohPicoSession *session = node->session;
  z_id_t zid = z_info_zid(z_loan(session->session));

  node_info = ZenohPicoDataRefClone(node->entity->node_info);
  entity = zenoh_pico_generate_subscription_entity(&zid,
						   node->id,
						   node_info,
						   topic_name,
						   type_support,
						   qos_profile);
  if(entity == NULL)
    goto error;

  ZenohPicoSubData *sub_data = NULL;
  sub_data = ZenohPicoDataGenerate(sub_data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    sub_data,
    "failed to allocate struct for the ZenohPicoSubData",
    goto error);

  sub_data->id			= entity->id;
  sub_data->node		= node;
  sub_data->entity		= entity;
  sub_data->callbacks		= (const message_type_support_callbacks_t *)(type_support->data);
  sub_data->adapted_qos_profile = *qos_profile;

  // generate key from entity data
  if(_Z_IS_ERR(generate_liveliness(entity, &sub_data->liveliness_key))){
    RMW_SET_ERROR_MSG("failed generate_liveliness()");
    goto error;
  }

  // generate topic key
  ZenohPicoTopicInfo *topic_info = entity->topic_info;
  if(_Z_IS_ERR(ros_topic_name_to_zenoh_key(z_loan(node_info->domain),
					   z_loan(topic_info->name),
					   z_loan(topic_info->type),
					   z_loan(topic_info->hash),
					   &sub_data->topic_key))){
    RMW_SET_ERROR_MSG("failed ros_topic_name_to_zenoh_key()");
    goto error;
  }

  // init receive message list
  recv_msg_list_init(&sub_data->message_queue);

  // init data callback manager
  data_callback_init(&sub_data->data_event_mgr);

  // init rmw_wait condition
  z_mutex_init(&sub_data->condition_mutex);
  sub_data->wait_set_data = NULL;

  return sub_data;

  error:
  if(node_info != NULL)
    zenoh_pico_destroy_node_info(node_info);

  if(entity != NULL)
    zenoh_pico_destroy_entity(entity);

  if(sub_data != NULL){
    ZenohPicoDataDestroy(sub_data);
  }

  return NULL;
}

static bool zenoh_pico_destroy_subscription_data(ZenohPicoSubData *sub_data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, false);

  if(ZenohPicoDataRelease(sub_data)){

    (void)undeclaration_subscription_data(sub_data);

    z_drop(z_move(sub_data->liveliness_key));
    z_drop(z_move(sub_data->topic_key));

    z_drop(z_move(sub_data->liveliness));
    z_drop(z_move(sub_data->topic));

    while(!recv_msg_list_empty(&sub_data->message_queue)) {
      ReceiveMessageData *msg_data = recv_msg_list_pop(&sub_data->message_queue);
      ZenohPicoDataDestroy(msg_data);
    }

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

    ZenohPicoDataDestroy(sub_data);
  }

  return true;
}

static void zenoh_pico_debug_subscription_data(ZenohPicoSubData *sub_data)
{
  printf("--------- subscription data ----------\n");
  printf("ref = %d\n", sub_data->ref);

  Z_STRING_PRINTF(sub_data->liveliness_key, liveliness_key);
  Z_STRING_PRINTF(sub_data->topic_key, topic_key);

  printf("message_queue = %d\n", recv_msg_list_count(&sub_data->message_queue));

  // debug entity member
  zenoh_pico_debug_entity(sub_data->entity);
}

static void add_new_subscription_message(ZenohPicoSubData *sub_data, ReceiveMessageData *recv_data)
{
  // rmw_zenoh_pico_debug_recv_msg_data(recv_data);

  (void)recv_msg_list_append(&sub_data->message_queue, recv_data);

  (void)data_callback_trigger(&sub_data->data_event_mgr);

  (void)subscription_condition_trigger(sub_data);

  return;
}

static void _subscription_data_handler(z_loaned_sample_t *sample, void *ctx) {
  RMW_ZENOH_FUNC_ENTRY(sample);

  ZenohPicoSubData *sub_data = (ZenohPicoSubData *)ctx;
  if (sub_data == NULL) {
    z_view_string_t keystr;
    z_keyexpr_as_view_string(z_sample_keyexpr(sample), &keystr);

    RMW_ZENOH_LOG_INFO("keystr is %s ", z_string_data(z_loan(keystr)));
    RMW_ZENOH_LOG_ERROR("Unable to obtain ZenohPicoSubData from data for "
			"subscription for %s",
			z_string_data(z_loan(keystr)));
    return;
  }

  ReceiveMessageData *recv_data;
  recv_data = rmw_zenoh_pico_generate_recv_sample_msg_data(sample,zenoh_pico_gen_timestamp());
  RMW_CHECK_FOR_NULL_WITH_MSG(
    recv_data,
    "unable to generate_recv_msg_data",
    return);

  (void)add_new_subscription_message(sub_data, recv_data);
}

// callback: the typical ``callback`` function. ``context`` will be passed as its last argument.
// dropper: allows the callback's state to be freed. ``context`` will be passed as its last argument.
// context: a pointer to an arbitrary state.

static bool declaration_subscription_data(ZenohPicoSubData *sub_data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  ZenohPicoSession *session = sub_data->node->session;

  // declare subscriber
  z_subscriber_options_t options;
  z_subscriber_options_default(&options);

  z_owned_closure_sample_t sub_callback;
  z_closure(&sub_callback, _subscription_data_handler, 0, (void *)sub_data);

  z_view_keyexpr_t ke;
  const z_loaned_string_t *keyexpr = z_loan(sub_data->topic_key);
  z_view_keyexpr_from_substr(&ke, z_string_data(keyexpr), z_string_len(keyexpr));
  if(_Z_IS_ERR(z_declare_subscriber(z_loan(session->session),
				    &sub_data->topic,
				    z_loan(ke),
				    z_move(sub_callback),
				    &options))){
    RMW_ZENOH_LOG_INFO("Unable to declare subscriber.");
    return false;
  }

  return declaration_liveliness(session, z_loan(sub_data->liveliness_key), &sub_data->liveliness);
}

static bool undeclaration_subscription_data(ZenohPicoSubData *sub_data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  z_undeclare_subscriber(z_move(sub_data->topic));

  z_liveliness_undeclare_token(z_move(sub_data->liveliness));

  return true;
}

static void subscription_condition_trigger(ZenohPicoSubData *sub_data)
{
  ZenohPicoWaitCondition cond;
  cond.condition_mutex		= z_loan_mut(sub_data->condition_mutex);
  cond.msg_queue		= &sub_data->message_queue;
  cond.wait_set_data_ptr	= &sub_data->wait_set_data;

  zenoh_pico_condition_trigger(&cond);
}

bool subscription_condition_check_and_attach(ZenohPicoSubData *sub_data,
					     ZenohPicoWaitSetData * wait_set_data)
{
  ZenohPicoWaitCondition cond;
  cond.condition_mutex		= z_loan_mut(sub_data->condition_mutex);
  cond.msg_queue		= &sub_data->message_queue;
  cond.wait_set_data_ptr	= &sub_data->wait_set_data;

  return zenoh_pico_condition_check_and_attach(&cond, wait_set_data);
}

bool subscription_condition_detach_and_queue_is_empty(ZenohPicoSubData *sub_data)
{
  ZenohPicoWaitCondition cond;
  cond.condition_mutex		= z_loan_mut(sub_data->condition_mutex);
  cond.msg_queue		= &sub_data->message_queue;
  cond.wait_set_data_ptr	= &sub_data->wait_set_data;

  return zenoh_pico_condition_detach_and_queue_is_empty(&cond);
}

rmw_ret_t
rmw_init_subscription_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_subscription_allocation_t * allocation)
{
  // RMW_ZENOH_FUNC_ENTRY(NULL);

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

  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context,
    "expected initialized context",
    return NULL);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context->impl,
    "expected initialized context impl",
    return NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->data, NULL);

  if (!qos_profile->avoid_ros_namespace_conventions) {
    if(!rmw_zenoh_pico_check_validate_name(topic_name))
      return NULL;
  }

  // Get node data
  ZenohPicoNodeData *node_data = (ZenohPicoNodeData *)node->data;
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node_data, "unable to create subscription as node_data is invalid.",
    return NULL);

  // scan hash data by type_support
  const rosidl_message_type_support_t * type_support = find_message_type_support(type_supports);
  if (type_support == NULL) {
    // error was already set by find_message_type_support
    RMW_ZENOH_LOG_INFO("type_support is null");
    return NULL;
  }
  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    RMW_ZENOH_LOG_INFO("typesupport_identifier = [%s]", type_support->typesupport_identifier);
  }

  ZenohPicoSubData * sub_data = zenoh_pico_generate_subscription_data(
    ZenohPicoDataRefClone(node_data),
    topic_name,
    type_support,
    qos_profile);

  if(sub_data == NULL){
    goto error;
  }

  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    zenoh_pico_debug_subscription_data(sub_data);
  }

  rmw_subscription_t * rmw_subscription = Z_MALLOC(sizeof(rmw_subscription_t));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_subscription,
    "failed to allocate memory for the subscription",
    goto error);
  memset(rmw_subscription, 0, sizeof(rmw_subscription_t));

  const z_loaned_string_t *_name = z_loan(sub_data->entity->topic_info->name);

  rmw_subscription->implementation_identifier	= rmw_get_implementation_identifier();
  rmw_subscription->topic_name			= zenoh_pico_string_clone(_name);
  rmw_subscription->can_loan_messages		= false;
  rmw_subscription->is_cft_enabled		= false;
  rmw_subscription->data                        = (void *)sub_data;

  memcpy(&rmw_subscription->options, subscription_options, sizeof(rmw_subscription_options_t));

  if(!declaration_subscription_data(sub_data))
    goto error;

  return rmw_subscription;

  error:
  if(sub_data != NULL)
    zenoh_pico_destroy_subscription_data(sub_data);

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

  ZenohPicoSubData *sub_data = (ZenohPicoSubData *)subscription->data;

  if(sub_data != NULL){
    undeclaration_subscription_data(sub_data);
    zenoh_pico_destroy_subscription_data(sub_data);
    subscription->data = NULL;
  }

  Z_FREE(subscription->topic_name);
  Z_FREE(subscription);

  return RMW_RET_OK;

  // return _rmw_subscription_destroy(subscription);
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
    "Function not available");
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

rmw_ret_t
rmw_subscription_get_network_flow_endpoints(
  const rmw_subscription_t * subscription,
  rcutils_allocator_t * allocator,
  rmw_network_flow_endpoint_array_t * network_flow_endpoint_array)
{
  RMW_ZENOH_FUNC_ENTRY(subscription);

  (void) subscription;
  (void) allocator;
  (void) network_flow_endpoint_array;

  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_get_subscriptions_info_by_topic(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  rmw_topic_endpoint_info_array_t * subscriptions_info)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  (void)node;
  (void)allocator;
  (void)topic_name;
  (void)no_mangle;
  (void)subscriptions_info;
  RMW_ZENOH_LOG_INFO("Function not available");
  return RMW_RET_UNSUPPORTED;
}
