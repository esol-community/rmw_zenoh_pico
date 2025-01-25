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

#include "rmw/ret_types.h"
#include "rmw/types.h"
#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_entity.h"
#include "zenoh-pico/api/macros.h"
#include "zenoh-pico/api/primitives.h"
#include "zenoh-pico/api/types.h"
#include <rmw_zenoh_pico/config.h>

#include <rosidl_typesupport_microxrcedds_c/identifier.h>
#include <rosidl_typesupport_microxrcedds_c/message_type_support.h>

#include <rmw/allocators.h>
#include <rmw/rmw.h>
#include <rmw/validate_full_topic_name.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

z_mutex_t mutex_ZenohPicoPubData;

ZenohPicoPubData * zenoh_pico_generate_publisher_data(
  size_t pub_id,
  ZenohPicoNodeData *node,
  ZenohPicoEntity *entity,
  const rosidl_message_type_support_t * type_support,
  const message_type_support_callbacks_t *callbacks,
  rmw_qos_profile_t *qos_profile)
{
  if((node == NULL) || (entity == NULL))
    return NULL;

  ZenohPicoPubData *pub_data = NULL;
  ZenohPicoGenerateData(pub_data, ZenohPicoPubData);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    pub_data,
    "failed to allocate struct for the ZenohPicoPubData",
    return NULL);

  pub_data->id         = pub_id;
  pub_data->node	= node;
  pub_data->entity	= entity;

  pub_data->callbacks  = callbacks;
  pub_data->adapted_qos_profile = *qos_profile;

  ZenohPicoNodeInfo  *node_info  = entity->node_info;
  ZenohPicoTopicInfo *topic_info = entity->topic_info;

  // generate key from entity data
  pub_data->token_key = generate_liveliness(entity);

  // generate topic key
  pub_data->topic_key = ros_topic_name_to_zenoh_key(Z_STRING_VAL(node_info->domain),
						     Z_STRING_VAL(topic_info->name),
						     Z_STRING_VAL(topic_info->type),
						     Z_STRING_VAL(topic_info->hash));

  return pub_data;
}

bool zenoh_pico_destroy_publisher_data(ZenohPicoPubData *pub_data)
{
  RMW_ZENOH_FUNC_ENTRY();

  (void)undeclaration_publisher_data(pub_data);

  Z_STRING_FREE(pub_data->token_key);
  Z_STRING_FREE(pub_data->topic_key);

  if(pub_data->node != NULL){
    zenoh_pico_destroy_node_data(pub_data->node);
    pub_data->node = NULL;
  }

  if(pub_data->entity != NULL){
    zenoh_pico_destroy_entity(pub_data->entity);
    pub_data->entity = NULL;
  }

  ZenohPicoDestroyData(pub_data, ZenohPicoPubData);

  return true;
}

void zenoh_pico_debug_publisher_data(ZenohPicoPubData *pub_data)
{
  printf("--------- publisher data ----------\n");
  printf("ref = %d\n", pub_data->ref);

  Z_STRING_PRINTF(pub_data->token_key, token_key);
  Z_STRING_PRINTF(pub_data->topic_key, topic_key);

  // debug node member
  zenoh_pico_debug_node_data(pub_data->node);

  // debug entity member
  zenoh_pico_debug_entity(pub_data->entity);
}

static void _token_handler(const z_sample_t *sample, void *ctx) {
  RMW_ZENOH_FUNC_ENTRY();

  ZenohPicoPubData *pub_data = (ZenohPicoPubData *)ctx;

  return;
}

bool declaration_publisher_data(ZenohPicoPubData *pub_data)
{
  RMW_ZENOH_FUNC_ENTRY();

  ZenohPicoSession *session = pub_data->node->session;

  z_publisher_options_t options = z_publisher_options_default();
  options.congestion_control = Z_CONGESTION_CONTROL_DROP;
  if(pub_data->adapted_qos_profile.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL &&
     pub_data->adapted_qos_profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    {
      options.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
    }

  pub_data->publisher = z_declare_publisher(z_loan(session->session),
					     z_keyexpr(pub_data->topic_key.val),
					     &options);
  if(!z_check(pub_data->publisher)) {
    RMW_ZENOH_LOG_DEBUG("Unable to declare publisher.");
    return false;
  }

  // liveliness tokendeclare
  const char *keyexpr = Z_STRING_VAL(pub_data->token_key);
  RMW_ZENOH_LOG_DEBUG("Declaring subscriber key expression '%s'...", keyexpr);

  z_owned_closure_sample_t token_callback_ = z_closure(_token_handler, 0, (void *)pub_data);
  pub_data->token = z_declare_subscriber(z_loan(session->session),
					 z_keyexpr(keyexpr),
					 z_move(token_callback_),
					 NULL);
  if (!z_check(pub_data->token)) {
    RMW_ZENOH_LOG_DEBUG("Unable to declare token.");
    return false;
  }

  return true;
}

bool undeclaration_publisher_data(ZenohPicoPubData *pub_data)
{
  RMW_ZENOH_FUNC_ENTRY();

  if (z_check(pub_data->token)) {
    z_undeclare_subscriber(z_move(pub_data->token));
  }

  if (z_check(pub_data->publisher)) {
    z_undeclare_publisher(z_move(pub_data->publisher));
  }

  return true;
}

static rmw_publisher_t *_rmw_publisher_generate(rmw_context_t *context,
						ZenohPicoPubData *pub_data,
						const rmw_publisher_options_t *options)
{
  RMW_ZENOH_FUNC_ENTRY();

  rmw_publisher_t * rmw_publisher = Z_MALLOC(sizeof(rmw_publisher_t));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_publisher,
    "failed to allocate memory for the publisher",
    return NULL);

  rmw_publisher->implementation_identifier	= rmw_get_implementation_identifier();
  rmw_publisher->topic_name			= Z_STRING_VAL(pub_data->entity->topic_info->name);
  rmw_publisher->can_loan_messages		= false;
  rmw_publisher->data				= (void *)pub_data;

  memcpy(&rmw_publisher->options, options, sizeof(rmw_subscription_options_t));

  return rmw_publisher;
}

static rmw_ret_t _rmw_publisher_destroy(rmw_publisher_t * pub)
{
  RMW_ZENOH_FUNC_ENTRY();

  RMW_CHECK_ARGUMENT_FOR_NULL(pub, RMW_RET_INVALID_ARGUMENT);

  ZenohPicoPubData *pub_data = (ZenohPicoPubData *)pub->data;

  if(pub_data != NULL){
    undeclaration_publisher_data(pub_data);
    zenoh_pico_destroy_publisher_data(pub_data);
  }

  Z_FREE(pub);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_publisher_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_publisher_allocation_t * allocation)
{
  (void)type_support;
  (void)message_bounds;
  (void)allocation;
  RMW_SET_ERROR_MSG("rmw_init_publisher_allocation: unimplemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_fini_publisher_allocation(
  rmw_publisher_allocation_t * allocation)
{
  (void)allocation;
  RMW_SET_ERROR_MSG("rmw_fini_publisher_allocation: unimplemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_publisher_t *
rmw_create_publisher(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_publisher_options_t * publisher_options)
{
  RMW_ZENOH_FUNC_ENTRY();

  RMW_CHECK_ARGUMENT_FOR_NULL(node, NULL);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node->implementation_identifier,
    return NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, NULL);
  if (topic_name[0] == '\0') {
    RMW_SET_ERROR_MSG("topic_name argument is an empty string");
    return NULL;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->data, NULL);

  if (!qos_profile->avoid_ros_namespace_conventions) {
    int validation_result = RMW_TOPIC_VALID;
    rmw_ret_t ret = rmw_validate_full_topic_name(topic_name,
						 &validation_result,
						 NULL);
    if (RMW_RET_OK != ret) {
      return NULL;
    }
    if (RMW_TOPIC_VALID != validation_result) {
      const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid topic name: %s", reason);
      return NULL;
    }
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_options, NULL);
  if (publisher_options->require_unique_network_flow_endpoints == RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_STRICTLY_REQUIRED)
    {
      RMW_SET_ERROR_MSG(
	"Strict requirement on unique network flow endpoints for publishers not supported");
      return NULL;
    }

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

  // Get the RMW type support.
  const rosidl_message_type_support_t * type_support = find_message_type_support(type_supports);
  if (type_support == NULL) {
    // error was already set by find_message_type_support
    RMW_ZENOH_LOG_INFO("type_support is null");
    return NULL;
  }
  RMW_ZENOH_LOG_INFO("typesupport_identifier = [%s]", type_support->typesupport_identifier);

  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context,
    "expected initialized context",
    return NULL);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context->impl,
    "expected initialized context impl",
    return NULL);

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
							 Publisher,
							 _node_info,
							 _topic_info);
  ZenohPicoNodeData *_node = zenoh_pico_loan_node_data(node_data);
  ZenohPicoPubData *_pub_data = zenoh_pico_generate_publisher_data(_entity_id,
								   _node,
								   _entity,
								   type_support,
								   callbacks,
								   &_qos_profile);
  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    zenoh_pico_debug_publisher_data(_pub_data);
  }

  rmw_publisher_t * rmw_publisher = _rmw_publisher_generate(node->context,
							    _pub_data,
							    publisher_options);
  declaration_publisher_data(_pub_data);

  return rmw_publisher;
}

rmw_ret_t
rmw_destroy_publisher(
  rmw_node_t * node,
  rmw_publisher_t * publisher)
{
  RMW_ZENOH_FUNC_ENTRY();

  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  return _rmw_publisher_destroy(publisher);
}

rmw_ret_t
rmw_publisher_count_matched_subscriptions(
  const rmw_publisher_t * publisher,
  size_t * subscription_count)
{
  (void)publisher;
  (void)subscription_count;
  RMW_ZENOH_LOG_INFO(
    "Function not available; enable RMW_UXRCE_GRAPH configuration profile before using");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_publisher_assert_liveliness(
  const rmw_publisher_t * publisher)
{
  (void)publisher;
  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_publisher_get_actual_qos(
  const rmw_publisher_t * publisher,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_borrow_loaned_message(
  const rmw_publisher_t * publisher,
  const rosidl_message_type_support_t * type_support,
  void ** ros_message)
{
  (void)publisher;
  (void)type_support;
  (void)ros_message;

  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_return_loaned_message_from_publisher(
  const rmw_publisher_t * publisher,
  void * loaned_message)
{
  (void)publisher;
  (void)loaned_message;

  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}
