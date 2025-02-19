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

#include <rmw_zenoh_pico/config.h>

#include <rmw/allocators.h>
#include <rmw/rmw.h>
#include <rmw/validate_full_topic_name.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

static bool declaration_publisher_data(ZenohPicoPubData *pub_data);
static bool undeclaration_publisher_data(ZenohPicoPubData *pub_data);

static ZenohPicoPubData * zenoh_pico_generate_publisher_data(
  ZenohPicoNodeData *node,
  const char * topic_name,
  const rosidl_message_type_support_t * type_support,
  const rmw_qos_profile_t *qos_profile)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  RMW_CHECK_ARGUMENT_FOR_NULL(node, NULL);

  ZenohPicoNodeInfo *node_info	= NULL;
  ZenohPicoEntity *entity	= NULL;
  ZenohPicoPubData *pub_data	= NULL;

  // clone node_info
  node_info = zenoh_pico_clone_node_info(node->entity->node_info);

  // generate entity data
  ZenohPicoSession *session = node->session;
  z_id_t zid = z_info_zid(z_loan(session->session));
  entity = zenoh_pico_generate_publisher_entity(&zid,
						node->id,
						node_info,
						topic_name,
						type_support,
						qos_profile);
  if(entity == NULL)
    goto error;

  ZenohPicoGenerateData(pub_data, ZenohPicoPubData);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    pub_data,
    "failed to allocate struct for the ZenohPicoPubData",
    goto error);

  pub_data->id				= entity->id;
  pub_data->node			= node;
  pub_data->entity			= entity;
  pub_data->callbacks			= (const message_type_support_callbacks_t *)(type_support->data);
  pub_data->adapted_qos_profile		= *qos_profile;

  // generate key from entity data
  z_string_empty(&pub_data->liveliness_key);
  if(_Z_IS_ERR(generate_liveliness(entity, &pub_data->liveliness_key))){
    RMW_SET_ERROR_MSG("failed generate_liveliness()");
    goto error;
  }

  // generate topic key
  z_string_empty(&pub_data->topic_key);
  ZenohPicoTopicInfo *topic_info = entity->topic_info;
  if(_Z_IS_ERR(ros_topic_name_to_zenoh_key(z_loan(node_info->domain),
					   z_loan(topic_info->name),
					   z_loan(topic_info->type),
					   z_loan(topic_info->hash),
					   &pub_data->topic_key))){
    RMW_SET_ERROR_MSG("failed ros_topic_name_to_zenoh_key()");
    goto error;
  }

  // generate mutex
  z_mutex_init(&pub_data->mutex);

  // generate gid
  uint8_t _gid[RMW_GID_STORAGE_SIZE];
  zenoh_pico_gen_gid(z_loan(pub_data->topic_key), _gid);

  z_slice_copy_from_buf(&pub_data->attachment.gid, _gid, sizeof(_gid));
  pub_data->attachment.sequence_num = 0;

  return pub_data;

  error:
  if(topic_info != NULL)
    zenoh_pico_destroy_topic_info(topic_info);

  if(node_info != NULL)
    zenoh_pico_destroy_node_info(node_info);

  if(entity != NULL)
    zenoh_pico_destroy_entity(entity);

  if(pub_data != NULL)
    ZenohPicoDestroyData(pub_data, ZenohPicoPubData);

  return NULL;
}

static bool zenoh_pico_destroy_publisher_data(ZenohPicoPubData *pub_data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  RMW_CHECK_ARGUMENT_FOR_NULL(pub_data, false);

  (void)undeclaration_publisher_data(pub_data);

  z_drop(z_move(pub_data->liveliness_key));
  z_drop(z_move(pub_data->topic_key));

  z_drop(z_move(pub_data->liveliness));
  z_drop(z_move(pub_data->topic));

  attachment_destroy(&pub_data->attachment);
  z_drop(z_move(pub_data->mutex));

  if(pub_data->node != NULL){
    (void)zenoh_pico_destroy_node_data(pub_data->node);
    pub_data->node = NULL;
  }

  if(pub_data->entity != NULL){
    (void)zenoh_pico_destroy_entity(pub_data->entity);
    pub_data->entity = NULL;
  }

  ZenohPicoDestroyData(pub_data, ZenohPicoPubData);

  return true;
}

static void zenoh_pico_debug_publisher_data(ZenohPicoPubData *pub_data)
{
  printf("--------- publisher data ----------\n");
  printf("ref = %d\n", pub_data->ref);

  Z_STRING_PRINTF(pub_data->liveliness_key, liveliness_key);
  Z_STRING_PRINTF(pub_data->topic_key, topic_key);

  // debug attachment
  attachment_debug(&pub_data->attachment);

  // debug entity member
  zenoh_pico_debug_entity(pub_data->entity);
}

static bool declaration_publisher_data(ZenohPicoPubData *pub_data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  ZenohPicoSession *session = pub_data->node->session;

  // liveliness token declare
  (void)declaration_liveliness(session, z_loan(pub_data->liveliness_key), &pub_data->liveliness);


  z_publisher_options_t options;
  z_publisher_options_default(&options);
  options.congestion_control = Z_CONGESTION_CONTROL_DROP;
  if(pub_data->adapted_qos_profile.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL &&
     pub_data->adapted_qos_profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    {
      options.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
    }

  // topic declare
  z_view_keyexpr_t ke;
  const z_loaned_string_t *keyexpr = z_loan(pub_data->topic_key);
  z_view_keyexpr_from_substr(&ke, z_string_data(keyexpr), z_string_len(keyexpr));
  if(_Z_IS_ERR(z_declare_publisher(z_loan(session->session),
				   &pub_data->topic,
				   z_loan(ke),
				   &options))){
    RMW_ZENOH_LOG_INFO("Unable to declare publisher.");
    return false;
  }

  return true;
}

static bool undeclaration_publisher_data(ZenohPicoPubData *pub_data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  z_undeclare_publisher(z_move(pub_data->topic));

  z_liveliness_undeclare_token(z_move(pub_data->liveliness));

  return true;
}

rmw_ret_t
rmw_init_publisher_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_publisher_allocation_t * allocation)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

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
  RMW_ZENOH_FUNC_ENTRY(NULL);

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
  RMW_ZENOH_FUNC_ENTRY(node);

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

  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context,
    "expected initialized context",
    return NULL);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context->impl,
    "expected initialized context impl",
    return NULL);

  if (!qos_profile->avoid_ros_namespace_conventions) {
    if(!rmw_zenoh_pico_check_validate_name(topic_name))
      return NULL;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_options, NULL);
  if (publisher_options->require_unique_network_flow_endpoints == RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_STRICTLY_REQUIRED)
    {
      RMW_SET_ERROR_MSG(
	"Strict requirement on unique network flow endpoints for publishers not supported");
      return NULL;
    }

  // Get node data
  ZenohPicoNodeData *node_data = (ZenohPicoNodeData *)node->data;
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node_data, "unable to create subscription as node_data is invalid.",
    return NULL);

  // Get the RMW type support.
  const rosidl_message_type_support_t * type_support = find_message_type_support(type_supports);
  if (type_support == NULL) {
    // error was already set by find_message_type_support
    RMW_ZENOH_LOG_INFO("type_support is null");
    return NULL;
  }
  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    RMW_ZENOH_LOG_INFO("typesupport_identifier = [%s]", type_support->typesupport_identifier);
  }

  ZenohPicoPubData * pub_data = zenoh_pico_generate_publisher_data(
    zenoh_pico_loan_node_data(node_data),
    topic_name,
    type_support,
    qos_profile);

  if(pub_data == NULL){
    goto error;
  }

  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    zenoh_pico_debug_publisher_data(pub_data);
  }

  rmw_publisher_t * rmw_publisher = Z_MALLOC(sizeof(rmw_publisher_t));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_publisher,
    "failed to allocate memory for the publisher",
    goto error);
  memset((void *)rmw_publisher, 0, sizeof(rmw_publisher_t));

  const z_loaned_string_t *_name = z_loan(pub_data->entity->topic_info->name);

  rmw_publisher->implementation_identifier	= rmw_get_implementation_identifier();
  rmw_publisher->topic_name                     = zenoh_pico_string_clone(_name);
  rmw_publisher->can_loan_messages		= false;
  rmw_publisher->data				= (void *)pub_data;
  memcpy(&rmw_publisher->options, publisher_options, sizeof(rmw_publisher_options_t));

  if(!declaration_publisher_data(pub_data))
    goto error;

  return rmw_publisher;

  error:
  if(pub_data != NULL)
    zenoh_pico_destroy_publisher_data(pub_data);

  return NULL;
}

rmw_ret_t
rmw_destroy_publisher(
  rmw_node_t * node,
  rmw_publisher_t * publisher)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);


  ZenohPicoPubData *pub_data = (ZenohPicoPubData *)publisher->data;

  if(pub_data != NULL){
    undeclaration_publisher_data(pub_data);
    zenoh_pico_destroy_publisher_data(pub_data);
    publisher->data = NULL;
  }

  Z_FREE(publisher->topic_name);
  Z_FREE(publisher);

  return RMW_RET_OK;

  // return _rmw_publisher_destroy(publisher);
}

rmw_ret_t
rmw_publisher_count_matched_subscriptions(
  const rmw_publisher_t * publisher,
  size_t * subscription_count)
{
  RMW_ZENOH_FUNC_ENTRY(publisher);

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
  RMW_ZENOH_FUNC_ENTRY(publisher);

  (void)publisher;
  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_publisher_get_actual_qos(
  const rmw_publisher_t * publisher,
  rmw_qos_profile_t * qos)
{
  RMW_ZENOH_FUNC_ENTRY(publisher);

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
  RMW_ZENOH_FUNC_ENTRY(publisher);

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
  RMW_ZENOH_FUNC_ENTRY(publisher);

  (void)publisher;
  (void)loaned_message;

  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_get_gid_for_publisher(
  const rmw_publisher_t * publisher,
  rmw_gid_t * gid)
{
  RMW_ZENOH_FUNC_ENTRY(publisher);

  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  return RMW_RET_ERROR;
}
