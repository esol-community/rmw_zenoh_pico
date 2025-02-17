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

z_owned_mutex_t mutex_ZenohPicoClientData;

static void zenoh_pico_debug_client_data(ZenohPicoClientData *client_data);

static ZenohPicoClientData * zenoh_pico_generate_client_data(
  ZenohPicoNodeData *node,
  const char * topic_name,
  const rosidl_service_type_support_t *type_support,
  const rmw_qos_profile_t *qos_profile)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  RMW_CHECK_ARGUMENT_FOR_NULL(node, NULL);

  ZenohPicoNodeInfo *node_info		= NULL;
  ZenohPicoEntity *entity		= NULL;
  ZenohPicoClientData *client_data	= NULL;

  // clone node_info
  node_info = zenoh_pico_clone_node_info(node->entity->node_info);

  // generate entity data
  ZenohPicoSession *session = node->session;
  z_id_t zid = z_info_zid(z_loan(session->session));
  entity = zenoh_pico_generate_client_entity(&zid,
					     node->id,
					     node_info,
					     topic_name,
					     type_support,
					     qos_profile,
					     Client);
  if(entity == NULL)
    goto error;

  ZenohPicoGenerateData(client_data, ZenohPicoClientData);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    client_data,
    "failed to allocate struct for the ZenohPicoClientData",
    goto error);

  client_data->id			= entity->id;
  client_data->node			= node;
  client_data->entity			= entity;
  client_data->request_callback		= get_request_callback(type_support);
  client_data->response_callback	= get_response_callback(type_support);

  memcpy(&client_data->qos_profile, qos_profile, sizeof(rmw_qos_profile_t));

  // generate key from entity data
  if(_Z_IS_ERR(generate_liveliness(entity, &client_data->liveliness_key))){
    RMW_SET_ERROR_MSG("failed generate_liveliness()");
    goto error;
  }

  // generate topic key
  z_string_empty(&client_data->topic_key);
  ZenohPicoTopicInfo *topic_info = entity->topic_info;
  if(_Z_IS_ERR(ros_topic_name_to_zenoh_key(z_loan(node_info->domain),
					   z_loan(topic_info->name),
					   z_loan(topic_info->type),
					   z_loan(topic_info->hash),
					   &client_data->topic_key))){
    RMW_SET_ERROR_MSG("failed ros_topic_name_to_zenoh_key()");
    goto error;
  }

  // init reply message list
  recv_msg_list_init(&client_data->reply_queue);

  // init data callback manager
  data_callback_init(&client_data->data_callback_mgr);

  // init rmw_wait condition
  z_mutex_init(&client_data->condition_mutex);
  client_data->wait_set_data = NULL;

  // generate gid
  uint8_t _gid[RMW_GID_STORAGE_SIZE];
  zenoh_pico_gen_gid(z_loan(client_data->topic_key), _gid);

  z_slice_copy_from_buf(&client_data->attachment.gid, _gid, sizeof(_gid));
  client_data->attachment.sequence_num = 0;

  return client_data;

  error:
  if(node_info != NULL)
    zenoh_pico_destroy_node_info(node_info);

  if(entity != NULL)
    zenoh_pico_destroy_entity(entity);

  return NULL;
}

static bool zenoh_pico_destroy_client_data(ZenohPicoClientData *client_data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  RMW_CHECK_ARGUMENT_FOR_NULL(client_data, false);

  z_drop(z_move(client_data->liveliness_key));
  z_drop(z_move(client_data->token));

  attachment_destroy(&client_data->attachment);
  z_drop(z_move(client_data->mutex));

  if(client_data->node != NULL){
    (void)zenoh_pico_destroy_node_data(client_data->node);
    client_data->node = NULL;
  }

  if(client_data->entity != NULL){
    (void)zenoh_pico_destroy_entity(client_data->entity);
    client_data->entity = NULL;
  }


  ZenohPicoDestroyData(client_data, ZenohPicoClientData);

  return true;
}

static void zenoh_pico_debug_client_data(ZenohPicoClientData *client_data)
{
  printf("--------- client data ----------\n");
  printf("ref = %d\n", client_data->ref);

  Z_STRING_PRINTF(client_data->liveliness_key, liveliness_key);
  Z_STRING_PRINTF(client_data->topic_key, topic_key);

  printf("reply_queue = %d\n", recv_msg_list_count(&client_data->reply_queue));

  // debug attachment
  attachment_debug(&client_data->attachment);

  // debug entity member
  zenoh_pico_debug_entity(client_data->entity);

  return;
}

static bool declaration_client_data(ZenohPicoClientData *client_data)
{
  ZenohPicoSession *session = client_data->node->session;

  // liveliness token declare
  (void)declaration_liveliness(session, z_loan(client_data->liveliness_key), &client_data->token);

  return true;
}

static bool undeclaration_client_data(ZenohPicoClientData *client_data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  z_liveliness_undeclare_token(z_move(client_data->token));

  return true;
}

static void client_condition_trigger(ZenohPicoClientData *client_data)
{
  ZenohPicoWaitCondition cond;
  cond.condition_mutex		= z_loan_mut(client_data->condition_mutex);
  cond.msg_queue		= &client_data->reply_queue;
  cond.wait_set_data_ptr	= &client_data->wait_set_data;

  zenoh_pico_condition_trigger(&cond);
}

bool client_condition_check_and_attach(ZenohPicoClientData *client_data,
				       ZenohPicoWaitSetData *wait_set_data)
{
  ZenohPicoWaitCondition cond;
  cond.condition_mutex		= z_loan_mut(client_data->condition_mutex);
  cond.msg_queue		= &client_data->reply_queue;
  cond.wait_set_data_ptr	= &client_data->wait_set_data;

  return zenoh_pico_condition_check_and_attach(&cond, wait_set_data);
}

bool client_condition_detach_and_queue_is_empty(ZenohPicoClientData *client_data)
{
  ZenohPicoWaitCondition cond;
  cond.condition_mutex		= z_loan_mut(client_data->condition_mutex);
  cond.msg_queue		= &client_data->reply_queue;
  cond.wait_set_data_ptr	= &client_data->wait_set_data;

  return zenoh_pico_condition_detach_and_queue_is_empty(&cond);
}

rmw_client_t *
rmw_create_client(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile)
{
  RMW_ZENOH_FUNC_ENTRY(service_name);

  RMW_CHECK_ARGUMENT_FOR_NULL(node, NULL);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node->implementation_identifier,
    return NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, NULL);
  if (strlen(service_name) == 0) {
    RMW_SET_ERROR_MSG("service name is empty string");
    return NULL;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, NULL);

  if (!qos_profile->avoid_ros_namespace_conventions) {
    if(!rmw_zenoh_pico_check_validate_name(service_name))
      return NULL;
  }
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context,
    "expected initialized context",
    return NULL);

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

  // Get the service type support.
  const rosidl_service_type_support_t * type_support = find_service_type_support(type_supports);
  if (type_support == NULL) {
    // error was already set by find_service_type_support
    return NULL;
  }

  ZenohPicoClientData * client_data = zenoh_pico_generate_client_data(
    node_data,
    service_name,
    type_support,
    qos_profile);

  if(client_data == NULL)
    goto error;

  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    zenoh_pico_debug_client_data(client_data);
  }

  rmw_client_t *rmw_client = Z_MALLOC(sizeof(rmw_client_t));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_client,
    "failed to allocate memory for the client",
    goto error);
  memset((void *)rmw_client, 0, sizeof(rmw_client_t));

  char *_service_name = Z_MALLOC(strlen(service_name) +1);
  RMW_CHECK_FOR_NULL_WITH_MSG(_service_name,
			      "failed to allocate service name",
			      goto error);
  memset(_service_name, 0, strlen(service_name) +1);
  strcpy(_service_name, service_name);

  rmw_client->implementation_identifier = rmw_get_implementation_identifier();
  rmw_client->service_name		= _service_name;
  rmw_client->data			= client_data;

  if(!declaration_client_data(client_data))
    goto error;

  return rmw_client;

  error:
  if(client_data != NULL)
    zenoh_pico_destroy_client_data(client_data);

  return NULL;
}

rmw_ret_t
rmw_destroy_client(
  rmw_node_t * node,
  rmw_client_t * client)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  ZenohPicoClientData *clinet_data = (ZenohPicoClientData *)client->data;

  if(client_data != NULL){
    undeclaration_service_data(client_data);
    zenoh_pico_destroy_service_data(client_data);
    client->data = NULL;
  }

  Z_FREE(client->service_name);
  Z_FREE(client);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_client_request_publisher_get_actual_qos(
  const rmw_client_t * client,
  rmw_qos_profile_t * qos)
{
  RMW_ZENOH_FUNC_ENTRY(client);

  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);
  ZenohPicoClientData * client_data = (ZenohPicoClientData * )client->data;
  RMW_CHECK_ARGUMENT_FOR_NULL(client_data, RMW_RET_INVALID_ARGUMENT);

  *qos = client_data->qos_profile;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_client_response_subscription_get_actual_qos(
  const rmw_client_t * client,
  rmw_qos_profile_t * qos)
{
  RMW_ZENOH_FUNC_ENTRY(client);

  // The same QoS profile is used for sending requests and receiving responses.
  return rmw_client_request_publisher_get_actual_qos(client, qos);
}

rmw_ret_t
rmw_get_gid_for_client(
  const rmw_client_t * client,
  rmw_gid_t * gid)
{
  RMW_ZENOH_FUNC_ENTRY(client);

  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  return RMW_RET_ERROR;
}

static void add_new_replay_message(ZenohPicoClientData *client_data, ReceiveMessageData *recv_data)
{
  (void)recv_msg_list_push(&client_data->reply_queue, recv_data);

  (void)data_callback_trigger(&client_data->data_callback_mgr);

  (void)client_condition_trigger(client_data);
}

static void _reply_handler(z_loaned_reply_t *reply, void *ctx) {
  if (z_reply_is_ok(reply)) {
    const z_loaned_sample_t *sample = z_reply_ok(reply);
    if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
      RMW_ZENOH_LOG_INFO(">> Received");
    }

    ZenohPicoClientData *client_data = (ZenohPicoClientData *)ctx;
    if (client_data == NULL) {
      z_view_string_t keystr;
      z_keyexpr_as_view_string(z_sample_keyexpr(sample), &keystr);

      RMW_ZENOH_LOG_INFO("keystr is %s ", z_string_data(z_loan(keystr)));
      RMW_ZENOH_LOG_ERROR("Unable to obtain rmw_subscription_data_t from data for "
			  "subscription for %s",
			  z_string_data(z_loan(keystr)));
      return;
    }

    ReceiveMessageData * recv_data;
    if((recv_data = rmw_zenoh_pico_generate_recv_msg_data(sample, zenoh_pico_gen_timestamp())) == NULL) {
      RMW_ZENOH_LOG_ERROR("unable to generate_recv_msg_data");
      return;
    }

    add_new_replay_message(client_data, recv_data);
  }
}

static void _reply_dropper(void *ctx) {
  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    RMW_ZENOH_LOG_INFO(">> Received query final notification");
  }
}

rmw_ret_t
rmw_send_request(
  const rmw_client_t * client,
  const void * ros_request,
  int64_t * sequence_id)
{
  RMW_ZENOH_FUNC_ENTRY(client);

  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    client->service_name, "client has no service name", RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  ZenohPicoClientData * client_data = (ZenohPicoClientData * )client->data;
  RMW_CHECK_FOR_NULL_WITH_MSG(
    client_data,
    "Unable to retrieve client_data from client.",
    RMW_RET_INVALID_ARGUMENT);

  size_t data_length;
  uint8_t * msg_bytes = rmw_zenoh_pico_serialize(client_data->request_callback,
						 ros_request,
						 &data_length);
  if(msg_bytes == NULL){
    return RMW_RET_ERROR;
  }

  ZenohPicoSession *session = client_data->node->session;

  z_mutex_lock(z_loan_mut(client_data->mutex));

  // set attachment value
  z_get_options_t options;
  z_get_options_default(&options);
  *sequence_id = attachment_sequence_num_inc(&client_data->attachment);

  z_owned_bytes_t attachment;
  if(_Z_IS_ERR(attachment_gen(&client_data->attachment, &attachment))){
    z_mutex_unlock(z_loan_mut(client_data->mutex));
    return RMW_RET_ERROR;
  }
  options.attachment = z_move(attachment);

  // set option paramater
  options.timeout_ms = ULLONG_MAX;
  options.consolidation = z_query_consolidation_none();

  z_owned_bytes_t payload;
  z_bytes_copy_from_buf(&payload, msg_bytes, data_length);
  // z_bytes_from_static_buf(&payload, msg_bytes, data_length);
  options.payload = z_bytes_move(&payload);

  // send client request and wait until responce from service on closure (callback)
  z_owned_closure_reply_t callback;
  z_closure(&callback, _reply_handler, _reply_dropper, client_data);

  z_view_keyexpr_t ke;
  const z_loaned_string_t *keyexpr = z_loan(client_data->topic_key);
  z_view_keyexpr_from_substr(&ke, z_string_data(keyexpr), z_string_len(keyexpr));
  z_result_t ret = z_get(z_loan(session->session),
			 z_loan(ke),
			 "",
			 z_move(callback),
			 &options);

  z_mutex_unlock(z_loan_mut(client_data->mutex));

  TOPIC_FREE(msg_bytes);

  return ret == _Z_RES_OK ?  RMW_RET_OK : RMW_RET_ERROR;
}

rmw_ret_t
rmw_take_response(
  const rmw_client_t * client,
  rmw_service_info_t * request_header,
  void * ros_response,
  bool * taken)
{
  RMW_ZENOH_FUNC_ENTRY(client);

  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client->data, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RMW_CHECK_FOR_NULL_WITH_MSG(
    client->service_name, "client has no service name", RMW_RET_INVALID_ARGUMENT);

  ZenohPicoClientData * client_data = (ZenohPicoClientData * )client->data;
  RMW_CHECK_FOR_NULL_WITH_MSG(
    client_data,
    "Unable to retrieve client_data from client.",
    RMW_RET_INVALID_ARGUMENT);

  ReceiveMessageData *msg_data = recv_msg_list_pop(&client_data->reply_queue);
  RMW_CHECK_ARGUMENT_FOR_NULL(msg_data, RMW_RET_ERROR);

  bool deserialize_rv = rmw_zenoh_pico_deserialize_msg(msg_data,
						       client_data->response_callback,
						       ros_response,
						       NULL);
  if (taken != NULL) {
    *taken = deserialize_rv;
  }

  if (!deserialize_rv) {
    RMW_SET_ERROR_MSG("Typesupport deserialize error.");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t rmw_service_server_is_available(
  const rmw_node_t * node,
  const rmw_client_t * client,
  bool * is_available)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  (void)node;
  (void)client;
  (void)is_available;
  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}
