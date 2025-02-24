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

void client_condition_trigger(ZenohPicoServiceData *data)
{
  ZenohPicoWaitCondition cond;
  cond.condition_mutex		= z_loan_mut(data->condition_mutex);
  cond.msg_queue		= &data->response_queue;
  cond.wait_set_data_ptr	= &data->wait_set_data;

  zenoh_pico_condition_trigger(&cond);
}

bool client_condition_check_and_attach(ZenohPicoServiceData *data,
					ZenohPicoWaitSetData *wait_set_data)
{
  ZenohPicoWaitCondition cond;
  cond.condition_mutex		= z_loan_mut(data->condition_mutex);
  cond.msg_queue		= &data->response_queue;
  cond.wait_set_data_ptr	= &data->wait_set_data;

  return zenoh_pico_condition_check_and_attach(&cond, wait_set_data);
}

bool client_condition_detach_and_queue_is_empty(ZenohPicoServiceData *data)
{
  ZenohPicoWaitCondition cond;
  cond.condition_mutex		= z_loan_mut(data->condition_mutex);
  cond.msg_queue		= &data->response_queue;
  cond.wait_set_data_ptr	= &data->wait_set_data;

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

  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context,
    "expected initialized context",
    return NULL);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context->impl,
    "expected initialized context impl",
    return NULL);

  ZenohPicoNodeData *node_data = (ZenohPicoNodeData *)node->data;
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node_data, "NodeData not found.",
    return NULL);

  if(!isEnableSession(node_data->session)){
    RMW_SET_ERROR_MSG("zenoh session is invalid");
    return NULL;
  }

  // Get the service type support.
  const rosidl_service_type_support_t * type_support = find_service_type_support(type_supports);
  if (type_support == NULL) {
    // error was already set by find_service_type_support
    return NULL;
  }

  ZenohPicoServiceData * client_data = zenoh_pico_generate_service_data(
    node_data,
    service_name,
    type_support,
    qos_profile,
    Client);

  if(client_data == NULL)
    goto error;

  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    zenoh_pico_debug_service_data(client_data);
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

  if(!declaration_service_data(client_data))
    goto error;

  return rmw_client;

  error:
  if(client_data != NULL)
    zenoh_pico_destroy_service_data(client_data);

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

  ZenohPicoServiceData *client_data = (ZenohPicoServiceData *)client->data;

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
  ZenohPicoServiceData * client_data = (ZenohPicoServiceData * )client->data;
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

static void add_new_replay_message(ZenohPicoServiceData *client_data, ReceiveMessageData *recv_data)
{
  (void)recv_msg_list_append(&client_data->response_queue, recv_data);

  (void)data_callback_trigger(&client_data->data_callback_mgr);

  (void)client_condition_trigger(client_data);
}

static void _reply_handler(z_loaned_reply_t *reply, void *ctx) {
  if (z_reply_is_ok(reply)) {
    const z_loaned_sample_t *sample = z_reply_ok(reply);
    if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
      RMW_ZENOH_LOG_INFO(">> Received");
    }

    ZenohPicoServiceData *client_data = (ZenohPicoServiceData *)ctx;
    if (client_data == NULL) {
      z_view_string_t keystr;
      z_keyexpr_as_view_string(z_sample_keyexpr(sample), &keystr);

      RMW_ZENOH_LOG_INFO("keystr is %s ", z_string_data(z_loan(keystr)));
      RMW_ZENOH_LOG_ERROR("Unable to obtain ZenohPicoServiceData from data for "
			  "client for %s",
			  z_string_data(z_loan(keystr)));
      return;
    }

    ReceiveMessageData * recv_data;
    if((recv_data = rmw_zenoh_pico_generate_recv_sample_msg_data(sample, zenoh_pico_gen_timestamp())) == NULL) {
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

  ZenohPicoServiceData * client_data = (ZenohPicoServiceData * )client->data;
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

  TOPIC_FREE(msg_bytes);

  z_mutex_unlock(z_loan_mut(client_data->mutex));

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

  ZenohPicoServiceData * client_data = (ZenohPicoServiceData * )client->data;
  RMW_CHECK_FOR_NULL_WITH_MSG(
    client_data,
    "Unable to retrieve client_data from client.",
    RMW_RET_INVALID_ARGUMENT);

  ReceiveMessageData *msg_data = recv_msg_list_pop(&client_data->response_queue);
  RMW_CHECK_ARGUMENT_FOR_NULL(msg_data, RMW_RET_ERROR);

  *taken = false;
  if(msg_data != NULL){
    bool deserialize_rv = rmw_zenoh_pico_deserialize_response_msg(msg_data,
								  client_data->response_callback,
								  ros_response,
								  request_header);
    *taken = deserialize_rv;

    if (!deserialize_rv) {
      RMW_SET_ERROR_MSG("Typesupport deserialize error.");
      return RMW_RET_ERROR;
    }

    ZenohPicoDataDestroy(msg_data);
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
