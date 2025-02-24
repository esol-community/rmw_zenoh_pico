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

void service_condition_trigger(ZenohPicoServiceData *data)
{
  ZenohPicoWaitCondition cond;
  cond.condition_mutex		= z_loan_mut(data->condition_mutex);
  cond.msg_queue		= &data->request_queue;
  cond.wait_set_data_ptr	= &data->wait_set_data;

  zenoh_pico_condition_trigger(&cond);
}

bool service_condition_check_and_attach(ZenohPicoServiceData *data,
					ZenohPicoWaitSetData *wait_set_data)
{
  ZenohPicoWaitCondition cond;
  cond.condition_mutex		= z_loan_mut(data->condition_mutex);
  cond.msg_queue		= &data->request_queue;
  cond.wait_set_data_ptr	= &data->wait_set_data;

  return zenoh_pico_condition_check_and_attach(&cond, wait_set_data);
}

bool service_condition_detach_and_queue_is_empty(ZenohPicoServiceData *data)
{
  ZenohPicoWaitCondition cond;
  cond.condition_mutex		= z_loan_mut(data->condition_mutex);
  cond.msg_queue		= &data->request_queue;
  cond.wait_set_data_ptr	= &data->wait_set_data;

  return zenoh_pico_condition_detach_and_queue_is_empty(&cond);
}

ZenohPicoServiceData * zenoh_pico_generate_service_data(
  ZenohPicoNodeData *node,
  const char * topic_name,
  const rosidl_service_type_support_t *type_support,
  const rmw_qos_profile_t *qos_profile,
  ZenohPicoEntityType service_type)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  RMW_CHECK_ARGUMENT_FOR_NULL(node, NULL);

  ZenohPicoNodeInfo *node_info	= NULL;
  ZenohPicoEntity *entity	= NULL;
  ZenohPicoServiceData *data	= NULL;

  // generate entity data
  ZenohPicoSession *session = node->session;
  z_id_t zid = z_info_zid(z_loan(session->session));

  node_info = ZenohPicoDataRefClone(node->entity->node_info);
  if(service_type == Service){
    entity = zenoh_pico_generate_service_entity(&zid,
						node->id,
						node_info,
						topic_name,
						type_support,
						qos_profile);
  }else{
    entity = zenoh_pico_generate_client_entity(&zid,
					       node->id,
					       node_info,
					       topic_name,
					       type_support,
					       qos_profile);
  }
  if(entity == NULL)
    goto error;

  data = ZenohPicoDataGenerate(data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    data,
    "failed to allocate struct for the ZenohPicoServiceData",
    goto error);

  data->id			= entity->id;
  data->node			= node;
  data->entity			= entity;
  data->request_callback	= get_request_callback(type_support);
  data->response_callback	= get_response_callback(type_support);

  memcpy(&data->qos_profile, qos_profile, sizeof(rmw_qos_profile_t));

  // generate key from entity data
  if(_Z_IS_ERR(generate_liveliness(entity, &data->liveliness_key))){
    RMW_SET_ERROR_MSG("failed generate_liveliness()");
    goto error;
  }

  // generate topic key
  z_string_empty(&data->topic_key);
  ZenohPicoTopicInfo *topic_info = entity->topic_info;
  if(_Z_IS_ERR(ros_topic_name_to_zenoh_key(z_loan(node_info->domain),
					   z_loan(topic_info->name),
					   z_loan(topic_info->type),
					   z_loan(topic_info->hash),
					   &data->topic_key))){
    RMW_SET_ERROR_MSG("failed ros_topic_name_to_zenoh_key()");
    goto error;
  }

  // init reply message list
  recv_msg_list_init(&data->request_queue);
  recv_msg_list_init(&data->response_queue);

  // init data callback manager
  data_callback_init(&data->data_callback_mgr);

  // init rmw_wait condition
  z_mutex_init(&data->condition_mutex);
  data->wait_set_data = NULL;

  // generate gid
  uint8_t _gid[RMW_GID_STORAGE_SIZE];
  zenoh_pico_gen_gid(z_loan(data->topic_key), _gid);

  z_slice_copy_from_buf(&data->attachment.gid, _gid, sizeof(_gid));
  data->attachment.sequence_num = 0;

  return data;

  error:
  if(node_info != NULL)
    zenoh_pico_destroy_node_info(node_info);

  if(entity != NULL)
    zenoh_pico_destroy_entity(entity);

  if(data != NULL){
    ZenohPicoDataDestroy(data);
  }

  return NULL;
}

bool zenoh_pico_destroy_service_data(ZenohPicoServiceData *data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  RMW_CHECK_ARGUMENT_FOR_NULL(data, false);

  if(ZenohPicoDataRelease(data)){

    z_drop(z_move(data->liveliness_key));
    z_drop(z_move(data->liveliness));

    attachment_destroy(&data->attachment);
    z_drop(z_move(data->mutex));

    if(data->node != NULL){
      (void)zenoh_pico_destroy_node_data(data->node);
      data->node = NULL;
    }

    if(data->entity != NULL){
      (void)zenoh_pico_destroy_entity(data->entity);
      data->entity = NULL;
    }

    if(recv_msg_list_count(&data->request_queue) > 0){
      while(true){
	ReceiveMessageData * recv_data = recv_msg_list_pop(&data->request_queue);
	if(recv_data == NULL)
	  break;

	(void)zenoh_pico_delete_recv_msg_data(recv_data);
      }
    }

    if(recv_msg_list_count(&data->response_queue) > 0){
      while(true){
	ReceiveMessageData * recv_data = recv_msg_list_pop(&data->response_queue);
	if(recv_data == NULL)
	  break;

	(void)zenoh_pico_delete_recv_msg_data(recv_data);
      }
    }

    ZenohPicoDataDestroy(data);
  }

  return true;
}

void zenoh_pico_debug_service_data(ZenohPicoServiceData *data)
{
  printf("--------- client data ----------\n");
  printf("ref = %d\n", data->ref);

  Z_STRING_PRINTF(data->liveliness_key, liveliness_key);
  Z_STRING_PRINTF(data->topic_key, topic_key);

  printf("request_queue = %d\n", recv_msg_list_count(&data->request_queue));

  // debug attachment
  attachment_debug(&data->attachment);

  // debug entity member
  zenoh_pico_debug_entity(data->entity);

  return;
}

static void add_new_request_message(ZenohPicoServiceData *service_data, ReceiveMessageData *recv_data)
{
  (void)recv_msg_list_push(&service_data->request_queue, recv_data);

  (void)data_callback_trigger(&service_data->data_callback_mgr);

  (void)service_condition_trigger(service_data);
}

static void error_reply(ZenohPicoServiceData *service_data, const z_loaned_query_t *query)
{
  z_owned_bytes_t reply_payload;
  z_bytes_from_static_str(&reply_payload, "rmw_zenoh_pico error");
  z_query_reply_err(query, z_move(reply_payload), NULL);
  return;
}

static void request_handler(z_loaned_query_t *query, void *ctx)
{
  RMW_ZENOH_FUNC_ENTRY(ctx);

  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    RMW_ZENOH_LOG_INFO(">> [Queryable handler] Received Query");
  }

  ZenohPicoServiceData *service_data = (ZenohPicoServiceData *)ctx;
  if (service_data == NULL) {
    z_view_string_t keystr;
    z_keyexpr_as_view_string(z_query_keyexpr(query), &keystr);

    RMW_ZENOH_LOG_INFO("keystr is %s ", z_string_data(z_loan(keystr)));
    RMW_ZENOH_LOG_ERROR("Unable to obtain ZenohPicoServiceData from data for"
			"service for %s",
			z_string_data(z_loan(keystr)));
    error_reply(service_data, query);
    return;
  }

  ReceiveMessageData * recv_data;
  if((recv_data = rmw_zenoh_pico_generate_recv_query_msg_data(query, zenoh_pico_gen_timestamp())) == NULL) {
    RMW_ZENOH_LOG_ERROR("unable to generate_recv_msg_data");
    error_reply(service_data, query);
    return;
  }

  add_new_request_message(service_data, recv_data);
  return;
}

bool declaration_service_data(ZenohPicoServiceData *data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  ZenohPicoSession *session = data->node->session;

  // declare queryable
  z_view_keyexpr_t ke;
  const z_loaned_string_t *keyexpr = z_loan(data->topic_key);
  z_view_keyexpr_from_substr(&ke, z_string_data(keyexpr), z_string_len(keyexpr));

  z_queryable_options_t options;
  options.complete = true;

  z_owned_closure_query_t callback;
  z_closure(&callback, request_handler, NULL, data);
  if (_Z_IS_ERR(z_declare_queryable(z_loan(session->session),
				    &data->qable,
				    z_loan(ke),
				    z_move(callback),
				    &options))) {
    RMW_SET_ERROR_MSG("Unable to create queryable");
    return false;
  }

  return declaration_liveliness(session, z_loan(data->liveliness_key), &data->liveliness);
}

bool undeclaration_service_data(ZenohPicoServiceData *data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  z_drop(z_move(data->qable));

  return z_liveliness_undeclare_token(z_move(data->liveliness));
}

rmw_service_t *
rmw_create_service(
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

  const rosidl_service_type_support_t * type_support = find_service_type_support(type_supports);
  if (type_support == NULL) {
    // error was already set by find_service_type_support
    return NULL;
  }

  ZenohPicoServiceData * service_data = zenoh_pico_generate_service_data(
    node_data,
    service_name,
    type_support,
    qos_profile,
    Service);
  if(service_data == NULL)
    goto error;

  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    zenoh_pico_debug_service_data(service_data);
  }

  rmw_service_t * rmw_service = Z_MALLOC(sizeof(rmw_service_t));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_service,
    "failed to allocate memory for the service",
    goto error);
  memset((void *)rmw_service, 0, sizeof(rmw_service_t));

  char *_service_name = Z_MALLOC(strlen(service_name) +1);
  RMW_CHECK_FOR_NULL_WITH_MSG(_service_name,
			      "failed to allocate service name",
			      goto error);
  memset(_service_name, 0, strlen(service_name) +1);
  strcpy(_service_name, service_name);

  rmw_service->implementation_identifier = rmw_get_implementation_identifier();
  rmw_service->service_name		 = _service_name;
  rmw_service->data			 = service_data;

  if(!declaration_service_data(service_data))
    goto error;

  return rmw_service;

  error:
  if(service_data != NULL)
    zenoh_pico_destroy_service_data(service_data);

  return NULL;
}

rmw_ret_t
rmw_destroy_service(
  rmw_node_t * node,
  rmw_service_t * service)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  ZenohPicoServiceData *service_data = (ZenohPicoServiceData *)service->data;

  if(service_data != NULL){
    undeclaration_service_data(service_data);
    zenoh_pico_destroy_service_data(service_data);
    service->data = NULL;
  }

  Z_FREE(service->service_name);
  Z_FREE(service);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_service_request_subscription_get_actual_qos(
  const rmw_service_t * service,
  rmw_qos_profile_t * qos)
{
  RMW_ZENOH_FUNC_ENTRY(service);


  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);
  ZenohPicoServiceData * service_data = (ZenohPicoServiceData * )service->data;
  RMW_CHECK_ARGUMENT_FOR_NULL(service_data, RMW_RET_INVALID_ARGUMENT);

  *qos = service_data->qos_profile;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_service_response_publisher_get_actual_qos(
  const rmw_service_t * service,
  rmw_qos_profile_t * qos)
{
  RMW_ZENOH_FUNC_ENTRY(service);

  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  // The same QoS profile is used for sending requests and receiving responses.
  return rmw_service_request_subscription_get_actual_qos(service, qos);
}

rmw_ret_t
rmw_take_request(
  const rmw_service_t * service,
  rmw_service_info_t * request_header,
  void * ros_request,
  bool * taken)
{
  RMW_ZENOH_FUNC_ENTRY(service);

  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service->service_name, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(service->data, RMW_RET_ERROR);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RMW_CHECK_FOR_NULL_WITH_MSG(
    service->service_name, "service has no service name", RMW_RET_INVALID_ARGUMENT);

  ZenohPicoServiceData * service_data = (ZenohPicoServiceData * )service->data;
  RMW_CHECK_FOR_NULL_WITH_MSG(
    service_data,
    "Unable to retrieve service_data from service.",
    RMW_RET_INVALID_ARGUMENT);

  ReceiveMessageData *msg_data = recv_msg_list_pop(&service_data->request_queue);
  RMW_CHECK_ARGUMENT_FOR_NULL(msg_data, RMW_RET_ERROR);

  bool deserialize_rv = rmw_zenoh_pico_deserialize_request_msg(msg_data,
							       service_data->request_callback,
							       ros_request,
							       request_header);
  if (taken != NULL) {
    *taken = deserialize_rv;
  }

  (void)recv_msg_list_push(&service_data->response_queue, msg_data);

  return RMW_RET_OK;
}

static bool _compare_responce_msg(ReceiveMessageData *msg, const void *data)
{
  const rmw_request_id_t *service = (const rmw_request_id_t *)data;

  const uint8_t *gid_ptr = z_slice_data(z_loan(msg->attachment.gid));
  if(memcmp(gid_ptr, service->writer_guid, sizeof(service->writer_guid)) != 0)
    return false;

  if(msg->attachment.sequence_num != service->sequence_number)
    return false;

  return true;
}

rmw_ret_t
rmw_send_response(
  const rmw_service_t * service,
  rmw_request_id_t * request_header,
  void * ros_response)
{
  RMW_ZENOH_FUNC_ENTRY(service);

  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(request_header, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_response, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RMW_CHECK_FOR_NULL_WITH_MSG(
    service->service_name, "service has no service name", RMW_RET_INVALID_ARGUMENT);

  ZenohPicoServiceData * service_data = (ZenohPicoServiceData * )service->data;
  RMW_CHECK_FOR_NULL_WITH_MSG(
    service_data,
    "Unable to retrieve service_data from service.",
    RMW_RET_INVALID_ARGUMENT);

  ReceiveMessageData *msg_data = recv_msg_list_pickup(&service_data->response_queue,
						      _compare_responce_msg,
						      (const void *)request_header);

  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    rmw_zenoh_pico_debug_recv_msg_data(msg_data);
  }

  size_t data_length;
  uint8_t * msg_bytes = rmw_zenoh_pico_serialize(service_data->response_callback,
						 ros_response,
						 &data_length);
  if(msg_bytes == NULL){
    (void)zenoh_pico_delete_recv_msg_data(msg_data);
    return RMW_RET_ERROR;
  }

  z_mutex_lock(z_loan_mut(service_data->mutex));

  // set attachment value
  z_query_reply_options_t options;
  z_query_reply_options_default(&options);

  attachment_sequence_num_inc(&service_data->attachment);
  z_owned_bytes_t reply_attachment;
  if(_Z_IS_ERR(attachment_gen(&service_data->attachment, &reply_attachment))){
    z_mutex_unlock(z_loan_mut(service_data->mutex));
    return RMW_RET_ERROR;
  }
  options.attachment = z_move(reply_attachment);

  // set reply payload
  z_owned_bytes_t reply_payload;
  z_bytes_copy_from_buf(&reply_payload, msg_bytes, data_length);

  // send response
  const z_loaned_query_t *query = z_query_loan(&msg_data->query);
  z_result_t ret = z_query_reply(query,
				 z_query_keyexpr(query),
				 z_move(reply_payload),
				 &options);

  (void)zenoh_pico_delete_recv_msg_data(msg_data);

  TOPIC_FREE(msg_bytes);

  z_mutex_unlock(z_loan_mut(service_data->mutex));

  return ret == _Z_RES_OK ?  RMW_RET_OK : RMW_RET_ERROR;
}
