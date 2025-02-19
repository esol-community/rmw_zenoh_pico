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

  // clone node_info
  node_info = zenoh_pico_clone_node_info(node->entity->node_info);

  // generate entity data
  ZenohPicoSession *session = node->session;
  z_id_t zid = z_info_zid(z_loan(session->session));

  if(service_type == Service){
    entity = zenoh_pico_generate_service_entity(&zid, node->id, node_info, topic_name, type_support, qos_profile);
  }else{
    entity = zenoh_pico_generate_client_entity(&zid, node->id, node_info, topic_name, type_support, qos_profile);
  }
  if(entity == NULL)
    goto error;

  ZenohPicoGenerateData(data, ZenohPicoServiceData);
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
  recv_msg_list_init(&data->service_queue);

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

  return NULL;
}

bool zenoh_pico_destroy_service_data(ZenohPicoServiceData *data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  RMW_CHECK_ARGUMENT_FOR_NULL(data, false);

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

  ZenohPicoDestroyData(data, ZenohPicoServiceData);

  return true;
}

void zenoh_pico_debug_service_data(ZenohPicoServiceData *data)
{
  printf("--------- client data ----------\n");
  printf("ref = %d\n", data->ref);

  Z_STRING_PRINTF(data->liveliness_key, liveliness_key);
  Z_STRING_PRINTF(data->topic_key, topic_key);

  printf("service_queue = %d\n", recv_msg_list_count(&data->service_queue));

  // debug attachment
  attachment_debug(&data->attachment);

  // debug entity member
  zenoh_pico_debug_entity(data->entity);

  return;
}

bool declaration_service_data(ZenohPicoServiceData *data)
{
  ZenohPicoSession *session = data->node->session;

  // liveliness token declare
  (void)declaration_liveliness(session, z_loan(data->liveliness_key), &data->liveliness);

  return true;
}

bool undeclaration_service_data(ZenohPicoServiceData *data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  z_liveliness_undeclare_token(z_move(data->liveliness));

  return true;
}

rmw_service_t *
rmw_create_service(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_support,
  const char * service_name,
  const rmw_qos_profile_t * qos_policies)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  rmw_service_t * rmw_service = NULL;
  if (!node) {
    RMW_ZENOH_LOG_INFO("node handle is null");
  } else if (!type_support) {
    RMW_ZENOH_LOG_INFO("type support is null");
  } else if (!service_name || strlen(service_name) == 0) {
    RMW_ZENOH_LOG_INFO("service name is null or empty string");
  } else if (!qos_policies) {
    RMW_ZENOH_LOG_INFO("qos_profile is null");
  } else {
  }
  return rmw_service;
}

rmw_ret_t
rmw_destroy_service(
  rmw_node_t * node,
  rmw_service_t * service)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  rmw_ret_t result_ret = RMW_RET_OK;
  if (!node) {
    RMW_ZENOH_LOG_INFO("node handle is null");
    result_ret = RMW_RET_ERROR;
    result_ret = RMW_RET_ERROR;
  } else if (!node->data) {
    RMW_ZENOH_LOG_INFO("node imp is null");
    result_ret = RMW_RET_ERROR;
  } else if (!service) {
    RMW_ZENOH_LOG_INFO("service handle is null");
    result_ret = RMW_RET_ERROR;
    result_ret = RMW_RET_ERROR;
  } else if (!service->data) {
    RMW_ZENOH_LOG_INFO("service imp is null");
    result_ret = RMW_RET_ERROR;
  } else {
  }

  return result_ret;
}

rmw_ret_t
rmw_service_response_publisher_get_actual_qos(
  const rmw_service_t * service,
  rmw_qos_profile_t * qos)
{
  RMW_ZENOH_FUNC_ENTRY(service);

  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_service_request_subscription_get_actual_qos(
  const rmw_service_t * service,
  rmw_qos_profile_t * qos)
{
  RMW_ZENOH_FUNC_ENTRY(service);

  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_send_response(
  const rmw_service_t * service,
  rmw_request_id_t * request_header,
  void * ros_response)
{
  RMW_ZENOH_FUNC_ENTRY(service);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_request(
  const rmw_service_t * service,
  rmw_service_info_t * request_header,
  void * ros_request,
  bool * taken)
{
  RMW_ZENOH_FUNC_ENTRY(service);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  if (taken != NULL) {
    *taken = false;
  }

  return RMW_RET_OK;
}
