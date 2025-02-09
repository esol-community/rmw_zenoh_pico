// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
#include "zenoh-pico/api/primitives.h"
#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

z_owned_mutex_t mutex_ZenohPicoNodeData;

ZenohPicoNodeData * zenoh_pico_generate_node_data(size_t node_id,
						  ZenohPicoSession *session,
						  ZenohPicoEntity *entity)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  if((session == NULL) || (entity == NULL))
    return NULL;

  ZenohPicoNodeData *node_data = NULL;
  ZenohPicoGenerateData(node_data, ZenohPicoNodeData);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node_data,
    "failed to allocate struct for the ZenohPicoNodeData",
    return NULL);

  node_data->session	= session;
  node_data->entity	= entity;
  node_data->id	        = node_id;

  // generate key from entity data
  z_string_empty(&node_data->token_key);
  if(_Z_IS_ERR(generate_liveliness(entity, &node_data->token_key))){
    RMW_SET_ERROR_MSG("failed generate_liveliness()");
    return NULL;
  }

  return node_data;
}

ZenohPicoNodeData *zenoh_pico_loan_node_data(ZenohPicoNodeData *node_data)
{
  ZenohPicoLoanData(node_data, ZenohPicoNodeData);

  return node_data;
}

bool zenoh_pico_destroy_node_data(ZenohPicoNodeData *node_data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  RMW_CHECK_ARGUMENT_FOR_NULL(node_data, false);

  z_liveliness_undeclare_token(z_move(node_data->token));

  z_drop(z_move(node_data->token_key));

  // delete entity
  if(node_data->entity != NULL){
    zenoh_pico_destroy_entity(node_data->entity);
    node_data->entity = NULL;
  }

  ZenohPicoDestroyData(node_data, ZenohPicoNodeData);

  return true;
}

void zenoh_pico_debug_node_data(ZenohPicoNodeData *node_data)
{
  printf("--------- node data ----------\n");
  printf("ref = %d\n", node_data->ref);

  Z_STRING_PRINTF(node_data->token_key, token_key);

  // debug entity member
  zenoh_pico_debug_entity(node_data->entity);
}

bool declaration_node_data(ZenohPicoNodeData *node_data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  ZenohPicoSession *session = node_data->session;

  z_view_keyexpr_t ke;
  const z_loaned_string_t *keyexpr = z_loan(node_data->token_key);
  z_view_keyexpr_from_substr(&ke, z_string_data(keyexpr), z_string_len(keyexpr));
  if(_Z_IS_ERR(z_liveliness_declare_token(z_loan(session->session),
					  &node_data->token,
					  z_loan(ke),
					  NULL))){
    return false;
  }

  // dump node infomation
  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    zenoh_pico_debug_node_data(node_data);
  }

  return true;
}

static rmw_node_t *rmw_node_generate(rmw_context_t *context, ZenohPicoNodeData *node_data)
{
  RMW_ZENOH_FUNC_ENTRY(context);

  if(node_data->entity->node_info == NULL)
    return NULL;

  ZenohPicoNodeInfo *node_info = node_data->entity->node_info;

  rmw_node_t * node = Z_MALLOC(sizeof(rmw_node_t));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node,
    "failed to allocate memory for the node",
    return NULL);

  memset(node, 0, sizeof(rmw_node_t));

  const z_loaned_string_t *_name = z_loan(node_info->name);
  const z_loaned_string_t *_ns   = z_loan(node_info->ns);

  node->name				= zenoh_pico_string_clone(_name);
  node->namespace_			= zenoh_pico_string_clone(_ns);
  node->data				= (void *)node_data;
  node->implementation_identifier	= rmw_get_implementation_identifier();
  node->context				= context;

  return node;
}

static rmw_ret_t rmw_node_destroy(rmw_node_t * node)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  if(node != NULL){
    ZenohPicoNodeData *node_data = (ZenohPicoNodeData *)node->data;
    if(node_data != NULL){
      zenoh_pico_destroy_node_data(node_data);
      node_data = NULL;
    }

    Z_FREE(node->name);
    Z_FREE(node->namespace_);
    Z_FREE(node);
  }

  return RMW_RET_OK;
}

rmw_node_t *
rmw_create_node(rmw_context_t * context, const char * name, const char * namespace)
{
  RMW_ZENOH_FUNC_ENTRY(context);

  RMW_ZENOH_LOG_INFO("name = [%s], namespace = [%s]", name, namespace);

  RMW_CHECK_ARGUMENT_FOR_NULL(context, NULL);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context->implementation_identifier,
    return NULL);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(name, NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(namespace, NULL);

  if (strlen(name) == 0 || strlen(namespace) == 0) {
    RMW_ZENOH_LOG_INFO("name or namespace is null");
    return NULL;
  }

  ZenohPicoSession *session = (ZenohPicoSession *)context->impl;

  // open zenoh session
  if(session_connect(session) != RMW_RET_OK){
    RMW_ZENOH_LOG_ERROR("zenoh session open error...");
    return NULL;
  }

  ZenohPicoNodeInfo *_node_info = NULL;
  ZenohPicoEntity *_entity = NULL;
  ZenohPicoNodeData *_node_data = NULL;
  size_t _entity_id = 0;

  {
    // generate private node info data
    _node_info = zenoh_pico_generate_node_info(context->actual_domain_id,
					       namespace,
					       name,
					       z_loan(session->enclave));
    if(_node_info == NULL){
      goto error;
    }
  }

  {
    // generate entity data
    _entity_id = zenoh_pico_get_next_entity_id();
    z_id_t _zid = z_info_zid(z_loan(session->session));
    _entity = zenoh_pico_generate_entity(&_zid,
					 _entity_id,
					 _entity_id,
					 Node,
					 _node_info,
					 NULL);
    if(_entity == NULL){
      goto error;
    }
  }

  {
    // generate private node data
    _node_data = zenoh_pico_generate_node_data(_entity_id,
					       session,
					       _entity);
    if(_node_data == NULL){
      goto error;
    }
  }

  // generate rmw_node_handle data
  rmw_node_t * node = rmw_node_generate(context, _node_data);
  if(node == NULL)
    goto error;

  declaration_node_data(_node_data);

  return node;

  error:
  if(_node_info != NULL)
    zenoh_pico_destroy_node_info(_node_info);

  if(_entity != NULL)
    zenoh_pico_destroy_entity(_entity);

  if(_node_data != NULL)
    zenoh_pico_destroy_node_data(_node_data);

  return NULL;
}

rmw_ret_t rmw_destroy_node(rmw_node_t * node)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);

  if (!node) {
    RMW_ZENOH_LOG_INFO("node handle is null");
    return RMW_RET_ERROR;
  }

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  if (node->data == NULL) {
    RMW_ZENOH_LOG_INFO("node impl is null");
    return RMW_RET_ERROR;
  }

  return rmw_node_destroy(node);
}

rmw_ret_t
rmw_node_assert_liveliness(const rmw_node_t * node)
{
  RMW_ZENOH_FUNC_ENTRY(node);
  RMW_ZENOH_LOG_INFO("start(%p)", node);

  (void)node;
  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

const rmw_guard_condition_t *
rmw_node_get_graph_guard_condition(const rmw_node_t * node)
{
  RMW_ZENOH_FUNC_ENTRY(node);
  RMW_ZENOH_LOG_INFO("start(%p)", node);

  ZenohPicoNodeData *node_data = (ZenohPicoNodeData *)node->data;
  ZenohPicoSession *session = node_data->session;
  rmw_guard_condition_t * graph_guard_condition = &session->graph_guard_condition;

  return graph_guard_condition;
}
