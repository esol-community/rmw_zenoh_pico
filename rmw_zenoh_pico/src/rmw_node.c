// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_entity.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_session.h"
#include "zenoh-pico/api/primitives.h"
#include "zenoh-pico/collections/string.h"

#include <rmw/allocators.h>
#include <rmw/rmw.h>

#include <string.h>
#include <zenoh-pico.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

//-----------------------------

ZenohPicoNodeData * zenoh_pico_generate_node_data(size_t node_id,
						  ZenohPicoSession *session,
						  ZenohPicoEntity *entity)
{
  _Z_DEBUG("%s : start", __func__);

  if((session == NULL) || (entity == NULL))
    return NULL;

  ZenohPicoNodeData *node_data = NULL;
  ZenohPicoGenerateData(node_data, ZenohPicoNodeData);
  if(node_data == NULL)
    return NULL;

  node_data->session_	= session;
  node_data->entity_	= entity;
  node_data->id_	= node_id;

  // generate key from entity data
  node_data->token_key_ = generate_liveliness(entity);

  return node_data;
}

ZenohPicoNodeData *zenoh_pico_loan_node_data(ZenohPicoNodeData *node_data)
{
  // ATTENTION :
  // I have not implemented smart pointer, yet.

  return node_data;
}

bool zenoh_pico_destroy_node_data(ZenohPicoNodeData *node_data)
{
  _Z_DEBUG("%s : start", __func__);

  (void)undeclaration_node_data(node_data);

  Z_STRING_FREE(node_data->token_key_);

  if (z_check(node_data->token_)) {
      ZenohPicoSession *session = node_data->session_;
      z_undeclare_keyexpr(z_loan(session->session_), &node_data->token_);
  }

  // delete entitiy
  if(node_data->entity_ != NULL){
    zenoh_pico_destroy_entitiy(node_data->entity_);
    node_data->entity_ = NULL;
  }

  ZenohPicoDestroyData(node_data);

  return true;
}

void zenoh_pico_debug_node_data(ZenohPicoNodeData *node_data)
{
  printf("--------- node data ----------\n");
  printf("ref = %d\n", node_data->ref_);

  Z_STRING_PRINTF(node_data->token_key_, token_key);

  z_keyexpr_t *value = node_data->token_._value;
  if(value != NULL){
    printf("z_keyexpr id = %d\n", value->_id);
    printf("z_keyexpr mapping = %x\n", value->_mapping._val);
    printf("z_keyexpr suffix = %s\n", value->_suffix);
  } else {
    printf("value is zero\n");
  }

  // debug entity member
  zenoh_pico_debug_entitiy(node_data->entity_);
}

//-----------------------------

bool declaration_node_data(ZenohPicoNodeData *node_data)
{
  ZenohPicoSession *session = node_data->session_;
  const char *keyexpr = Z_STRING_VAL(node_data->token_key_);

  _Z_DEBUG("Declaring node key expression '%s'...", keyexpr);
  node_data->token_ = z_declare_keyexpr(z_loan(session->session_), z_keyexpr(keyexpr));
  if (!z_check(node_data->token_)) {
    return false;
  }

  zenoh_pico_debug_node_data(node_data);

  return true;
}

bool undeclaration_node_data(ZenohPicoNodeData *node_data)
{
  ZenohPicoSession *session = node_data->session_;

  if (z_check(node_data->token_)) {
    z_undeclare_keyexpr(z_loan(session->session_), &node_data->token_);
  }

  return true;
}

//-----------------------------

static rmw_node_t *rmw_node_generate(rmw_context_t *context, ZenohPicoNodeData *node_data)
{
  _Z_DEBUG("%s : start()", __func__);
  if(node_data->entity_->node_info_ == NULL)
    return NULL;

  ZenohPicoNodeInfo_t *node_info = node_data->entity_->node_info_;

  rmw_node_t * node = z_malloc(sizeof(rmw_node_t));
  if(node == NULL)
    return NULL;

  memset(node, 0, sizeof(rmw_node_t));

  node->name				= node_info->name_.val;
  node->namespace_			= node_info->ns_.val;
  node->data				= (void *)node_data;
  node->implementation_identifier	= rmw_get_implementation_identifier();
  node->context				= context;

  return node;
}

static rmw_ret_t rmw_node_destroy(rmw_node_t * node)
{
  _Z_DEBUG("%s : start()", __func__);

  if(node != NULL){
    ZenohPicoNodeData *node_data = (ZenohPicoNodeData *)node->data;
    if(node_data != NULL){
      zenoh_pico_destroy_node_data(node_data);
      node_data = NULL;
    }

    z_free(node);
  }

  return RMW_RET_OK;
}

rmw_node_t *
rmw_create_node(
  rmw_context_t * context,
  const char * name,
  const char * namespace_)
{
  _Z_DEBUG("%s : start(name = [%s], namespace = [%s])", __func__, name, namespace_);

  RMW_CHECK_ARGUMENT_FOR_NULL(context, NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(name, NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(namespace_, NULL);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context->implementation_identifier,
    NULL);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return NULL);

  if (strlen(name) == 0 || strlen(namespace_) == 0) {
    RMW_UROS_TRACE_MESSAGE("name or namespace_ is null");
    return NULL;
  }

  ZenohPicoSession *session = (ZenohPicoSession *)context->impl;

  // generate private node info data
  ZenohPicoNodeInfo_t *node_info;
  z_string_t _domain = conv_domain(context->actual_domain_id);
  z_string_t _ns = _z_string_make(namespace_);
  z_string_t _name = _z_string_make(name);
  z_string_t _enclave = _z_string_make(session->enclave_.val);

  node_info = zenoh_pico_generate_node_info(&_domain,
					    &_ns,
					    &_name,
					    &_enclave);
  if(node_info == NULL){
    return NULL;
  }

  // generate entity data
  size_t _entity_id = zenoh_pico_get_next_entity_id();
  ZenohPicoEntity *entity = zenoh_pico_generate_entitiy(NULL,
							z_info_zid(z_loan(session->session_)),
							_entity_id,
							_entity_id,
							Node,
							node_info,
							NULL);
  if(entity == NULL){
    zenoh_pico_destroy_node_info(node_info);
    return NULL;
  }

  // generate private node data
  ZenohPicoNodeData *node_data = zenoh_pico_generate_node_data(_entity_id,
							       session,
							       entity);
  if(node_data == NULL){
    zenoh_pico_destroy_entitiy(entity);
    return NULL;
  }

  if(!declaration_node_data(node_data)){
    zenoh_pico_destroy_node_data(node_data);
    return NULL;
  }

  // generate rmw_node_handle data
  rmw_node_t * node = rmw_node_generate(context, node_data);
  if(node == NULL){
    zenoh_pico_destroy_node_data(node_data);
    return NULL;
  }

  return node;
}

rmw_ret_t rmw_destroy_node(
  rmw_node_t * node)
{
  _Z_DEBUG("%s : start(%p)", __func__, node);

  if (!node) {
    RMW_UROS_TRACE_MESSAGE("node handle is null");
    return RMW_RET_ERROR;
  }

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(node->implementation_identifier, RMW_RET_ERROR);

  if (node->data == NULL) {
    RMW_UROS_TRACE_MESSAGE("node impl is null");
    return RMW_RET_ERROR;
  }

  return rmw_node_destroy(node);
}

rmw_ret_t
rmw_node_assert_liveliness(
  const rmw_node_t * node)
{
  _Z_DEBUG("%s : start(%p)", __func__, node);

  (void)node;
  RMW_UROS_TRACE_MESSAGE("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

const rmw_guard_condition_t *
rmw_node_get_graph_guard_condition(
  const rmw_node_t * node)
{
  _Z_DEBUG("%s : start(%p)", __func__, node);
  ZenohPicoNodeData *node_data = (ZenohPicoNodeData *)node->data;
  ZenohPicoSession *session = node_data->session_;
  rmw_guard_condition_t * graph_guard_condition = &session->graph_guard_condition;

  return graph_guard_condition;
}
