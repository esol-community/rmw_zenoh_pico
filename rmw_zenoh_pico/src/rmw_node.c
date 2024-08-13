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
#include "zenoh-pico/api/primitives.h"

#include <rmw/allocators.h>
#include <rmw/rmw.h>

#include <zenoh-pico.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

rmw_node_t *
rmw_create_node(
  rmw_context_t * context,
  const char * name,
  const char * namespace_)
{
  _Z_DEBUG("%s : start(name = [%s], namespace = [%s])", __func__, name, namespace_);

  (void)context;

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
  node_info = zenoh_pico_generate_node_info(NULL,
					    context->actual_domain_id,
					    namespace_,
					    name,
					    session->enclave_.val);
  if(node_info == NULL){
    return NULL;
  }

  // generate entity data
  // ATTENTION:
  // this ownership of node_info move to new entity.
  // when this node_info is destroy, this entity is destroy.
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
  // ATTENTION:
  // this ownership of entity move to new node_data.
  // when this node_data is destroy, this entity is destroy.
  ZenohPicoNodeData *node_data = zenoh_pico_generate_node_data(NULL, session, entity);
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
