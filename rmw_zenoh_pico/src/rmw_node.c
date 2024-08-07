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


#include <rmw/allocators.h>
#include <rmw/rmw.h>

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

  rmw_node_t _rmw_node;
  memset(&_rmw_node, 0, sizeof(rmw_node_t));

  _rmw_node.implementation_identifier = rmw_get_implementation_identifier();

  while (1) {
    sleep(1);
  }

  rmw_node_t *rmw_node_handle = z_malloc(sizeof(rmw_node_t));
  memcpy(rmw_node_handle, &_rmw_node, sizeof(rmw_node_t));

  return rmw_node_handle;
}

rmw_ret_t rmw_destroy_node(
  rmw_node_t * node)
{
  _Z_DEBUG("%s : start", __func__);

  rmw_ret_t ret = RMW_RET_OK;
  if (!node) {
    RMW_UROS_TRACE_MESSAGE("node handle is null")
      return RMW_RET_ERROR;
  }

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(node->implementation_identifier, RMW_RET_ERROR);

  if (!node->data) {
    RMW_UROS_TRACE_MESSAGE("node impl is null")
      return RMW_RET_ERROR;
  }

  return ret;
}

rmw_ret_t
rmw_node_assert_liveliness(
  const rmw_node_t * node)
{
  _Z_DEBUG("%s : start", __func__);

  (void)node;
  RMW_UROS_TRACE_MESSAGE("function not implemented")
    return RMW_RET_UNSUPPORTED;
}

const rmw_guard_condition_t *
rmw_node_get_graph_guard_condition(
  const rmw_node_t * node)
{
  _Z_DEBUG("%s : start", __func__);

  return NULL;
}
