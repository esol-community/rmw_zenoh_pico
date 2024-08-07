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

#include <rmw_zenoh_pico/config.h>
#include <rmw_zenoh_pico/rmw_c_macros.h>
#include <rmw/allocators.h>
#include <rmw/rmw.h>

#include "./rmw_microros_internal/error_handling_internal.h"

rmw_node_t *
rmw_create_node(
  rmw_context_t * context,
  const char * name,
  const char * namespace_)
{
  (void)context;
  printf("rmw_create_node() : check 1\n");

  rmw_node_t * rmw_node = NULL;
  if (!name || strlen(name) == 0) {
    RMW_UROS_TRACE_MESSAGE("name is null");
  } else if (!namespace_ || strlen(namespace_) == 0) {
    RMW_UROS_TRACE_MESSAGE("namespace is null");
  } else {
  }
  return rmw_node;
}

rmw_ret_t rmw_destroy_node(
  rmw_node_t * node)
{
  printf("rmw_destroy_node() : check 1\n");

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
  (void)node;
  RMW_UROS_TRACE_MESSAGE("function not implemented")
  return RMW_RET_UNSUPPORTED;
}

const rmw_guard_condition_t *
rmw_node_get_graph_guard_condition(
  const rmw_node_t * node)
{
  return NULL;
}
