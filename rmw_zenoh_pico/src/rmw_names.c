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

#include <rmw/rmw.h>
#include <rmw/sanity_checks.h>

#include <rcutils/types/string_array.h>

#include <rmw_zenoh_pico/config.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico.h>


rmw_ret_t
rmw_get_publisher_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  (void)node;
  (void)allocator;
  (void)node_name;
  (void)node_namespace;
  (void)demangle;
  (void)topic_names_and_types;
  RMW_ZENOH_LOG_INFO(
    "Function not available");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_get_subscriber_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  (void)node;
  (void)allocator;
  (void)node_name;
  (void)node_namespace;
  (void)demangle;
  (void)topic_names_and_types;
  RMW_ZENOH_LOG_INFO(
    "Function not available");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_get_service_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * service_names_and_types)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  (void)node;
  (void)allocator;
  (void)node_name;
  (void)node_namespace;
  (void)service_names_and_types;
  RMW_ZENOH_LOG_INFO(
    "Function not available");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_get_client_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * service_names_and_types)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  (void)node;
  (void)allocator;
  (void)node_name;
  (void)node_namespace;
  (void)service_names_and_types;
  RMW_ZENOH_LOG_INFO(
    "Function not available");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_get_service_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * service_names_and_types)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  (void)node;
  (void)allocator;
  (void)service_names_and_types;
  RMW_ZENOH_LOG_INFO(
    "Function not available");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_get_node_names(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  (void)node;
  (void)node_names;
  (void)node_namespaces;
  RMW_ZENOH_LOG_INFO(
    "Function not available");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_get_node_names_with_enclaves(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces,
  rcutils_string_array_t * enclaves)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  (void)enclaves;   // TODO(jamoralp): what is this used for?
  return rmw_get_node_names(node, node_names, node_namespaces);
}

rmw_ret_t
rmw_get_topic_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  (void)node;
  (void)allocator;
  (void)no_demangle;
  (void)topic_names_and_types;
  RMW_ZENOH_LOG_INFO(
    "Function not available");
  return RMW_RET_UNSUPPORTED;
}
