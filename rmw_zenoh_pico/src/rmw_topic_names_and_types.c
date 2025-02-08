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
#include <rmw/names_and_types.h>

#include <rmw_zenoh_pico/config.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

#ifdef RMW_UXRCE_GRAPH
#include <rmw_zenoh_pico/rmw_graph.h>
#endif  // RMW_UXRCE_GRAPH

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
    "Function not available: enable RMW_UXRCE_GRAPH configuration profile before using");
  return RMW_RET_UNSUPPORTED;
}
