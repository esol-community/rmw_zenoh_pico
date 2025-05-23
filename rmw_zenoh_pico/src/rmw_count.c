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

rmw_ret_t
rmw_count_publishers(
  const rmw_node_t * node,
  const char * topic_name,
  size_t * count)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  (void)node;
  (void)topic_name;
  (void)count;
  RMW_ZENOH_LOG_INFO(
    "Function not available");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_count_subscribers(
  const rmw_node_t * node,
  const char * topic_name,
  size_t * count)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  (void)node;
  (void)topic_name;
  (void)count;
  RMW_ZENOH_LOG_INFO(
    "Function not available");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_count_clients(
  const rmw_node_t * node,
  const char * service_name,
  size_t * count)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  (void)node;
  (void)service_name;
  (void)count;
  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_count_services(
  const rmw_node_t * node,
  const char * service_name,
  size_t * count)
{
  (void)node;
  (void)service_name;
  (void)count;
  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}
