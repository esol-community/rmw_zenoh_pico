// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
#include <rmw/ret_types.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

rmw_ret_t
rmw_get_gid_for_client(
  const rmw_client_t * client,
  rmw_gid_t * gid)
{
  RMW_ZENOH_FUNC_ENTRY(client);

  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  return RMW_RET_ERROR;
}
