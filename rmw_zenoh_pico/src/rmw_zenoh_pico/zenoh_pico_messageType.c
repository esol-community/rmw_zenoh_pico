/*
 * Copyright (C)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

const rosidl_message_type_support_t * find_message_type_support(
  const rosidl_message_type_support_t * type_supports)
{
  const rosidl_message_type_support_t * type_support = get_message_typesupport_handle(
    type_supports, RMW_ZENOH_PICO_TYPESUPPORT_C);
  if (!type_support) {
    rcutils_error_string_t error_string = rcutils_get_error_string();
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Type support not from this implementation. Got:\n"
      "    %s\n"
      "while fetching it",
      error_string.str);
    return NULL;
  }

  return type_support;
}
