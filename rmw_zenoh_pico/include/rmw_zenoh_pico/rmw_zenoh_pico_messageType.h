/*
 * Copyright(C) 2024 eSOL Co., Ltd.
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

#ifndef ZENOH_PICO_MESSAGETYPE_H
#define ZENOH_PICO_MESSAGETYPE_H

#include <rmw/rmw.h>
#include <rmw/types.h>

#include <ucdr/microcdr.h>

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_typesupport_microxrcedds_c/message_type_support.h>

extern const rosidl_message_type_support_t * find_message_type_support(
  const rosidl_message_type_support_t * type_supports);

#endif
