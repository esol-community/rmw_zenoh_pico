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

#ifndef RMW_ZENOH_PICO_LIVELINESS_H
#define RMW_ZENOH_PICO_LIVELINESS_H

#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_entity.h"

#include <rmw/rmw.h>
#include <rmw/types.h>
#include <rmw/allocators.h>
#include <rmw/error_handling.h>

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_typesupport_microxrcedds_c/message_type_support.h>

extern z_string_t generate_liveliness(ZenohPicoEntity *entity);

extern z_string_t conv_domain(size_t domain);
extern z_string_t convert_hash(const rosidl_type_hash_t * type_hash);
extern z_string_t convert_message_type(const message_type_support_callbacks_t *callbacks);
extern z_string_t qos_to_keyexpr(rmw_qos_profile_t *qos);

#endif
