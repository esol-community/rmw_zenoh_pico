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

#ifndef RMW_ZENOH_PICO_NODEINFO_H
#define RMW_ZENOH_PICO_NODEINFO_H

#include "zenoh-pico/api/types.h"
#include <stddef.h>
#include <unistd.h>

#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoNodeInfo
  {
    int ref;
    z_owned_mutex_t lock;

    z_owned_string_t domain;
    z_owned_string_t ns;
    z_owned_string_t name;
    z_owned_string_t enclave;

  } ZenohPicoNodeInfo;

  extern const z_loaned_string_t *node_domain(ZenohPicoNodeInfo *node);
  extern const z_loaned_string_t *node_enclave(ZenohPicoNodeInfo *node);
  extern const z_loaned_string_t *node_namespace(ZenohPicoNodeInfo *node);
  extern const z_loaned_string_t *node_name(ZenohPicoNodeInfo *node);

  extern ZenohPicoNodeInfo *zenoh_pico_generate_node_info(size_t domain_id,
							  const char *ns,
							  const char *name,
							  const z_loaned_string_t *enclave);
  extern bool zenoh_pico_destroy_node_info(ZenohPicoNodeInfo *node);

  extern void zenoh_pico_debug_node_info(ZenohPicoNodeInfo *node);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
