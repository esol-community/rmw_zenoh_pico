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

#ifndef RMW_ZENOH_PICO_NODE_H
#define RMW_ZENOH_PICO_NODE_H

#include <rmw/rmw.h>
#include <zenoh-pico.h>

#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_nodeInfo.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_entity.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_liveliness.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico_session.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoNodeData
  {
    int ref;
    z_owned_mutex_t lock;

    size_t id;

    // Liveliness key for the node.
    z_owned_string_t liveliness_key;

    // Liveliness token for the node.
    z_owned_liveliness_token_t token;

    // this node session
    ZenohPicoSession *session;

    // this node entity
    ZenohPicoEntity *entity;

  } ZenohPicoNodeData;

  extern ZenohPicoNodeData * zenoh_pico_generate_node_data(size_t domain_id,
							   const char *name,
							   const char *namespace,
							   ZenohPicoSession *session);
  extern bool zenoh_pico_destroy_node_data(ZenohPicoNodeData *node_data);

  extern void zenoh_pico_debug_node_data(ZenohPicoNodeData *node_data);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
