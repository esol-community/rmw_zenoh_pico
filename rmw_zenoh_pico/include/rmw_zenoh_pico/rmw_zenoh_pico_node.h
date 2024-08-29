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

#ifndef RMW_ZENOH_PICO_NODE_H
#define RMW_ZENOH_PICO_NODE_H

#include <rmw/rmw.h>
#include <zenoh-pico.h>

#include <rmw_zenoh_pico/liveliness/rmw_zenoh_pico_nodeInfo.h>
#include <rmw_zenoh_pico/liveliness/rmw_zenoh_pico_entity.h>
#include <rmw_zenoh_pico/liveliness/rmw_zenoh_pico_liveliness.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico_session.h>

#include <zenoh-pico/collections/string.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoNodeData
  {
    int ref_;

    size_t id_;

    // Liveliness key for the node.
    _z_string_t token_key_;

    z_owned_keyexpr_t token_;

    // this node session
    ZenohPicoSession *session_;

    // this node entity
    ZenohPicoEntity *entity_;

  } ZenohPicoNodeData;

  extern ZenohPicoNodeData * zenoh_pico_generate_node_data(size_t node_id,
							   ZenohPicoSession *session,
							   ZenohPicoEntity *entity);

  extern ZenohPicoNodeData *zenoh_pico_loan_node_data(ZenohPicoNodeData *node_data);
  extern bool zenoh_pico_destroy_node_data(ZenohPicoNodeData *node_data);
  extern void zenoh_pico_debug_node_data(ZenohPicoNodeData *node_data);

  extern bool declaration_node_data(ZenohPicoNodeData *node_data);
  extern bool undeclaration_node_data(ZenohPicoNodeData *node_data);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
