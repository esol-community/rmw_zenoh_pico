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
    bool is_alloc_;

    // Liveliness key for the node.
    _z_string_t key_;
    z_owned_keyexpr_t keyexpr_;

    // this node session
    ZenohPicoSession *session_;

    // this node entity
    ZenohPicoEntity *entity_;

  } ZenohPicoNodeData;

  extern ZenohPicoNodeData * zenoh_pico_generate_node_data(ZenohPicoNodeData *node_data,
							   ZenohPicoSession *session,
							   ZenohPicoEntity *entity);
  extern bool zenoh_pico_destroy_node_data(ZenohPicoNodeData *node_data);
  extern void zenoh_pico_debug_node_data(ZenohPicoNodeData *node_data);

  extern rmw_node_t * rmw_node_generate(rmw_context_t *context, ZenohPicoNodeData *node_data);
  extern rmw_ret_t rmw_node_destroy(rmw_node_t * node);

  // --------------------------

  extern bool declaration_node_data(ZenohPicoNodeData *node_data);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
