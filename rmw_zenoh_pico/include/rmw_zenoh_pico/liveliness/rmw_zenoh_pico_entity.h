#ifndef RMW_ZENOH_PICO_ENTITY_H
#define RMW_ZENOH_PICO_ENTITY_H

#include <stddef.h>
#include <unistd.h>

#include <zenoh-pico.h>

#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_nodeInfo.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_topic.h"

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef enum _ZenohPicoEntityType
  {
    Node,
    Publisher,
    Subscription,
    Service,
    Client
  } ZenohPicoEntityType;

  typedef struct _ZenohPicoEntity
  {
    bool is_alloc_;

    _z_string_t zid_;
    _z_string_t id_;
    _z_string_t nid_;

    ZenohPicoEntityType type_;

    ZenohPicoNodeInfo_t *node_info_;
    ZenohPicoTopicInfo_t *topic_info_;

  } ZenohPicoEntity;

  // getter entry
  extern const _z_string_t *entity_zid(ZenohPicoEntity *entity);
  extern const _z_string_t *entity_nid(ZenohPicoEntity *entity);
  extern const _z_string_t *entity_id(ZenohPicoEntity *entity);

  extern ZenohPicoEntityType entity_type(ZenohPicoEntity *entity);

  extern bool entity_enable_node(ZenohPicoEntity *entity);
  extern const _z_string_t *entity_node_domain(ZenohPicoEntity *entity);
  extern const _z_string_t *entity_node_namespace(ZenohPicoEntity *entity);
  extern const _z_string_t *entity_node_name(ZenohPicoEntity *entity);
  extern const _z_string_t *entity_node_enclave(ZenohPicoEntity *entity);

  extern bool entity_enable_topic(ZenohPicoEntity *entity);
  extern const _z_string_t *entity_topic_name(ZenohPicoEntity *entity);
  extern const _z_string_t *entity_topic_type(ZenohPicoEntity *entity);
  extern const _z_string_t *entity_topic_typehash(ZenohPicoEntity *entity);

  // functions
  extern ZenohPicoEntity * zenoh_pico_generate_entitiy(ZenohPicoEntity *entity,
						       z_id_t zid,
						       const char *id,
						       const char *nid,
						       ZenohPicoEntityType type,
						       ZenohPicoNodeInfo_t *node_info,
						       ZenohPicoTopicInfo_t *topic_info);

  extern void zenoh_pico_destroy_entitiy(ZenohPicoEntity *entity);

  extern void zenoh_pico_clone_entitiy(ZenohPicoEntity *dst, ZenohPicoEntity *src);
  extern void zenoh_pico_debug_entitiy(ZenohPicoEntity *entity);

// -------------------------------------------------
  extern size_t zenoh_pico_get_next_entity_id(void);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
