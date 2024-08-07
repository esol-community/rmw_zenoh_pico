#ifndef RMW_ZENOH_PICO_ENTITY_H
#define RMW_ZENOH_PICO_ENTITY_H

#include <stddef.h>
#include <unistd.h>

#include <zenoh-pico.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico_node.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_topic.h>

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

  typedef struct _ZenohPicoEntity_s
  {
    bool alloc_;

    _z_string_t zid_;
    _z_string_t nid_;
    _z_string_t id_;

    _z_string_t keyexpr_;

    ZenohPicoEntityType type_;

    bool enable_node_;
    ZenohPicoNodeInfo_t node_info_;

    bool enable_topic_;
    ZenohPicoTopicInfo_t topic_info_;

  } ZenohPicoEntity_t;

  // getter entry
  extern const _z_string_t *entity_zid(ZenohPicoEntity_t *entity);
  extern const _z_string_t *entity_nid(ZenohPicoEntity_t *entity);
  extern const _z_string_t *entity_id(ZenohPicoEntity_t *entity);
  extern const _z_string_t *entity_keyexpr(ZenohPicoEntity_t *entity);

  extern ZenohPicoEntityType entity_type(ZenohPicoEntity_t *entity);

  extern bool entity_enable_node(ZenohPicoEntity_t *entity);
  extern const _z_string_t *entity_node_domain(ZenohPicoEntity_t *entity);
  extern const _z_string_t *entity_node_namespace(ZenohPicoEntity_t *entity);
  extern const _z_string_t *entity_node_name(ZenohPicoEntity_t *entity);
  extern const _z_string_t *entity_node_enclave(ZenohPicoEntity_t *entity);

  extern bool entity_enable_topic(ZenohPicoEntity_t *entity);
  extern const _z_string_t *entity_topic_name(ZenohPicoEntity_t *entity);
  extern const _z_string_t *entity_topic_type(ZenohPicoEntity_t *entity);
  extern const _z_string_t *entity_topic_typehash(ZenohPicoEntity_t *entity);

  // functions
  extern ZenohPicoEntity_t * zenoh_pico_generate_entitiy(ZenohPicoEntity_t *entity,
					      z_id_t *zid,
					      const char *nid,
					      const char *id,
					      ZenohPicoEntityType type,
					      ZenohPicoNodeInfo_t *node_info,
					      ZenohPicoTopicInfo_t *topic_info);

  extern void zenoh_pico_destroy_entitiy(ZenohPicoEntity_t *entity);

  extern void zenoh_pico_clone_entitiy(ZenohPicoEntity_t *dst, ZenohPicoEntity_t *src);
  extern void zenoh_pico_debug_entitiy(ZenohPicoEntity_t *entity);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
