#ifndef RMW_ZENOH_PICO_ENTITY_H
#define RMW_ZENOH_PICO_ENTITY_H

#include <stddef.h>
#include <unistd.h>

#include <zenoh-pico.h>

#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_nodeInfo.h"
#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_topicInfo.h"

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
    int ref_;

    _z_string_t zid_;
    size_t id_;
    size_t nid_;

    ZenohPicoEntityType type_;

    ZenohPicoNodeInfo *node_info_;
    ZenohPicoTopicInfo *topic_info_;

  } ZenohPicoEntity;

  extern const char *get_zid(ZenohPicoEntity *entity);
  extern size_t get_nid(ZenohPicoEntity *entity);
  extern size_t get_id(ZenohPicoEntity *entity);
  extern ZenohPicoEntityType get_type(ZenohPicoEntity *entity);

  extern const char *get_node_domain(ZenohPicoEntity *entity);
  extern const char *get_node_enclave(ZenohPicoEntity *entity);
  extern const char *get_node_namespace(ZenohPicoEntity *entity);
  extern const char *get_node_name(ZenohPicoEntity *entity);

  extern const char *get_topic_name(ZenohPicoEntity *entity);
  extern const char *get_topic_type(ZenohPicoEntity *entity);
  extern const char *get_topic_hash(ZenohPicoEntity *entity);
  extern const char *get_topic_qos(ZenohPicoEntity *entity);

  // functions
  extern ZenohPicoEntity * zenoh_pico_generate_entity(z_id_t zid,
						      size_t id,
						      size_t nid,
						      ZenohPicoEntityType type,
						      ZenohPicoNodeInfo *node_info,
						      ZenohPicoTopicInfo *topic_info);
  extern void zenoh_pico_destroy_entity(ZenohPicoEntity *entity);
  extern void zenoh_pico_debug_entity(ZenohPicoEntity *entity);

  extern size_t zenoh_pico_get_next_entity_id(void);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
