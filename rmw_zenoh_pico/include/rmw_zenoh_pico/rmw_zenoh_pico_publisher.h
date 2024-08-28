#ifndef RMW_ZENOH_PICO_PUBLISHER_H
#define RMW_ZENOH_PICO_PUBLISHER_H

#include <rmw/rmw.h>
#include <zenoh-pico.h>

#include <rmw_zenoh_pico/liveliness/rmw_zenoh_pico_nodeInfo.h>
#include <rmw_zenoh_pico/liveliness/rmw_zenoh_pico_entity.h>
#include <rmw_zenoh_pico/liveliness/rmw_zenoh_pico_liveliness.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico_node.h>

typedef struct _ZenohPicoPubData {
  uint ref_;

  size_t id_;

  // Liveliness key for the publisher.
  _z_string_t token_key_;
  _z_string_t topic_key_;

  // Liveliness token for the publisher.
  z_owned_keyexpr_t token_;
  z_owned_publisher_t publisher_;

  // this node
  ZenohPicoNodeData *node_;

  // this subscribe entity
  ZenohPicoEntity *entity_;

  // CDR callback
  const message_type_support_callbacks_t *callbacks_;

  // Store the actual QoS profile used to configure this publisher.
  rmw_qos_profile_t adapted_qos_profile_;

  uint8_t pub_gid_[RMW_GID_STORAGE_SIZE];

} ZenohPicoPubData;

extern ZenohPicoPubData * zenoh_pico_generate_publisher_data(
  size_t pub_id,
  ZenohPicoNodeData *node,
  ZenohPicoEntity *entity,
  const rosidl_message_type_support_t * type_support,
  const message_type_support_callbacks_t *callbacks,
  rmw_qos_profile_t *qos_profile);
extern bool zenoh_pico_destroy_publisher_data(ZenohPicoPubData *pub_data);
extern bool undeclaration_publisher_data(ZenohPicoPubData *pub_data);
extern void zenoh_pico_debug_publisher_data(ZenohPicoPubData *pub_data);
extern bool declaration_publisher_data(ZenohPicoPubData *pub_data);
extern bool undeclaration_publisher_data(ZenohPicoPubData *pub_data);

#endif