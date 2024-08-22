#ifndef RMW_ZENOH_PICO_SUBSCRIPTION_H
#define RMW_ZENOH_PICO_SUBSCRIPTION_H

#include <rmw/rmw.h>
#include <zenoh-pico.h>

#include <rmw_zenoh_pico/liveliness/rmw_zenoh_pico_nodeInfo.h>
#include <rmw_zenoh_pico/liveliness/rmw_zenoh_pico_entity.h>
#include <rmw_zenoh_pico/liveliness/rmw_zenoh_pico_liveliness.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico_node.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_receiveMessage.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_event_callbacks.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoSubData {
    uint ref_;

    size_t id_;

    // Liveliness key for the node.
    _z_string_t token_key_;
    _z_string_t topic_key_;

    z_owned_keyexpr_t token_;
    z_owned_subscriber_t subscriber_;

    // this node
    ZenohPicoNodeData *node_;

    // this subscribe entity
    ZenohPicoEntity *entity_;

    // Store the actual QoS profile used to configure this subscription.
    rmw_qos_profile_t adapted_qos_profile_;

    // recived message list
    ReceiveMessageDataList list_;

    // data callback on new message
    DataCallbackManager data_callback_mgr;

  } ZenohPicoSubData;

  extern ZenohPicoSubData * zenoh_pico_generate_subscription_data(size_t sub_id,
							 ZenohPicoNodeData *node,
							 ZenohPicoEntity *entity,
							 const rosidl_message_type_support_t * type_support,
							 rmw_qos_profile_t *qos_profile);

  extern bool zenoh_pico_destroy_subscription_data(ZenohPicoSubData *sub_data);
  extern void zenoh_pico_debug_subscription_data(ZenohPicoSubData *sub_data);

  // --------------------------

  extern bool declaration_subscription_data(ZenohPicoSubData *sub_data);
  extern bool undeclaration_subscription_data(ZenohPicoSubData *sub_data);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
