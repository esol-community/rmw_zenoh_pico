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

#ifndef RMW_ZENOH_PICO_ENTITY_H
#define RMW_ZENOH_PICO_ENTITY_H

#include <stddef.h>
#include <unistd.h>

#include <rmw/rmw.h>

#include <rosidl_typesupport_microxrcedds_c/message_type_support.h>

#include <zenoh-pico.h>

#include "rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_nodeInfo.h"
#include "rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_topicInfo.h"

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
    int ref;
    z_owned_mutex_t lock;

    z_owned_string_t zid;
    size_t id;
    size_t nid;

    ZenohPicoEntityType type;

    ZenohPicoNodeInfo *node_info;
    ZenohPicoTopicInfo *topic_info;

  } ZenohPicoEntity;

  extern const z_loaned_string_t *get_zid(ZenohPicoEntity *entity);
  extern size_t get_nid(ZenohPicoEntity *entity);
  extern size_t get_id(ZenohPicoEntity *entity);
  extern ZenohPicoEntityType get_type(ZenohPicoEntity *entity);

  extern const z_loaned_string_t *get_node_domain(ZenohPicoEntity *entity);
  extern const z_loaned_string_t *get_node_enclave(ZenohPicoEntity *entity);
  extern const z_loaned_string_t *get_node_namespace(ZenohPicoEntity *entity);
  extern const z_loaned_string_t *get_node_name(ZenohPicoEntity *entity);

  extern const z_loaned_string_t *get_topic_name(ZenohPicoEntity *entity);
  extern const z_loaned_string_t *get_topic_type(ZenohPicoEntity *entity);
  extern const z_loaned_string_t *get_topic_hash(ZenohPicoEntity *entity);
  extern const z_loaned_string_t *get_topic_qos(ZenohPicoEntity *entity);

  // functions
  extern ZenohPicoEntity * zenoh_pico_generate_node_entity(z_id_t *zid,
							   size_t nid,
							   ZenohPicoEntityType type,
							   ZenohPicoNodeInfo *node_info,
							   ZenohPicoTopicInfo *topic_info);

  extern ZenohPicoEntity * zenoh_pico_generate_subscription_entity(z_id_t *zid,
								   size_t nid,
								   ZenohPicoNodeInfo *node_info,
								   const char * topic_name,
								   const rosidl_message_type_support_t * type_support,
								   const rmw_qos_profile_t *qos_profile);

  extern ZenohPicoEntity * zenoh_pico_generate_publisher_entity(z_id_t *zid,
								size_t nid,
								ZenohPicoNodeInfo *node_info,
								const char * topic_name,
								const rosidl_message_type_support_t * type_support,
								const rmw_qos_profile_t *qos_profile);

  extern const message_type_support_callbacks_t * get_request_callback(
    const rosidl_service_type_support_t * type_support);

  extern const message_type_support_callbacks_t * get_response_callback(
    const rosidl_service_type_support_t * type_support);

  extern ZenohPicoEntity * zenoh_pico_generate_service_entity(z_id_t *zid,
							      size_t nid,
							      ZenohPicoNodeInfo *node_info,
							      const char * topic_name,
							      const rosidl_service_type_support_t * type_support,
							      const rmw_qos_profile_t *qos_profile);

  extern ZenohPicoEntity * zenoh_pico_generate_client_entity(z_id_t *zid,
							     size_t nid,
							     ZenohPicoNodeInfo *node_info,
							     const char * topic_name,
							     const rosidl_service_type_support_t * type_support,
							     const rmw_qos_profile_t *qos_profile);

  extern bool zenoh_pico_destroy_entity(ZenohPicoEntity *entity);
  extern void zenoh_pico_debug_entity(ZenohPicoEntity *entity);

  extern size_t zenoh_pico_get_next_entity_id(void);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
