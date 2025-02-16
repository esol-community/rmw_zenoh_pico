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

#ifndef RMW_ZENOH_PICO_TOPICINFO_H
#define RMW_ZENOH_PICO_TOPICINFO_H

#include "rmw/types.h"

#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoTopicInfo
  {
    int ref;

    z_owned_string_t name;
    z_owned_string_t type;
    z_owned_string_t hash;
    z_owned_string_t qos;
  } ZenohPicoTopicInfo;

  extern const z_loaned_string_t *topic_name(ZenohPicoTopicInfo *topic);
  extern const z_loaned_string_t *topic_type(ZenohPicoTopicInfo *topic);
  extern const z_loaned_string_t *topic_hash(ZenohPicoTopicInfo *topic);
  extern const z_loaned_string_t *topic_qos(ZenohPicoTopicInfo *topic);

  extern ZenohPicoTopicInfo *zenoh_pico_generate_topic_info(const char *name,
							    const rmw_qos_profile_t *qos,
							    const z_loaned_string_t *type,
							    const z_loaned_string_t *hash);

  extern bool zenoh_pico_destroy_topic_info(ZenohPicoTopicInfo *topic);

  extern void zenoh_pico_debug_topic_info(ZenohPicoTopicInfo *topic);

  // -------

  extern z_result_t ros_topic_name_to_zenoh_key(const z_loaned_string_t *domain,
						const z_loaned_string_t *name,
						const z_loaned_string_t *type,
						const z_loaned_string_t *hash,
						z_owned_string_t *key);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
