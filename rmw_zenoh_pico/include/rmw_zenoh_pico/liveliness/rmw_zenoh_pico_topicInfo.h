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

#ifndef RMW_ZENOH_PICO_TOPICINFO_H
#define RMW_ZENOH_PICO_TOPICINFO_H

#include "zenoh-pico/api/types.h"
#include "zenoh-pico/collections/string.h"
#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoTopicInfo
  {
    int ref_;

    _z_string_t name_;
    _z_string_t type_;
    _z_string_t hash_;
    _z_string_t qos_;
  } ZenohPicoTopicInfo;

  extern const char *topic_name(ZenohPicoTopicInfo *topic);
  extern const char *topic_type(ZenohPicoTopicInfo *topic);
  extern const char *topic_hash(ZenohPicoTopicInfo *topic);
  extern const char *topic_qos(ZenohPicoTopicInfo *topic);

  extern ZenohPicoTopicInfo *zenoh_pico_generate_topic_info(z_string_t *name,
							    z_string_t *type,
							    z_string_t *typehash,
							    z_string_t *qos);
  extern bool zenoh_pico_destroy_topic_info(ZenohPicoTopicInfo *topic);

  extern void zenoh_pico_debug_topic_info(ZenohPicoTopicInfo *topic);

  // -------

  extern z_string_t ros_topic_name_to_zenoh_key(const char * domain,
						const char * name,
						const char * type,
						const char * hash);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
