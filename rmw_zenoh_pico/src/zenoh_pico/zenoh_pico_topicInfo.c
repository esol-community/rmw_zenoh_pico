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

#include "zenoh-pico/api/macros.h"
#include "zenoh-pico/api/primitives.h"
#include "zenoh-pico/api/types.h"
#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

const z_loaned_string_t *topic_name(ZenohPicoTopicInfo *topic)	{ return z_loan(topic->name); }
const z_loaned_string_t *topic_type(ZenohPicoTopicInfo *topic)	{ return z_loan(topic->type); }
const z_loaned_string_t *topic_hash(ZenohPicoTopicInfo *topic)	{ return z_loan(topic->hash); }
const z_loaned_string_t *topic_qos(ZenohPicoTopicInfo *topic)	{ return z_loan(topic->qos); }

z_owned_mutex_t mutex_ZenohPicoTopicInfo;

ZenohPicoTopicInfo *zenoh_pico_generate_topic_info(const char *name,
						   const rmw_qos_profile_t *qos,
						   const z_loaned_string_t *type,
						   const z_loaned_string_t *hash)
{
  RMW_ZENOH_FUNC_ENTRY();

  ZenohPicoTopicInfo *topic = NULL;
  ZenohPicoGenerateData(topic, ZenohPicoTopicInfo);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    topic,
    "failed to allocate struct for the ZenohPicoTopicInfo",
    return NULL);

  z_string_empty(&topic->name);
  z_string_empty(&topic->type);
  z_string_empty(&topic->hash);
  z_string_empty(&topic->qos);

  z_string_copy_from_str(&topic->name, name);

  z_string_clone(&topic->type, type);
  z_string_clone(&topic->hash, hash);

  {
    z_owned_string_t qos_key;
    rmw_qos_profile_t _qos_profile;
    memset(&_qos_profile, 0, sizeof(_qos_profile));
    test_qos_profile(&_qos_profile);

    qos_to_keyexpr(&_qos_profile, &qos_key);
    const z_loaned_string_t *_qos_key = z_loan(qos_key);
    RMW_ZENOH_LOG_INFO("qos = [%*s]", (int)z_string_len(_qos_key), z_string_data(_qos_key));

    z_string_copy_from_substr(&topic->qos, z_string_data(_qos_key), z_string_len(_qos_key));

    z_drop(z_move(qos_key));
  }

  return topic;
}

bool zenoh_pico_destroy_topic_info(ZenohPicoTopicInfo *topic)
{
  z_drop(z_move(topic->name));
  z_drop(z_move(topic->type));
  z_drop(z_move(topic->hash));
  z_drop(z_move(topic->qos));

  ZenohPicoDestroyData(topic, ZenohPicoTopicInfo);

  return true;
}

void zenoh_pico_debug_topic_info(ZenohPicoTopicInfo *topic)
{
  printf("topic info ...\n");

  const z_loaned_string_t *name = z_loan(topic->name);
  const z_loaned_string_t *type = z_loan(topic->type);
  const z_loaned_string_t *hash = z_loan(topic->hash);
  const z_loaned_string_t *qos  = z_loan(topic->qos);

  printf("\tname = [%.*s][%ld]\n", (int)z_string_len(name), z_string_data(name), z_string_len(name));
  printf("\ttype = [%.*s][%ld]\n", (int)z_string_len(type), z_string_data(type), z_string_len(type));
  printf("\thash = [%.*s][%ld]\n", (int)z_string_len(hash), z_string_data(hash), z_string_len(hash));
  printf("\tqos  = [%.*s][%ld]\n", (int)z_string_len(qos),  z_string_data(qos), z_string_len(qos));
}

// ------

z_result_t ros_topic_name_to_zenoh_key(const z_loaned_string_t *domain,
				       const z_loaned_string_t *name,
				       const z_loaned_string_t *type,
				       const z_loaned_string_t *hash,
				       z_owned_string_t *key)
{
  char keyexpr_str[RMW_ZENOH_PICO_MAX_LINENESS_LEN];
  char *keyexpr_ptr = keyexpr_str;
  int left_size = sizeof(keyexpr_str);;
  int ret;

  // A function that generates a key expression for message transport of the format
  // <ros_domain_id>/<topic_name>/<topic_type>/<topic_hash>
  memset(keyexpr_str, 0, sizeof(keyexpr_str));
  ret = snprintf(keyexpr_ptr, left_size, "%.*s/", (int)z_string_len(domain), z_string_data(domain));

  keyexpr_ptr += ret;
  left_size   -= ret;

  if(!z_string_is_empty(name)){
    const char *data = z_string_data(name);
    int len = (int)z_string_len(name);

    if(*(data +0) != '/')
      ret = snprintf(keyexpr_ptr, left_size, "%.*s", len, data);
    else
      ret = snprintf(keyexpr_ptr, left_size, "%.*s", len -1, data +1);

    keyexpr_ptr += ret;
    left_size   -= ret;

    if(*(data +len -1) != '/'){
      ret = snprintf(keyexpr_ptr, left_size, "/");
      keyexpr_ptr += ret;
      left_size   -= ret;
    }
  }

  ret = snprintf(keyexpr_ptr, left_size, "%.*s/%.*s",
		 (int)z_string_len(type), z_string_data(type),
		 (int)z_string_len(hash), z_string_data(hash));

  return z_string_copy_from_str(key, (const char *)keyexpr_str);
}
