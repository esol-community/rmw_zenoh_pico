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

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

const char *topic_name(ZenohPicoTopicInfo *topic)	{ return Z_STRING_VAL(topic->name); }
const char *topic_type(ZenohPicoTopicInfo *topic)	{ return Z_STRING_VAL(topic->type); }
const char *topic_hash(ZenohPicoTopicInfo *topic)	{ return Z_STRING_VAL(topic->hash); }
const char *topic_qos(ZenohPicoTopicInfo *topic)	{ return Z_STRING_VAL(topic->qos); }

z_mutex_t mutex_ZenohPicoTopicInfo;

static void _zenoh_pico_clear_topic_info_member(ZenohPicoTopicInfo *topic)
{
  Z_STRING_FREE(topic->name);
  Z_STRING_FREE(topic->type);
  Z_STRING_FREE(topic->hash);
  Z_STRING_FREE(topic->qos);
}

ZenohPicoTopicInfo *zenoh_pico_generate_topic_info(z_string_t *name,
						   z_string_t *type,
						   z_string_t *hash,
						   z_string_t *qos)
{
  ZenohPicoTopicInfo *topic = NULL;
  ZenohPicoGenerateData(topic, ZenohPicoTopicInfo);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    topic,
    "failed to allocate struct for the ZenohPicoTopicInfo",
    return NULL);

  _zenoh_pico_clear_topic_info_member(topic);

  _z_string_move(&topic->name, name);
  _z_string_move(&topic->type, type);
  _z_string_move(&topic->hash, hash);
  _z_string_move(&topic->qos, qos);

  return topic;
}

bool zenoh_pico_destroy_topic_info(ZenohPicoTopicInfo *topic)
{
  _zenoh_pico_clear_topic_info_member(topic);

  ZenohPicoDestroyData(topic, ZenohPicoTopicInfo);

  return true;
}

void zenoh_pico_debug_topic_info(ZenohPicoTopicInfo *topic)
{
  printf("topic info ...\n");

  printf("\tname = %s\n", topic->name.val);
  printf("\ttype = %s\n", topic->type.val);
  printf("\thash = %s\n", topic->hash.val);
  printf("\tqos  = %s\n", topic->qos.val);
}

// ------

z_string_t ros_topic_name_to_zenoh_key(const char * domain,
				       const char * name,
				       const char * type,
				       const char * hash)
{
  char keyexpr_str[RMW_ZENOH_PICO_MAX_LINENESS_LEN];
  char *keyexpr_ptr = keyexpr_str;
  int left_size = sizeof(keyexpr_str);;
  int ret;

  // A function that generates a key expression for message transport of the format
  // <ros_domain_id>/<topic_name>/<topic_type>/<topic_hash>
  memset(keyexpr_str, 0, sizeof(keyexpr_str));
  ret = snprintf(keyexpr_ptr, left_size, "%s/", domain);
  keyexpr_ptr += ret;
  left_size   -= ret;

  if(strlen(name) > 0){
    if(*(name +0) != '/')
      ret = snprintf(keyexpr_ptr, left_size, "%s", name);
    else
      ret = snprintf(keyexpr_ptr, left_size, "%s", name +1);

    keyexpr_ptr += ret;
    left_size   -= ret;

    if(*(name +strlen(name)) != '/'){
      ret = snprintf(keyexpr_ptr, left_size, "/");
      keyexpr_ptr += ret;
      left_size   -= ret;
    }
  }

  ret = snprintf(keyexpr_ptr, left_size, "%s/%s", type, hash);

  RMW_ZENOH_LOG_INFO("new z_keyexpr is [%s]\n", keyexpr_str);

  return z_string_make(keyexpr_str);
}
