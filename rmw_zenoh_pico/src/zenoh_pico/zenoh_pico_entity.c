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

const z_loaned_string_t *get_zid(ZenohPicoEntity *entity)		{ return z_loan(entity->zid); }
size_t get_nid(ZenohPicoEntity *entity)					{ return entity->nid; }
size_t get_id(ZenohPicoEntity *entity)					{ return entity->id; }
ZenohPicoEntityType get_type(ZenohPicoEntity *entity)			{ return entity->type; }

const z_loaned_string_t *get_node_domain(ZenohPicoEntity *entity)	{ return node_domain(entity->node_info); }
const z_loaned_string_t *get_node_namespace(ZenohPicoEntity *entity)	{ return node_namespace(entity->node_info); }
const z_loaned_string_t *get_node_name(ZenohPicoEntity *entity)		{ return node_name(entity->node_info); }
const z_loaned_string_t *get_node_enclave(ZenohPicoEntity *entity)	{ return node_enclave(entity->node_info); }

const z_loaned_string_t *get_topic_name(ZenohPicoEntity *entity)	{ return topic_name(entity->topic_info); }
const z_loaned_string_t *get_topic_type(ZenohPicoEntity *entity)	{ return topic_type(entity->topic_info); }
const z_loaned_string_t *get_topic_hash(ZenohPicoEntity *entity)	{ return topic_hash(entity->topic_info); }
const z_loaned_string_t *get_topic_qos(ZenohPicoEntity *entity)		{ return topic_qos(entity->topic_info); }

z_owned_mutex_t mutex_ZenohPicoEntity;

ZenohPicoEntity * zenoh_pico_generate_entity(
  z_id_t *zid,
  size_t nid,
  ZenohPicoEntityType type,
  ZenohPicoNodeInfo *node_info,
  ZenohPicoTopicInfo *topic_info)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  ZenohPicoEntity *entity = NULL;
  ZenohPicoGenerateData(entity, ZenohPicoEntity);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    entity,
    "failed to allocate struct for the ZenohPicoEntity",
    return NULL);

  if(_z_id_check(*zid))
    z_id_to_string(zid, &entity->zid);
  else
    z_string_copy_from_str(&entity->zid, "");

  entity->id		= zenoh_pico_get_next_entity_id();
  entity->nid		= nid;
  entity->type		= type;

  entity->node_info	= node_info;
  entity->topic_info	= topic_info;

  // if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
  //   zenoh_pico_debug_entity(entity);
  // }

  return entity;
}

ZenohPicoEntity * zenoh_pico_generate_topic_entity(
  z_id_t *zid,
  size_t nid,
  ZenohPicoNodeInfo *node_info,
  const char * topic_name,
  const rosidl_message_type_support_t * type_support,
  const rmw_qos_profile_t *qos_profile,
  ZenohPicoEntityType type)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  z_owned_string_t hash_data;
  z_owned_string_t type_name;

  // get hash data
  const rosidl_type_hash_t * type_hash = type_support->get_type_hash_func(type_support);
  (void)convert_hash(type_hash, &hash_data);

  // generate message type
  const message_type_support_callbacks_t *callbacks
    = (const message_type_support_callbacks_t *)(type_support->data);
  (void)convert_message_type(callbacks, &type_name);

  // generate topic data
  ZenohPicoTopicInfo *topic_info;
  topic_info = zenoh_pico_generate_topic_info(topic_name,
					      qos_profile,
					      z_loan(type_name),
					      z_loan(hash_data));

  z_drop(z_move(hash_data));
  z_drop(z_move(type_name));

  if(topic_info == NULL)
    return NULL;

  return zenoh_pico_generate_entity(zid, nid, type, node_info, topic_info);
}

bool zenoh_pico_destroy_entity(ZenohPicoEntity *entity)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  RMW_CHECK_ARGUMENT_FOR_NULL(entity, false);

  z_drop(z_move(entity->zid));

  if(entity->node_info != NULL){
    zenoh_pico_destroy_node_info(entity->node_info);
    entity->node_info = NULL;
  }

  if(entity->topic_info != NULL){
    zenoh_pico_destroy_topic_info(entity->topic_info);
    entity->topic_info = NULL;
  }

  ZenohPicoDestroyData(entity, ZenohPicoEntity);

  return true;
}

void zenoh_pico_debug_entity(ZenohPicoEntity *entity)
{
  printf("--------- entity data ----------\n");
  printf("ref = %d\n", entity->ref);

  Z_STRING_PRINTF(entity->zid, zid);
  printf("id  = %d\n", (int)entity->id);
  printf("nid = %d\n", (int)entity->nid);

  const char *_type_name = "unknown type";
  switch(entity->type){
  case Node:			_type_name = "Node"; break;
  case Publisher:		_type_name = "Publisher"; break;
  case Subscription:		_type_name = "Subscription"; break;
  case Service:			_type_name = "Service"; break;
  case Client:			_type_name = "Client"; break;
  }
  printf("type = %s\n",	_type_name);

  // debug node_info
  if(entity->node_info != NULL)
    zenoh_pico_debug_node_info(entity->node_info);
  else
    printf("not found node info\n");

  // debug topic_info
  if(entity->topic_info != NULL)
    zenoh_pico_debug_topic_info(entity->topic_info);
  else
    printf("not found topic info\n");

  return;
}

static size_t next_entity_id_ = 0;
size_t zenoh_pico_get_next_entity_id(void)
{
  return next_entity_id_++;
}
