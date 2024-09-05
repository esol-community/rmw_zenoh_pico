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

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

const char *get_zid(ZenohPicoEntity *entity)	        { return Z_STRING_VAL(entity->zid_); }
size_t get_nid(ZenohPicoEntity *entity)		        { return entity->nid_; }
size_t get_id(ZenohPicoEntity *entity)		        { return entity->id_; }
ZenohPicoEntityType get_type(ZenohPicoEntity *entity)   { return entity->type_; }

const char *get_node_domain(ZenohPicoEntity *entity)    { return node_domain(entity->node_info_); }
const char *get_node_namespace(ZenohPicoEntity *entity)	{ return node_namespace(entity->node_info_); }
const char *get_node_name(ZenohPicoEntity *entity)	{ return node_name(entity->node_info_); }
const char *get_node_enclave(ZenohPicoEntity *entity)	{ return node_enclave(entity->node_info_); }

const char *get_topic_name(ZenohPicoEntity *entity)     { return topic_name(entity->topic_info_); }
const char *get_topic_type(ZenohPicoEntity *entity)     { return topic_type(entity->topic_info_); }
const char *get_topic_hash(ZenohPicoEntity *entity)     { return topic_hash(entity->topic_info_); }
const char *get_topic_qos(ZenohPicoEntity *entity)      { return topic_qos(entity->topic_info_); }

z_mutex_t mutex_ZenohPicoEntity;

ZenohPicoEntity * zenoh_pico_generate_entity(z_id_t zid,
					     size_t id,
					     size_t nid,
					     ZenohPicoEntityType type,
					     ZenohPicoNodeInfo *node_info,
					     ZenohPicoTopicInfo *topic_info)
{
  ZenohPicoEntity *entity = NULL;
  ZenohPicoGenerateData(entity, ZenohPicoEntity);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    entity,
    "failed to allocate struct for the ZenohPicoEntity",
    return NULL);

  if(_z_id_check(zid)) {
    _z_bytes_t zid_data;
    zid_data.len	= _z_id_len(zid);
    zid_data.start	= zid.id;
    zid_data._is_alloc	= false;

    entity->zid_ = _z_string_from_bytes(&zid_data);
  }else{
    entity->zid_ = _z_string_make("");
  }

  entity->id_		= id;
  entity->nid_		= nid;
  entity->type_		= type;

  entity->node_info_	= node_info;
  entity->topic_info_	= topic_info;

  // zenoh_pico_debug_entity(entity);

  return entity;
}

static void _zenoh_pico_clear_entity_member(ZenohPicoEntity *entity)
{
  Z_STRING_FREE(entity->zid_);
}

void zenoh_pico_destroy_entity(ZenohPicoEntity *entity)
{
  _zenoh_pico_clear_entity_member(entity);

  if(entity->node_info_ != NULL){
    zenoh_pico_destroy_node_info(entity->node_info_);
    entity->node_info_ = NULL;
  }

  if(entity->topic_info_ != NULL){
    zenoh_pico_destroy_topic_info(entity->topic_info_);
    entity->topic_info_ = NULL;
  }

  ZenohPicoDestroyData(entity, ZenohPicoEntity);
}

void zenoh_pico_debug_entity(ZenohPicoEntity *entity)
{
  printf("--------- entity data ----------\n");
  printf("ref = %d\n", entity->ref_);

  Z_STRING_PRINTF(entity->zid_, zid);
  printf("id  = %d\n", (int)entity->id_);
  printf("nid = %d\n", (int)entity->nid_);

  const char *_type_name = "unknown type";
  switch(entity->type_){
  case Node:			_type_name = "Node"; break;
  case Publisher:		_type_name = "Publisher"; break;
  case Subscription:		_type_name = "Subscription"; break;
  case Service:			_type_name = "Service"; break;
  case Client:			_type_name = "Client"; break;
  }
  printf("type = %s\n",	_type_name);

  // debug node_info
  if(entity->node_info_ != NULL)
    zenoh_pico_debug_node_info(entity->node_info_);
  else
    printf("not found node info\n");

  // debug topic_info
  if(entity->topic_info_ != NULL)
    zenoh_pico_debug_topic_info(entity->topic_info_);
  else
    printf("not found topic info\n");

  return;
}

static size_t next_entity_id_ = 0;
size_t zenoh_pico_get_next_entity_id(void)
{
  return next_entity_id_++;
}
