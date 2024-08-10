
#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_nodeInfo.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_topic.h"

#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_entity.h"

const _z_string_t *entity_zid(ZenohPicoEntity *entity)		{ return &entity->zid_; }
const _z_string_t *entity_nid(ZenohPicoEntity *entity)		{ return &entity->nid_; }
ZenohPicoEntityType entity_type(ZenohPicoEntity *entity)	{ return entity->type_; }

const _z_string_t *entity_node_domain(ZenohPicoEntity *entity) {
  return node_domain(entity->node_info_);
}
const _z_string_t *entity_node_namespace(ZenohPicoEntity *entity) {
  return node_namespace(entity->node_info_);
}
const _z_string_t *entity_node_name(ZenohPicoEntity *entity) {
  return node_name(entity->node_info_);
}
const _z_string_t *entity_node_enclave(ZenohPicoEntity *entity) {
  return node_enclave(entity->node_info_);
}
const _z_string_t *entity_topic_name(ZenohPicoEntity *entity)	{
  return topic_name(entity->topic_info_);
}
const _z_string_t *entity_topic_type(ZenohPicoEntity *entity)	{
  return topic_type(entity->topic_info_);
}
const _z_string_t *entity_topic_typehash(ZenohPicoEntity *entity) {
  return topic_typehash(entity->topic_info_);
}

ZenohPicoEntity * zenoh_pico_generate_entitiy(ZenohPicoEntity *entity,
					      z_id_t zid,
					      const char *id,
					      const char *nid,
					      ZenohPicoEntityType type,
					      ZenohPicoNodeInfo_t *node_info,
					      ZenohPicoTopicInfo_t *topic_info)
{

  ZenohPicoGenerateData(entity, ZenohPicoEntity);
  if(entity == NULL){
    return NULL;
  }

  if(_z_id_check(zid)) {
    _z_bytes_t zid_data;
    zid_data.len	= _z_id_len(zid);
    zid_data.start	= zid.id;
    zid_data._is_alloc	= false;

    entity->zid_ = _z_string_from_bytes(&zid_data);
  }else{
    entity->zid_ = _z_string_make("");
  }

  entity->id_		= (nid != NULL) ? _z_string_make("") : _z_string_make(id);
  entity->nid_		= (nid != NULL) ? _z_string_make("") : _z_string_make(nid);
  entity->type_		= type;
  entity->node_info_	= node_info;
  entity->topic_info_	= topic_info;

  return entity;
}

static void _zenoh_pico_clear_entitiy_menber(ZenohPicoEntity *entity)
{
  Z_STRING_FREE(entity->zid_);
  Z_STRING_FREE(entity->id_);
  Z_STRING_FREE(entity->nid_);
}

void zenoh_pico_destroy_entitiy(ZenohPicoEntity *entity)
{
  _zenoh_pico_clear_entitiy_menber(entity);

  if(entity->node_info_ != NULL){
    zenoh_pico_destroy_node_info(entity->node_info_);
    entity->node_info_ = NULL;
  }

  if(entity->topic_info_ != NULL){
    zenoh_pico_destroy_topic(entity->topic_info_);
    entity->topic_info_ = NULL;
  }

  ZenohPicoDestroyData(entity);
}

void zenoh_pico_clone_entitiy(ZenohPicoEntity *dst, ZenohPicoEntity *src)
{
  _z_string_copy(&dst->zid_, &src->zid_);
  _z_string_copy(&dst->id_, &src->id_);
  _z_string_copy(&dst->nid_, &src->nid_);

  dst->type_ = src->type_;

  if(src->node_info_ != NULL)
    zenoh_pico_clone_node_info(dst->node_info_, src->node_info_);

  if(src->topic_info_ != NULL)
    zenoh_pico_clone_topic(dst->topic_info_, src->topic_info_);

  return;
}



void zenoh_pico_debug_entitiy(ZenohPicoEntity *entity)
{
  printf("--------- entity data ----------\n");
  printf("is_alloc = %d\n", entity->is_alloc_);

  PRINTF_Z_STRING(entity->zid_, zid);
  PRINTF_Z_STRING(entity->id_, id);
  PRINTF_Z_STRING(entity->nid_, nid);

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
    zenoh_pico_debug_topic(entity->topic_info_);
  else
    printf("not found topic info\n");

  return;
}


// ------------------------

static size_t next_entity_id_ = 0;
size_t zenoh_pico_get_next_entity_id(void)
{
  return next_entity_id_++;
}
