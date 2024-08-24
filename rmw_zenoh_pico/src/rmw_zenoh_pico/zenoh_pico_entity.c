
#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"

#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_nodeInfo.h"
#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_topicInfo.h"
#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_entity.h"

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

ZenohPicoEntity * zenoh_pico_generate_entitiy(z_id_t zid,
					      size_t id,
					      size_t nid,
					      ZenohPicoEntityType type,
					      ZenohPicoNodeInfo_t *node_info,
					      ZenohPicoTopicInfo_t *topic_info)
{
  ZenohPicoEntity *entity = NULL;
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

  entity->id_		= id;
  entity->nid_		= nid;
  entity->type_		= type;

  entity->node_info_	= node_info;
  entity->topic_info_	= topic_info;

  return entity;
}

static void _zenoh_pico_clear_entitiy_member(ZenohPicoEntity *entity)
{
  Z_STRING_FREE(entity->zid_);
}

void zenoh_pico_destroy_entitiy(ZenohPicoEntity *entity)
{
  _zenoh_pico_clear_entitiy_member(entity);

  if(entity->node_info_ != NULL){
    zenoh_pico_destroy_node_info(entity->node_info_);
    entity->node_info_ = NULL;
  }

  if(entity->topic_info_ != NULL){
    zenoh_pico_destroy_topic_info(entity->topic_info_);
    entity->topic_info_ = NULL;
  }

  ZenohPicoDestroyData(entity);
}

void zenoh_pico_debug_entitiy(ZenohPicoEntity *entity)
{
  printf("--------- entity data ----------\n");
  printf("ref = %d\n", entity->ref_);

  Z_STRING_PRINTF(entity->zid_, zid);
  printf("id  = %ld\n", entity->id_);
  printf("nid = %ld\n", entity->nid_);

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


// ------------------------

static size_t next_entity_id_ = 0;
size_t zenoh_pico_get_next_entity_id(void)
{
  return next_entity_id_++;
}
