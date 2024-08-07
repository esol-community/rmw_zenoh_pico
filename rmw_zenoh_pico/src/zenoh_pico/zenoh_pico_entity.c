
#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_node.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_topic.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_entity.h"

const _z_string_t *entity_zid(ZenohPicoEntity_t *entity)	{ return &entity->zid_; }
const _z_string_t *entity_nid(ZenohPicoEntity_t *entity)	{ return &entity->nid_; }
const _z_string_t *entity_id(ZenohPicoEntity_t *entity)		{ return &entity->id_;  }
const _z_string_t *entity_keyexpr(ZenohPicoEntity_t *entity)	{ return &entity->keyexpr_; }
ZenohPicoEntityType entity_type(ZenohPicoEntity_t *entity)	{ return entity->type_; }
bool entity_enable_node(ZenohPicoEntity_t *entity)		{ return entity->enable_node_; }

const _z_string_t *entity_node_domain(ZenohPicoEntity_t *entity) {
  return node_domain(&entity->node_info_);
}
const _z_string_t *entity_node_namespace(ZenohPicoEntity_t *entity) {
  return node_namespace(&entity->node_info_);
}
const _z_string_t *entity_node_name(ZenohPicoEntity_t *entity) {
  return node_name(&entity->node_info_);
}
const _z_string_t *entity_node_enclave(ZenohPicoEntity_t *entity) {
  return node_enclave(&entity->node_info_);
}
bool entity_enable_topic(ZenohPicoEntity_t *entity) {
  return entity->enable_topic_;
}
const _z_string_t *entity_topic_name(ZenohPicoEntity_t *entity)	{
  return topic_name(&entity->topic_info_);
}
const _z_string_t *entity_topic_type(ZenohPicoEntity_t *entity)	{
  return topic_type(&entity->topic_info_);
}
const _z_string_t *entity_topic_typehash(ZenohPicoEntity_t *entity) {
  return topic_typehash(&entity->topic_info_);
}

ZenohPicoEntity_t * zenoh_pico_generate_entitiy(ZenohPicoEntity_t *entity,
				     z_id_t *zid,
				     const char *nid,
				     const char *id,
				     ZenohPicoEntityType type,
				     ZenohPicoNodeInfo_t *node_info,
				     ZenohPicoTopicInfo_t *topic_info)
{

  ZenohPicoGenerateData(entity, ZenohPicoEntity_t);
  if(entity == NULL){
    return NULL;
  }

  if(zid != NULL){
    _z_bytes_t zid_data;
    zid_data.len	= sizeof(zid->id);
    zid_data.start	= zid->id;
    zid_data._is_alloc	= false;

    entity->zid_ = _z_string_from_bytes(&zid_data);
  }else{
    entity->zid_ = _z_string_make("");
  }

  entity->nid_ = (nid != NULL) ? _z_string_make("") : _z_string_make(nid);
  entity->id_  = (id != NULL) ? _z_string_make("") : _z_string_make(id);

  entity->type_ = type;

  if(node_info != NULL){
    zenoh_pico_generate_node(&entity->node_info_,
		  node_info->domain_id_.val,
		  node_info->ns_.val,
		  node_info->name_.val,
		  node_info->enclave_.val);
    entity->enable_node_ = true;
  }

  if(topic_info != NULL){
    zenoh_pico_generate_topic(&entity->topic_info_,
		   topic_info->name_.val,
		   topic_info->type_.val,
		   topic_info->typehash_.val);
    entity->enable_topic_ = true;
  }

  return entity;
}

static void _zenoh_pico_clear_entitiy_menber(ZenohPicoEntity_t *entity)
{
  if(entity->zid_.len != 0)     _z_string_clear(&entity->zid_);
  if(entity->nid_.len != 0)     _z_string_clear(&entity->nid_);
  if(entity->id_.len != 0)      _z_string_clear(&entity->id_);
  if(entity->keyexpr_.len != 0) _z_string_clear(&entity->keyexpr_);
}

void zenoh_pico_destroy_entitiy(ZenohPicoEntity_t *entity)
{
  _zenoh_pico_clear_entitiy_menber(entity);

  if(entity->enable_node_)
    zenoh_pico_destroy_node(&entity->node_info_);

  if(entity->enable_topic_)
    zenoh_pico_destroy_topic(&entity->topic_info_);

  ZenohPicoDestroyData(entity);
}

void zenoh_pico_clone_entitiy(ZenohPicoEntity_t *dst, ZenohPicoEntity_t *src)
{
  _z_string_copy(&dst->zid_, &src->zid_);
  _z_string_copy(&dst->nid_, &src->nid_);
  _z_string_copy(&dst->id_, &src->id_);
  _z_string_copy(&dst->keyexpr_, &src->keyexpr_);

  dst->type_ = src->type_;

  if(src->enable_node_)
    zenoh_pico_clone_node(&dst->node_info_, &src->node_info_);

  if(src->enable_topic_)
    zenoh_pico_clone_topic(&dst->topic_info_, &src->topic_info_);

  return;
}
