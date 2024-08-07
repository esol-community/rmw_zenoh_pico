#include <stdio.h>
#include <string.h>

#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_topic.h"

const _z_string_t *topic_name(ZenohPicoTopicInfo_t *topic)	{ return &topic->name_; }
const _z_string_t *topic_type(ZenohPicoTopicInfo_t *topic)	{ return &topic->type_; }
const _z_string_t *topic_typehash(ZenohPicoTopicInfo_t *topic)	{ return &topic->typehash_; }

static _z_string_t zenoh_pico_generate_typehash(ZenohPicoTopicInfo_t *topic){
  return _z_string_make("");
}

ZenohPicoTopicInfo_t *zenoh_pico_generate_topic(ZenohPicoTopicInfo_t *topic,
				     const char *name,
				     const char *type,
				     const char *typehash)
{
  ZenohPicoGenerateData(topic, ZenohPicoTopicInfo_t);
  if (topic == NULL) {
    return NULL;
  }

  topic->name_ = (name == NULL) ? _z_string_make("") : _z_string_make(name);
  topic->type_ = (type == NULL) ? _z_string_make("") : _z_string_make(type);

  if(typehash == NULL) {
    topic->typehash_ = zenoh_pico_generate_typehash(topic);
  }else{
    topic->typehash_ = _z_string_make(typehash);
  }

  return topic;
}

static void _zenoh_pico_clear_topic_member(ZenohPicoTopicInfo_t *topic)
{
  if (topic->name_.len != 0)	 _z_string_clear(&topic->name_);
  if (topic->type_.len != 0)	 _z_string_clear(&topic->type_);
  if (topic->typehash_.len != 0) _z_string_clear(&topic->typehash_);
}

bool zenoh_pico_destroy_topic(ZenohPicoTopicInfo_t *topic)
{
  _zenoh_pico_clear_topic_member(topic);

  ZenohPicoDestroyData(topic);

  return true;
}

bool zenoh_pico_clone_topic(ZenohPicoTopicInfo_t *dst, ZenohPicoTopicInfo_t *src)
{
  _zenoh_pico_clear_topic_member(dst);

  _z_string_copy(&dst->name_, &src->name_);
  _z_string_copy(&dst->type_, &src->type_);
  _z_string_copy(&dst->typehash_, &src->typehash_);

  return true;
}

void zenoh_pico_debug_topic(ZenohPicoTopicInfo_t *topic)
{
  printf("topic info ...\n");

  printf("\tname     = %s\n", topic->name_.val);
  printf("\ttype     = %s\n", topic->type_.val);
  printf("\ttypehash = %s\n", topic->typehash_.val);
}
