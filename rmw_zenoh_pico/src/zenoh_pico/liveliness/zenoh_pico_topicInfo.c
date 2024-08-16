#include <stdio.h>
#include <string.h>

#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_topicInfo.h"
#include "zenoh-pico/collections/string.h"

const char *topic_name(ZenohPicoTopicInfo_t *topic)	{ return Z_STRING_VAL(topic->name_); }
const char *topic_type(ZenohPicoTopicInfo_t *topic)	{ return Z_STRING_VAL(topic->type_); }
const char *topic_hash(ZenohPicoTopicInfo_t *topic)	{ return Z_STRING_VAL(topic->hash_); }
const char *topic_qos(ZenohPicoTopicInfo_t *topic)	{ return Z_STRING_VAL(topic->qos_); }

static void _zenoh_pico_clear_topic_info_member(ZenohPicoTopicInfo_t *topic)
{
  Z_STRING_FREE(topic->name_);
  Z_STRING_FREE(topic->type_);
  Z_STRING_FREE(topic->hash_);
  Z_STRING_FREE(topic->qos_);
}

ZenohPicoTopicInfo_t *zenoh_pico_generate_topic_info(ZenohPicoTopicInfo_t *topic,
						     z_string_t *name,
						     z_string_t *type,
						     z_string_t *hash,
						     z_string_t *qos)
{
  ZenohPicoGenerateData(topic, ZenohPicoTopicInfo_t);
  if (topic == NULL) {
    return NULL;
  }

  _zenoh_pico_clear_topic_info_member(topic);

  _z_string_move(&topic->name_, name);
  _z_string_move(&topic->type_, type);
  _z_string_move(&topic->hash_, hash);
  _z_string_move(&topic->qos_, qos);

  return topic;
}

bool zenoh_pico_destroy_topic_info(ZenohPicoTopicInfo_t *topic)
{
  _zenoh_pico_clear_topic_info_member(topic);

  ZenohPicoDestroyData(topic);

  return true;
}

bool zenoh_pico_clone_topic_info(ZenohPicoTopicInfo_t *dst, ZenohPicoTopicInfo_t *src)
{
  _zenoh_pico_clear_topic_info_member(dst);

  _z_string_copy(&dst->name_, &src->name_);
  _z_string_copy(&dst->type_, &src->type_);
  _z_string_copy(&dst->hash_, &src->hash_);
  _z_string_copy(&dst->qos_, &src->qos_);

  return true;
}

void zenoh_pico_debug_topic_info(ZenohPicoTopicInfo_t *topic)
{
  printf("topic info ...\n");

  printf("\tname = %s\n", topic->name_.val);
  printf("\ttype = %s\n", topic->type_.val);
  printf("\thash = %s\n", topic->hash_.val);
  printf("\tqos  = %s\n", topic->qos_.val);
}
