#include <rmw_zenoh_pico/config.h>

#include <stdio.h>
#include <string.h>

#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_topicInfo.h"
#include "zenoh-pico/api/primitives.h"

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

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

ZenohPicoTopicInfo_t *zenoh_pico_generate_topic_info(z_string_t *name,
						     z_string_t *type,
						     z_string_t *hash,
						     z_string_t *qos)
{
  ZenohPicoTopicInfo_t *topic = NULL;
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

void zenoh_pico_debug_topic_info(ZenohPicoTopicInfo_t *topic)
{
  printf("topic info ...\n");

  printf("\tname = %s\n", topic->name_.val);
  printf("\ttype = %s\n", topic->type_.val);
  printf("\thash = %s\n", topic->hash_.val);
  printf("\tqos  = %s\n", topic->qos_.val);
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

  _Z_INFO("new z_keyexpr is [%s]\n", keyexpr_str);

  return z_string_make(keyexpr_str);
}
