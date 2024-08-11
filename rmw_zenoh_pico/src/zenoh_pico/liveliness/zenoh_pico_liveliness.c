
#include <stdio.h>
#include <string.h>

#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_entity.h"

static const char ADMIN_SPACE[] = "@ros2_lv";
static const char NODE_STR[] = "NN";
static const char PUB_STR[] = "MP";
static const char SUB_STR[] = "MS";
static const char SRV_STR[] = "SS";
static const char CLI_STR[] = "SC";
static const char KEYEXPR_DELIMITER = '/';
static const char SLASH_REPLACEMENT = '%';
static const char QOS_DELIMITER = ':';
static const char QOS_COMPONENT_DELIMITER = ',';

static const char *conv_entity_type(ZenohPicoEntityType type)
{
  const char *ret;

  if(type == Node)
    ret = NODE_STR;
  else if(type == Publisher)
    ret = PUB_STR;
  else if(type == Subscription)
    ret = SUB_STR;
  else if(type == Service)
    ret = SRV_STR;
  else if(type == Client)
    ret = CLI_STR;

  return ret;
}

static void mangle_name(char *buf, size_t size)
{
  for(size_t count = 0; count < size; count++){
    if(*(buf +count) == '/')
      *(buf +count) = SLASH_REPLACEMENT;
  }
}

static void demangle_name(char *buf, size_t size)
{
  for(size_t count = 0; count < size; count++){
    if(*(buf +count) == SLASH_REPLACEMENT)
      *(buf +count) = '/';
  }
}

static bool add_int_value(size_t value, char **buf, int *left)
{
  if(*left <= 0)
    return false;

  // join value string
  int ret = snprintf(*buf, *left, "%ld", value);

  *buf += ret;
  *left -= ret;

  return true;
}

static bool add_string_value(const char *value, char **buf, int *left)
{
  char _worker[64];

  if(*left <= 0)
    return false;

  // convert SLASH_REPLACEMENT for worker buffer
  memset(_worker, 0, sizeof(_worker));
  strncpy(_worker, value, sizeof(_worker) -1);
  mangle_name(_worker, strlen(_worker));

  // join _worker buffer
  int ret = snprintf(*buf, *left, "%s", _worker);

  *buf += ret;
  *left -= ret;

  return true;
}

static bool add_delimiter(char **buf, int *left)
{
  if(*left <= 0)
    return false;

  // join delimiter code which is '/'
  int ret = snprintf(*buf, *left, "/");

  *buf += ret;
  *left -= ret;

  return true;
}

#define APPEND_VALUE(v, b, s)				\
  _Generic((v),						\
	   size_t : add_int_value,			\
	   const char * : add_string_value)(v, b, s)

#define APPEND_DELIMITER(b, s)  add_delimiter(b, s)

size_t generate_liveliness(ZenohPicoEntity *entity, char *buf, size_t size)
{
  int left_size = size;
  char *buf_ptr = buf;
  int ret = 0;

  memset(buf, 0, size);

  // generate part of node
  if(entity->node_info_ != NULL) {

    // append admin header
    APPEND_VALUE(ADMIN_SPACE, &buf_ptr, &left_size);

    // append domain id
    APPEND_DELIMITER(&buf_ptr, &left_size);
    APPEND_VALUE(get_node_domain(entity), &buf_ptr, &left_size);

    // append zid
    APPEND_DELIMITER(&buf_ptr, &left_size);
    const char *_zid = Z_STRING_VAL(entity->zid_);
    APPEND_VALUE(_zid, &buf_ptr, &left_size);

    // append nid
    APPEND_DELIMITER(&buf_ptr, &left_size);
    APPEND_VALUE(get_nid(entity), &buf_ptr, &left_size);

    // append id
    APPEND_DELIMITER(&buf_ptr, &left_size);
    APPEND_VALUE(get_id(entity), &buf_ptr, &left_size);

    // append type
    APPEND_DELIMITER(&buf_ptr, &left_size);
    APPEND_VALUE(conv_entity_type(entity->type_), &buf_ptr, &left_size);

    // append node_enclave
    APPEND_DELIMITER(&buf_ptr, &left_size);
    APPEND_VALUE(get_node_enclave(entity), &buf_ptr, &left_size);

    // append node_namespace
    APPEND_DELIMITER(&buf_ptr, &left_size);
    APPEND_VALUE(get_node_namespace(entity), &buf_ptr, &left_size);

    // append node_name
    APPEND_DELIMITER(&buf_ptr, &left_size);
    APPEND_VALUE(get_node_name(entity), &buf_ptr, &left_size);

    if(entity->topic_info_ != NULL) {

      // append topic_name
      APPEND_DELIMITER(&buf_ptr, &left_size);
      APPEND_VALUE(get_topic_name(entity), &buf_ptr, &left_size);

      // append topic_type
      APPEND_DELIMITER(&buf_ptr, &left_size);
      APPEND_VALUE(get_topic_type(entity), &buf_ptr, &left_size);

      // append topic_hash
      APPEND_DELIMITER(&buf_ptr, &left_size);
      APPEND_VALUE(get_topic_hash(entity), &buf_ptr, &left_size);

      // append topic_qos(
      APPEND_DELIMITER(&buf_ptr, &left_size);
      APPEND_VALUE(get_topic_qos(entity), &buf_ptr, &left_size);
    }
  }

  if(left_size < 0)
    return -1;

  return ret;
}
