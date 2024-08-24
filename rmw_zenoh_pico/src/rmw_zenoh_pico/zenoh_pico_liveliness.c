
#include <stdio.h>
#include <string.h>

#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_entity.h"
#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_liveliness.h"
#include "zenoh-pico/api/primitives.h"
#include "zenoh-pico/collections/string.h"

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

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

z_string_t generate_liveliness(ZenohPicoEntity *entity)
{
  char buf[RMW_ZENOH_PICO_MAX_LINENESS_LEN];
  int left_size = sizeof(buf);
  char *buf_ptr = buf;

  memset(buf, 0, sizeof(buf));

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

  return z_string_make(buf);
}

z_string_t conv_domain(size_t domain){
  char _domain[16];

  memset(_domain, 0, sizeof(_domain));
  snprintf(_domain, sizeof(_domain), "%ld", domain);

  return _z_string_make(_domain);
}

static const char RIHS01_PREFIX[]	= "RIHS01_";
static const size_t RIHS_VERSION_IDX	= 4;
static const size_t RIHS_PREFIX_LEN	= 7;
static const size_t RIHS01_STRING_LEN	= 71;  // RIHS_PREFIX_LEN + (ROSIDL_TYPE_HASH_SIZE * 2);
static const uint8_t INVALID_NIBBLE	= 0xff;

z_string_t convert_hash(const rosidl_type_hash_t * type_hash)
{
  char _hash_data[RIHS01_STRING_LEN +1];

  memset(_hash_data, 0, sizeof(_hash_data));
  memcpy(_hash_data, RIHS01_PREFIX, RIHS_PREFIX_LEN);
  uint8_t nibble = 0;
  char * dest = NULL;
  for (size_t i = 0; i < ROSIDL_TYPE_HASH_SIZE; i++) {
    // Translate byte into two hex characters
    dest = _hash_data + RIHS_PREFIX_LEN + (i * 2);
    // First character is top half of byte
    nibble = (type_hash->value[i] >> 4) & 0x0f;
    if (nibble < 0xa) {
      dest[0] = '0' + nibble;
    } else {
      dest[0] = 'a' + (nibble - 0xa);
    }
    // Second character is bottom half of byte
    nibble = (type_hash->value[i] >> 0) & 0x0f;
    if (nibble < 0xa) {
      dest[1] = '0' + nibble;
    } else {
      dest[1] = 'a' + (nibble - 0xa);
    }
  }

  return _z_string_make(_hash_data);
}

z_string_t convert_message_type(const message_type_support_callbacks_t *callbacks)
{
  char _type_name[64];
  if(callbacks->message_name_ != NULL)
    snprintf(_type_name, sizeof(_type_name), "%s::dds_::%s_",
	     callbacks->message_namespace_,
	     callbacks->message_name_);
  else
    snprintf(_type_name, sizeof(_type_name), "dds_::%s_",
	     callbacks->message_name_);

  return _z_string_make(_type_name);
}

z_string_t qos_to_keyexpr(rmw_qos_profile_t *qos)
{
  char qos_data[64];

  memset(qos_data, 0, sizeof(qos_data));
  snprintf(qos_data, sizeof(qos_data),
	   "%d%c%d%c%d%c%ld%c%ld%c%ld%c%ld%c%ld%c%d%c%ld%c%ld",
	   qos->reliability, QOS_DELIMITER,
	   qos->durability,  QOS_DELIMITER,
	   qos->history, QOS_COMPONENT_DELIMITER,
	   qos->depth, QOS_DELIMITER,
	   qos->deadline.sec, QOS_COMPONENT_DELIMITER,
	   qos->deadline.nsec, QOS_DELIMITER,
	   qos->lifespan.sec, QOS_COMPONENT_DELIMITER,
	   qos->lifespan.nsec, QOS_DELIMITER,
	   qos->liveliness, QOS_COMPONENT_DELIMITER,
	   qos->liveliness_lease_duration.sec, QOS_COMPONENT_DELIMITER,
	   qos->liveliness_lease_duration.nsec);

  return _z_string_make(qos_data);
}
