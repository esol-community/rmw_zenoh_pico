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

#include <stdio.h>
#include <string.h>

#include "rmw_zenoh_pico/rmw_zenoh_pico_logging.h"
#include "zenoh-pico/api/primitives.h"
#include "zenoh-pico/api/types.h"
#include "zenoh-pico/utils/result.h"

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
  int ret = snprintf(*buf, *left, "%d", (int)value);

  *buf += ret;
  *left -= ret;

  return true;
}

#define STRING_MAX_LEN 128
static bool add_string_value(const char *value, char **buf, int *left)
{
  char _worker[STRING_MAX_LEN];

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

static bool add_loan_string_value(
  const z_loaned_string_t *value,
  char **buf, int *left)
{
  char _worker[STRING_MAX_LEN];

  if(*left <= 0)
    return false;

  size_t _len = z_string_len(value) > sizeof(_worker) -1 ? sizeof(_worker) -1 : z_string_len(value);

  memset(_worker, 0, sizeof(_worker));
  memcpy(_worker, z_string_data(value), _len);
  mangle_name(_worker, _len);

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

#define APPEND_VALUE(v)						\
  _Generic((v),							\
	   size_t : add_int_value,				\
	   const char * : add_string_value,			\
	   const z_loaned_string_t *: add_loan_string_value	\
    )(v, &buf_ptr, &left_size)

#define APPEND_DELIMITER()  add_delimiter(&buf_ptr, &left_size)

z_result_t generate_liveliness(ZenohPicoEntity *entity, z_owned_string_t *value)
{
  char buf[RMW_ZENOH_PICO_MAX_LINENESS_LEN];
  int left_size = sizeof(buf) -1;
  char *buf_ptr = buf;

  memset(buf, 0, sizeof(buf));

  // generate part of node
  if(entity->node_info != NULL) {

    // append admin header
    APPEND_VALUE(ADMIN_SPACE);

    // append domain id
    APPEND_DELIMITER();
    APPEND_VALUE(get_node_domain(entity));

    // append zid
    APPEND_DELIMITER();
    APPEND_VALUE(get_zid(entity));

    // append nid
    APPEND_DELIMITER();
    APPEND_VALUE(get_nid(entity));

    // append id
    APPEND_DELIMITER();
    APPEND_VALUE(get_id(entity));

    // append type
    APPEND_DELIMITER();
    APPEND_VALUE(conv_entity_type(entity->type));

    // append node_enclave
    APPEND_DELIMITER();
    APPEND_VALUE(get_node_enclave(entity));

    // append node_namespace
    APPEND_DELIMITER();
    APPEND_VALUE(get_node_namespace(entity));

    // append node_name
    APPEND_DELIMITER();
    APPEND_VALUE(get_node_name(entity));

    if(entity->topic_info != NULL) {

      // append topic_name
      APPEND_DELIMITER();
      APPEND_VALUE(get_topic_name(entity));

      // append topic_type
      APPEND_DELIMITER();
      APPEND_VALUE(get_topic_type(entity));

      // append topic_hash
      APPEND_DELIMITER();
      APPEND_VALUE(get_topic_hash(entity));

      // append topic_qos(
      APPEND_DELIMITER();
      APPEND_VALUE(get_topic_qos(entity));
    }
  }

  return z_string_copy_from_str(value, buf);
}

z_result_t conv_domain(size_t domain, z_owned_string_t *value){
  char _domain[16];

  memset(_domain, 0, sizeof(_domain));
  snprintf(_domain, sizeof(_domain), "%d", (int)domain);

  return z_string_copy_from_str(value, _domain);
}

#define RIHS01_PREFIX 	   "RIHS01_"
#define RIHS_VERSION_IDX   4
#define RIHS_PREFIX_LEN	   7
#define RIHS01_STRING_LEN  71  // RIHS_PREFIX_LEN + (ROSIDL_TYPE_HASH_SIZE * 2);
#define INVALID_NIBBLE	   0xff

z_result_t convert_hash(const rosidl_type_hash_t * type_hash, z_owned_string_t *value)
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

  z_result_t ret = z_string_copy_from_str(value, _hash_data);
  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    RMW_ZENOH_LOG_INFO("hash = [%*s][%d]",
		       Z_STRING_LEN(*value),
		       Z_STRING_VAL(*value),
		       Z_STRING_LEN(*value));
  }

  return ret;
}

#define TYPE_NAME_LEN 128
static z_result_t _convert_message_type(const char *message_namespace,
					const char *message_name,
					z_owned_string_t *value)
{
  char _type_name[TYPE_NAME_LEN];

  if(message_name != NULL)
    snprintf(_type_name, sizeof(_type_name), "%s::dds_::%s_",
	     message_namespace,
	     message_name);
  else
    snprintf(_type_name, sizeof(_type_name), "dds_::%s_",
	     message_name);

  z_result_t ret = z_string_copy_from_str(value, _type_name);

  if(rmw_zenoh_pico_debug_level_get() == _Z_LOG_LVL_DEBUG){
    RMW_ZENOH_LOG_INFO("type_name = [%.*s][%d]",
		       Z_STRING_LEN(*value),
		       Z_STRING_VAL(*value),
		       Z_STRING_LEN(*value));
  }

  return ret;
}

z_result_t convert_message_type(const message_type_support_callbacks_t *callbacks,
				z_owned_string_t *value)
{
  return _convert_message_type(callbacks->message_namespace_,
			       callbacks->message_name_,
			       value);
}

z_result_t convert_client_type(const message_type_support_callbacks_t *callbacks,
			       z_owned_string_t *value)
{
  const char *suffix_position = strstr(callbacks->message_name_, "_Request");
  if(suffix_position == NULL){
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("Unexpected type %s for client %s. Report this bug",
					 callbacks->message_name_,
					 callbacks->message_name_);
    return _Z_ERR_GENERIC;
  }

  char service_type[128];
  memset(service_type, 0, sizeof(service_type));
  memcpy(service_type, callbacks->message_name_, suffix_position - callbacks->message_name_);

  return _convert_message_type(callbacks->message_namespace_,
			       service_type,
			       value);
}

static inline bool qos_append_value(int value, char **buf, size_t *left)
{
  if(*left <= 0)
    return false;

  if(value != 0){
    // join value string
    int ret = snprintf(*buf, *left, "%d", (int)value);

    *buf += ret;
    *left -= ret;
  }

  return true;
}

static inline bool qos_add_delimiter(char delimiter, char **buf, size_t *left)
{
  if(*left <= 0)
    return false;

  **buf = delimiter;
  *buf += 1;
  *left -= 1;

  return true;
}

#define QOS_APPEND_VALUE(v) qos_append_value(v, &buf_ptr, &left_size)
#define QOS_APPEND_DELIMITER(v) qos_add_delimiter(v, &buf_ptr, &left_size)

z_result_t qos_to_keyexpr(rmw_qos_profile_t *qos, z_owned_string_t *value)
{
  char qos_data[64];

  memset(qos_data, 0, sizeof(qos_data));
  char *buf_ptr = qos_data;
  size_t left_size = sizeof(qos_data) -1;

  if(qos != NULL){

    QOS_APPEND_VALUE(qos->reliability);
    QOS_APPEND_DELIMITER(QOS_DELIMITER);

    QOS_APPEND_VALUE(qos->durability);
    QOS_APPEND_DELIMITER(QOS_DELIMITER);

    QOS_APPEND_VALUE((int)qos->history);
    QOS_APPEND_DELIMITER(QOS_COMPONENT_DELIMITER);

    QOS_APPEND_VALUE((int)qos->depth);
    QOS_APPEND_DELIMITER(QOS_DELIMITER);

    QOS_APPEND_VALUE((int)qos->deadline.sec);
    QOS_APPEND_DELIMITER(QOS_COMPONENT_DELIMITER);

    QOS_APPEND_VALUE((int)qos->deadline.nsec);
    QOS_APPEND_DELIMITER(QOS_DELIMITER);

    QOS_APPEND_VALUE((int)qos->lifespan.sec);
    QOS_APPEND_DELIMITER(QOS_COMPONENT_DELIMITER);

    QOS_APPEND_VALUE((int)qos->lifespan.nsec);
    QOS_APPEND_DELIMITER(QOS_DELIMITER);

    QOS_APPEND_VALUE(qos->liveliness);
    QOS_APPEND_DELIMITER(QOS_COMPONENT_DELIMITER);

    QOS_APPEND_VALUE((int)qos->liveliness_lease_duration.sec);
    QOS_APPEND_DELIMITER(QOS_COMPONENT_DELIMITER);

    QOS_APPEND_VALUE((int)qos->liveliness_lease_duration.nsec);
  }

  return z_string_copy_from_str(value, qos_data);
}
