#ifndef RMW_ZENOH_PICO_TOPICINFO_H
#define RMW_ZENOH_PICO_TOPICINFO_H

#include "zenoh-pico/api/types.h"
#include "zenoh-pico/collections/string.h"
#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoTopicInfo
  {
    int ref_;

    _z_string_t name_;
    _z_string_t type_;
    _z_string_t hash_;
    _z_string_t qos_;
  } ZenohPicoTopicInfo;

  extern const char *topic_name(ZenohPicoTopicInfo *topic);
  extern const char *topic_type(ZenohPicoTopicInfo *topic);
  extern const char *topic_hash(ZenohPicoTopicInfo *topic);
  extern const char *topic_qos(ZenohPicoTopicInfo *topic);

  extern ZenohPicoTopicInfo *zenoh_pico_generate_topic_info(z_string_t *name,
							    z_string_t *type,
							    z_string_t *typehash,
							    z_string_t *qos);
  extern bool zenoh_pico_destroy_topic_info(ZenohPicoTopicInfo *topic);

  extern void zenoh_pico_debug_topic_info(ZenohPicoTopicInfo *topic);

  // -------

  extern z_string_t ros_topic_name_to_zenoh_key(const char * domain,
						const char * name,
						const char * type,
						const char * hash);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
