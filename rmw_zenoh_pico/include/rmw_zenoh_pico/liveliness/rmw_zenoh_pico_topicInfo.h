#ifndef RMW_ZENOH_PICO_TOPICINFO_H
#define RMW_ZENOH_PICO_TOPICINFO_H

#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct ZenohPicoTopicInfo_s
  {
    bool is_alloc_;

    _z_string_t name_;
    _z_string_t type_;
    _z_string_t hash_;
    _z_string_t qos_;
  } ZenohPicoTopicInfo_t;

  extern const char *topic_name(ZenohPicoTopicInfo_t *topic);
  extern const char *topic_type(ZenohPicoTopicInfo_t *topic);
  extern const char *topic_hash(ZenohPicoTopicInfo_t *topic);
  extern const char *topic_qos(ZenohPicoTopicInfo_t *topic);

  extern ZenohPicoTopicInfo_t *zenoh_pico_generate_topic_info(ZenohPicoTopicInfo_t *topic,
					      const char *name,
					      const char *type,
					      const char *typehash);
  extern bool zenoh_pico_destroy_topic_info(ZenohPicoTopicInfo_t *topic);

  extern bool zenoh_pico_clone_topic_info(ZenohPicoTopicInfo_t *dst, ZenohPicoTopicInfo_t *src);
  extern void zenoh_pico_debug_topic_info(ZenohPicoTopicInfo_t *topic);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
