#ifndef RMW_ZENOH_PICO_TOPIC_H
#define RMW_ZENOH_PICO_TOPIC_H

#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct ZenohPicoTopicInfo_s
  {
    bool alloc_;

    _z_string_t name_;
    _z_string_t type_;
    _z_string_t typehash_;
  } ZenohPicoTopicInfo_t;

  extern const _z_string_t *topic_name(ZenohPicoTopicInfo_t *topic);
  extern const _z_string_t *topic_type(ZenohPicoTopicInfo_t *topic);
  extern const _z_string_t *topic_typehash(ZenohPicoTopicInfo_t *topic);

  extern ZenohPicoTopicInfo_t *zenoh_pico_generate_topic(ZenohPicoTopicInfo_t *topic,
					      const char *name,
					      const char *type,
					      const char *typehash);
  extern bool zenoh_pico_destroy_topic(ZenohPicoTopicInfo_t *topic);

  extern bool zenoh_pico_clone_topic(ZenohPicoTopicInfo_t *dst, ZenohPicoTopicInfo_t *src);
  extern void zenoh_pico_debug_topic(ZenohPicoTopicInfo_t *topic);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
