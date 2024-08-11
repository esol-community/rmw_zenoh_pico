#ifndef RMW_ZENOH_PICO_NODEINFO_H
#define RMW_ZENOH_PICO_NODEINFO_H

#include <stddef.h>
#include <unistd.h>

#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoNodeInfo
  {
    bool is_alloc_;

    _z_string_t domain_id_;
    _z_string_t ns_;
    _z_string_t name_;
    _z_string_t enclave_;

  } ZenohPicoNodeInfo_t;

  extern const _z_string_t *node_domain(ZenohPicoNodeInfo_t *node);
  extern const _z_string_t *node_namespace(ZenohPicoNodeInfo_t *node);
  extern const _z_string_t *node_name(ZenohPicoNodeInfo_t *node);
  extern const _z_string_t *node_enclave(ZenohPicoNodeInfo_t *node);

  extern ZenohPicoNodeInfo_t *zenoh_pico_generate_node_info(ZenohPicoNodeInfo_t *node,
							    const char *domain_id,
							    const char *ns,
							    const char *name,
							    const char *enclave);

  extern bool zenoh_pico_destroy_node_info(ZenohPicoNodeInfo_t *node);

  extern void zenoh_pico_clone_node_info(ZenohPicoNodeInfo_t *dst, ZenohPicoNodeInfo_t *src);
  extern void zenoh_pico_debug_node_info(ZenohPicoNodeInfo_t *node);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif