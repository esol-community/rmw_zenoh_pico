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
    int ref_;

    z_string_t domain_;
    z_string_t ns_;
    z_string_t name_;
    z_string_t enclave_;

  } ZenohPicoNodeInfo_t;

  extern const char *node_domain(ZenohPicoNodeInfo_t *node);
  extern const char *node_enclave(ZenohPicoNodeInfo_t *node);
  extern const char *node_namespace(ZenohPicoNodeInfo_t *node);
  extern const char *node_name(ZenohPicoNodeInfo_t *node);

  extern ZenohPicoNodeInfo_t *zenoh_pico_generate_node_info(z_string_t *domain,
							    z_string_t *ns,
							    z_string_t *name,
							    z_string_t *enclave);
  extern ZenohPicoNodeInfo_t *zenoh_pico_clone_node_info(ZenohPicoNodeInfo_t *node_info);
  extern bool zenoh_pico_destroy_node_info(ZenohPicoNodeInfo_t *node);

  extern void zenoh_pico_debug_node_info(ZenohPicoNodeInfo_t *node);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
