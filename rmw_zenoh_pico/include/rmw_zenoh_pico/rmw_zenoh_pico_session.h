#ifndef RMW_ZENOH_PICO_SESSION_H
#define RMW_ZENOH_PICO_SESSION_H

#include <rmw/rmw.h>
#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoSession
  {
    uint ref_;

    // Enclave, name used to find security artifacts in a sros2 keystore
    z_string_t enclave_;

    // configuration data array
    z_owned_config_t config_;

    // An owned session.
    z_owned_session_t session_;

    // graph infomation
    rmw_guard_condition_t graph_guard_condition;

  } ZenohPicoSession;

  extern ZenohPicoSession *zenoh_pico_generate_session(ZenohPicoSession *session,
						       z_owned_config_t *config,
						       const char *enclave);

  extern bool zenoh_pico_destroy_session(ZenohPicoSession *session);

  extern bool zenoh_pico_clone_session(ZenohPicoSession *dst, ZenohPicoSession *src);

  // --------------------------

  extern rmw_ret_t session_connect(ZenohPicoSession *session);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
