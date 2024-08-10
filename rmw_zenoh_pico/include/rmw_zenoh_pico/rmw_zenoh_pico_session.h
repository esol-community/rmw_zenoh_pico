#ifndef RMW_ZENOH_PICO_SESSION_H
#define RMW_ZENOH_PICO_SESSION_H

#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoSession
  {
    bool alloc_;

    // Enclave, name used to find security artifacts in a sros2 keystore
    z_string_t z_enclave_;

    // configuration data array
    z_owned_config_t z_config_;

    // An owned session.
    z_owned_session_t z_session_;

  } ZenohPicoSession;

  extern ZenohPicoSession *zenoh_pico_generate_session(ZenohPicoSession *session,
						       z_owned_config_t *z_config,
						       z_owned_session_t *z_session,
						       const char *enclave);

  extern bool zenoh_pico_destroy_session(ZenohPicoSession *session);

  extern bool zenoh_pico_clone_session(ZenohPicoSession *dst, ZenohPicoSession *src);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
