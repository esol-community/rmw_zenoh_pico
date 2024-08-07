#include <stdio.h>
#include <string.h>

#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_session.h"

#include "zenoh-pico/api/macros.h"
#include "zenoh-pico/api/primitives.h"
#include "zenoh-pico/collections/string.h"

ZenohPicoSession *zenoh_pico_generate_session(ZenohPicoSession *session,
					      z_owned_config_t *z_config,
					      z_owned_session_t *z_session,
					      const char *enclave)
{
  ZenohPicoGenerateData(session, ZenohPicoSession);
  if(session == NULL){
    return NULL;
  }

  session->z_config_  = *z_move(*z_config);
  session->z_session_ = *z_move(*z_session);

  if(enclave != NULL)
    session->z_enclave_ = z_string_make(enclave);

  return session;
}

bool zenoh_pico_destroy_session(ZenohPicoSession *session)
{
  z_drop(z_move(session->z_config_));
  z_drop(z_move(session->z_session_));

  if(session->z_enclave_.len != 0)
    _z_string_clear(&session->z_enclave_);

  ZenohPicoDestroyData(session);

  return true;
}

bool zenoh_pico_clone_session(ZenohPicoSession *dst, ZenohPicoSession *src)
{
  z_drop(z_move(dst->z_config_));
  z_drop(z_move(dst->z_session_));

  dst->z_config_ = z_clone(src->z_config_);
  dst->z_session_ = z_clone(src->z_session_);

  return true;
}
