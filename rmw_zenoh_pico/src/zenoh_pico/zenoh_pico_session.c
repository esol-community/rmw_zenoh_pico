#include <stdio.h>
#include <string.h>

#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_session.h"
#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

#include "zenoh-pico/api/macros.h"
#include "zenoh-pico/api/primitives.h"

ZenohPicoSession *zenoh_pico_generate_session(ZenohPicoSession *session,
					      z_owned_config_t *config,
					      const char *enclave)
{
  ZenohPicoGenerateData(session, ZenohPicoSession);
  if(session == NULL){
    return NULL;
  }

  session->config_ = *z_move(*config);

  if(enclave != NULL)
    session->enclave_ = z_string_make(enclave);

  session->graph_guard_condition.implementation_identifier = zenoh_pico_identifier;
  session->graph_guard_condition.data = NULL;

  return session;
}

bool zenoh_pico_destroy_session(ZenohPicoSession *session)
{
  z_drop(z_move(session->config_));
  z_drop(z_move(session->session_));

  Z_STRING_FREE(session->enclave_);

  ZenohPicoDestroyData(session);

  return true;
}

bool zenoh_pico_clone_session(ZenohPicoSession *dst, ZenohPicoSession *src)
{
  z_drop(z_move(dst->config_));
  z_drop(z_move(dst->session_));

  dst->config_ = z_clone(src->config_);
  dst->session_ = z_clone(src->session_);

  return true;
}

bool session_connect(ZenohPicoSession *session)
{
  z_owned_config_t *config = &session->config_;

  _Z_INFO("Opening session...");
  session->session_ = z_open(z_config_move(config));
  if (!z_session_check(&session->session_)) {
    RMW_SET_ERROR_MSG("Error setting up zenoh session");
    return RMW_RET_ERROR;
  }

  if (zp_start_read_task(z_loan(session->session_), NULL) < 0
      || zp_start_lease_task(z_loan(session->session_), NULL) < 0) {
    RMW_SET_ERROR_MSG("Unable to start read and lease tasks");
    z_drop(z_config_move(config));
    z_close(z_session_move(&session->session_));
    return RMW_RET_ERROR;
  }

}
