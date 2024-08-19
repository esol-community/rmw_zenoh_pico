#ifndef RMW_ZENOH_PICO_WAIT_H
#define RMW_ZENOH_PICO_WAIT_H

#include <rmw/rmw.h>
#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoWaitData {
    uint ref_;

    z_condvar_t condition_variable;
    z_mutex_t condition_mutex;

    bool triggered;
    rmw_context_t * context;

  } ZenohPicoWaitData;

  extern ZenohPicoWaitData * zenoh_pico_generate_wait_data(rmw_context_t * context);
  extern bool zenoh_pico_destroy_wait_data(ZenohPicoWaitData *wait_data);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)


#endif
