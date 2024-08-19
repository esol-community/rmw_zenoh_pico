#ifndef RMW_ZENOH_PICO_WAIT_H
#define RMW_ZENOH_PICO_WAIT_H

#include <rmw/rmw.h>
#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _ZenohPicoWaitSetData {
    uint ref_;

    z_condvar_t condition_variable;
    z_mutex_t condition_mutex;

    bool triggered;
    rmw_context_t * context;

  } ZenohPicoWaitSetData;

  extern ZenohPicoWaitSetData * zenoh_pico_generate_wait_data(rmw_context_t * context);
  extern bool zenoh_pico_destroy_wait_data(ZenohPicoWaitSetData *wait_data);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)


#endif
