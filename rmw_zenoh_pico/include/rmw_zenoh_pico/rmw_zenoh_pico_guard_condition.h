#ifndef RMW_ZENOH_PICO_GUARD_CONDITION_H
#define RMW_ZENOH_PICO_GUARD_CONDITION_H

#include <rmw/rmw.h>
#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

#include <rmw_zenoh_pico/rmw_zenoh_pico_wait.h>

  typedef struct _ZenohPicoGuardConditionData {
    z_mutex_t internal_mutex_;
    bool triggered_;
    ZenohPicoWaitSetData * wait_set_data_;

  } ZenohPicoGuardConditionData;

  extern ZenohPicoGuardConditionData * zenoh_pico_guard_condition_data(void);
  extern bool zenoh_pico_destroy_guard_condition_data(ZenohPicoGuardConditionData *condition_data);

  // ---------

  extern void condition_trigger(ZenohPicoGuardConditionData *condition_data);
  extern bool condition_check(ZenohPicoGuardConditionData *condition_data,
			      ZenohPicoWaitSetData * wait_set_data);
  extern bool condition_detach(ZenohPicoGuardConditionData *condition_data);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
