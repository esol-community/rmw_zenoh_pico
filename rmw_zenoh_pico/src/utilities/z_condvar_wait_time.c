
#include <errno.h>
#include <string.h>
#include <limits.h>
#include <time.h>

#include <rmw/rmw.h>
#include <rmw/time.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

#if defined(ZENOH_LINUX)
int8_t z_condvar_wait_time(z_condvar_t *cv, z_mutex_t *m, struct timespec *wait_timeout){
  struct timespec abstime;

  memset(&abstime, 0, sizeof(abstime));
  time(&abstime.tv_sec);
  abstime.tv_sec += wait_timeout->tv_sec;
  abstime.tv_nsec += wait_timeout->tv_nsec;

  // _Z_DEBUG("%s : wait_set_data->wait_timeout = [%ld : %ld]",
  // 	   __func__,
  // 	   abstime.tv_sec,
  // 	   abstime.tv_nsec);

  return pthread_cond_timedwait(cv, m, &abstime);
}
#else
#include "zenoh-pico/system/platform/void.h"
#error "Unknown platform"
#endif
