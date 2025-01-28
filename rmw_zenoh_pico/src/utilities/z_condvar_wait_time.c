/*
 * Copyright(C) 2024 eSOL Co., Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

#if defined(ZENOH_LINUX)
int8_t z_condvar_wait_time(z_condvar_t *cv, z_mutex_t *m, struct timespec *wait_timeout){
  struct timespec abstime;

  memset(&abstime, 0, sizeof(abstime));
  clock_gettime(CLOCK_REALTIME, &abstime);

  uint64_t _nsec_time = abstime.tv_nsec + wait_timeout->tv_nsec;
  abstime.tv_sec += wait_timeout->tv_sec + (_nsec_time/1000000000);
  abstime.tv_nsec = _nsec_time % 1000000000;

  // RMW_ZENOH_LOG_DEBUG("wait_timeout[%ld : %ld] = [%ld : %ld]",
  //		      wait_timeout->tv_sec,
  //		      wait_timeout->tv_nsec,
  //		      abstime.tv_sec,
  //		      abstime.tv_nsec);

  int ret = pthread_cond_timedwait(cv, m, &abstime);

  // RMW_ZENOH_LOG_DEBUG("pthread_cond_timedwait() return is %s", strerror(ret));

  return ret;
}
#else
#include "zenoh-pico/system/platform/void.h"
#error "Unknown platform"
#endif
