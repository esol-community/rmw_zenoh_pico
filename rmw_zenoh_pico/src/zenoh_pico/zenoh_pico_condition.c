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
int8_t z_condvar_wait_time(z_loaned_condvar_t *cv, z_loaned_mutex_t *m, struct timespec *wait_timeout){
  struct timespec abstime;

  memset(&abstime, 0, sizeof(abstime));
  time(&abstime.tv_sec);
  abstime.tv_sec += wait_timeout->tv_sec;
  abstime.tv_nsec += wait_timeout->tv_nsec;

  // RMW_ZENOH_LOG_INFO("%s : wait_set_data->wait_timeout = [%ld : %ld]",
  // 	   __func__,
  // 	   abstime.tv_sec,
  // 	   abstime.tv_nsec);

  return pthread_cond_timedwait(cv, m, &abstime);
}
#else
#include "zenoh-pico/system/platform/void.h"
#error "Unknown platform"
#endif
