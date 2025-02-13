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

#include <rmw_zenoh_pico/rmw_zenoh_pico_rosMessage.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_wait.h>

#ifndef RMW_ZENOH_PICO_CONDITION_H
#define RMW_ZENOH_PICO_CONDITION_H

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _zenoh_pico_wait_condition {
    z_loaned_mutex_t *condition_mutex;
    ReceiveMessageDataList *msg_queue;
    ZenohPicoWaitSetData **wait_set_data_ptr;
  } ZenohPicoWaitCondition;

  extern z_result_t z_condvar_init_with_attr(z_owned_condvar_t *cv,
					     pthread_condattr_t *attr);
  extern z_result_t z_condvar_timewait(z_loaned_condvar_t *cv,
				       z_loaned_mutex_t *mp,
				       struct timespec *wait_timeout);

  // -------------

  extern void zenoh_pico_condition_trigger(ZenohPicoWaitCondition *cond);
  extern bool zenoh_pico_condition_check_and_attach(ZenohPicoWaitCondition *cond,
						    ZenohPicoWaitSetData *wait_set_data);
  extern bool zenoh_pico_condition_detach_and_queue_is_empty(ZenohPicoWaitCondition *cond);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
