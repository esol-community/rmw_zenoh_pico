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

z_result_t z_condvar_init_with_attr(z_owned_condvar_t *cv, pthread_condattr_t *attr){
  _Z_CHECK_SYS_ERR(pthread_cond_init(&cv->_val, attr));
}

z_result_t z_condvar_timewait(z_loaned_condvar_t *cv, z_loaned_mutex_t *mp, struct timespec *wait_timeout){
  struct timespec abstime;

  memset(&abstime, 0, sizeof(abstime));
  clock_gettime(CLOCK_REALTIME, &abstime);

  uint64_t _nsec_time = abstime.tv_nsec + wait_timeout->tv_nsec;
  abstime.tv_sec += wait_timeout->tv_sec + (_nsec_time/1000000000);
  abstime.tv_nsec = _nsec_time % 1000000000;

  // RMW_ZENOH_LOG_DEBUG("wait_timeout[%ld : %ld] => [%ld : %ld]",
  // 		      wait_timeout->tv_sec,
  // 		      wait_timeout->tv_nsec,
  // 		      abstime.tv_sec,
  // 		      abstime.tv_nsec);

  z_result_t ret = _z_condvar_wait_until(cv, mp, &abstime);

  return ret;
}
#else
#include "zenoh-pico/system/platform/void.h"
#error "Unknown platform"
#endif

void zenoh_pico_condition_trigger(ZenohPicoWaitCondition *cond){

  z_mutex_lock(cond->condition_mutex);

  if(*cond->wait_set_data_ptr != NULL) {
    ZenohPicoWaitSetData * wait_set_data = *cond->wait_set_data_ptr;

    wait_condition_lock(wait_set_data);

    wait_condition_triggered(wait_set_data, true);
    wait_condition_signal(wait_set_data);

    wait_condition_unlock(wait_set_data);
  }

  z_mutex_unlock(cond->condition_mutex);
}

bool zenoh_pico_condition_check_and_attach(ZenohPicoWaitCondition *cond,
					   ZenohPicoWaitSetData *wait_set_data)
{
  z_mutex_lock(cond->condition_mutex);

  if(!recv_msg_list_empty(cond->msg_queue)){
    RMW_ZENOH_LOG_INFO("queue_has_data_and_attach_condition_if_notmessage_queue size is %d",
		       recv_msg_list_count(cond->msg_queue));
    z_mutex_unlock(cond->condition_mutex);
    return true;
  }

  *cond->wait_set_data_ptr = wait_set_data;

  z_mutex_unlock(cond->condition_mutex);

  return false;
}

bool zenoh_pico_condition_detach_and_queue_is_empty(ZenohPicoWaitCondition *cond)
{
  bool ret;

  z_mutex_lock(cond->condition_mutex);

  *cond->wait_set_data_ptr = NULL;
  ret = recv_msg_list_empty(cond->msg_queue);

  z_mutex_unlock(cond->condition_mutex);

  return ret;
}
