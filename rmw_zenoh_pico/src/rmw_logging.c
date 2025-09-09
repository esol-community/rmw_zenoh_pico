// Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// Copyright(C) 2024 eSOL Co., Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "zenoh-pico/api/macros.h"
#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

static z_owned_mutex_t mutex_logging;

// Timestamp function
#if defined(ZENOH_LINUX) || defined (ZENOH_ARDUINO_ESP32) || defined (ZENOH_ESPIDF)

void z_log_prefix(const char *prefix, const char *func_name, const char *fmt, ...) {
  static char tstamp[64];
  static char msg[128];

  struct timespec abstime;
  clock_gettime(CLOCK_REALTIME, &abstime);

  snprintf(tstamp, sizeof(tstamp) -1, "%ld.%09ld", abstime.tv_sec, abstime.tv_nsec);

  va_list args;
  va_start(args , fmt);
  int ret = vsnprintf(msg, sizeof(msg) -1, fmt, args);
  va_end(args);

  printf("[%-5s] [%s] [%s] : %s\n",  prefix, tstamp, func_name, msg);
}
#else
#error "Unsupported platform"
#endif

static int rmw_zenoh_pico_debug_level = _Z_LOG_LVL_ERROR;
void rmw_zenoh_pico_debug_level_init(void)
{
  char *pathvar;

  if((pathvar = getenv("RMW_ZNEOH_PICO_LOG")) == NULL)
    return;

  rmw_zenoh_pico_debug_level = _Z_LOG_LVL_ERROR;
  if(strncmp(pathvar, "Z_LOG_DEBUG", sizeof("Z_LOG_DEBUG")) == 0) {
    if(rmw_zenoh_pico_debug_level < _Z_LOG_LVL_DEBUG)
      rmw_zenoh_pico_debug_level = _Z_LOG_LVL_DEBUG;
  } else if (strncmp(pathvar, "Z_LOG_INFO", sizeof("Z_LOG_INFO")) == 0) {
    if(rmw_zenoh_pico_debug_level < _Z_LOG_LVL_INFO)
      rmw_zenoh_pico_debug_level = _Z_LOG_LVL_INFO;
  } else if (strncmp(pathvar, "Z_LOG_ERROR", sizeof("Z_LOG_ERROR")) == 0) {
    if(rmw_zenoh_pico_debug_level < _Z_LOG_LVL_ERROR)
      rmw_zenoh_pico_debug_level = _Z_LOG_LVL_ERROR;
  }

  return;
}

bool rmw_zenoh_pico_check_validate_name(const char * name)
{
  int validation_result = RMW_TOPIC_VALID;
  rmw_ret_t ret = rmw_validate_full_topic_name(name,
					       &validation_result,
					       NULL);
  if (RMW_RET_OK != ret) {
    return false;
  }
  if (RMW_TOPIC_VALID != validation_result) {
    const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid topic name: %s", reason);
    return false;
  }

  return true;
}

int rmw_zenoh_pico_debug_level_get(void)
{
  return rmw_zenoh_pico_debug_level;
}

int rmw_zenoh_pico_debug_level_set(int level)
{
  rmw_zenoh_pico_debug_level = level;

  return rmw_zenoh_pico_debug_level_get();
}

void rmw_zenoh_pico_log_init()
{
  z_mutex_init(&mutex_logging);
}
void rmw_zenoh_pico_log_lock()
{
  z_mutex_lock(z_loan_mut(mutex_logging));
}

void rmw_zenoh_pico_log_unlock()
{
  z_mutex_unlock(z_loan_mut(mutex_logging));
}

rmw_ret_t
rmw_set_log_severity(rmw_log_severity_t severity)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  (void)severity;
  RMW_ZENOH_LOG_INFO("function not implemented");
  return RMW_RET_UNSUPPORTED;
}
