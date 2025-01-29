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

#include <rmw/rmw.h>
#include <zenoh-pico.h>

#ifndef RMW_ZENOH_PICO_ATTACH_H
#define RMW_ZENOH_PICO_ATTACH_H

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  typedef struct _zenoh_pico_attachemt_data {
    int64_t sequence_num;
    int64_t timestamp;
    z_owned_slice_t gid;
  } zenoh_pico_attachemt_data;

  extern time_t zenoh_pico_gen_timestamp(void);
  extern uint64_t zenoh_pico_inc_sequence_num(zenoh_pico_attachemt_data *data);
  extern z_result_t zenoh_pico_attachment_data_get(const z_loaned_sample_t *sample, zenoh_pico_attachemt_data *data);
  extern z_result_t zenoh_pico_attachment_gen(zenoh_pico_attachemt_data *data, z_owned_bytes_t *attachment);
  extern bool zenoh_pico_destroy_attachment(zenoh_pico_attachemt_data *data);

  extern void zenoh_pico_debug_attachment(zenoh_pico_attachemt_data *data);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif
