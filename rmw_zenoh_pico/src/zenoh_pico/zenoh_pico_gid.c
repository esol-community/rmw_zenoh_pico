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

#define XXH_STATIC_LINKING_ONLY /* access advanced declarations */
#define XXH_IMPLEMENTATION      /* access definitions */

#include <string.h>
#include "utilities/xxhash.h"

static bool _zenoh_pico_gen_gid(const void *data, size_t length, uint64_t *high64, uint64_t *low64)
{
  XXH128_hash_t result = XXH3_128bits(data, length);

  *low64 = result.low64;
  *high64 = result.high64;

  return true;
}

#else
#include "zenoh-pico/system/platform/void.h"
#error "Unknown platform"
#endif

z_result_t zenoh_pico_gen_gid(const z_loaned_string_t *key, uint8_t *gid)
{
  uint64_t low64, high64;
  const char *string_data = z_string_data(key);
  size_t string_len = z_string_len(key);

  if(!_zenoh_pico_gen_gid(string_data, string_len, &high64, &low64))
    return(_Z_ERR_GENERIC);

  memcpy(gid, &low64, sizeof(low64));
  memcpy(gid + sizeof(low64), &high64, sizeof(high64));

  printf("key = [%.*s][%lx%lx]\n", (int)string_len, string_data, low64, high64);

  return Z_OK;
}
