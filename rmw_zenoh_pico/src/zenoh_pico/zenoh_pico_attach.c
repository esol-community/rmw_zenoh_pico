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

#include <stdint.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

uint64_t zenoh_pico_gen_timestamp(void) {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);

  uint64_t timestamp;
  timestamp = (uint64_t)(ts.tv_sec);
  timestamp *= 1000000000ull;
  timestamp += (uint64_t)ts.tv_nsec;

  return timestamp;
}

uint64_t attachment_sequence_num_inc(zenoh_pico_attachemt_data *data)
{
  return data->sequence_num ++;
}

#define CHECK_ATTACHMENT_DESERIALIZE_STRING(data, value)		\
  {									\
    z_result_t ret = ze_deserializer_deserialize_string(&data, &value); \
    if(_Z_IS_ERR(ret)) {						\
      return(ret);							\
    }									\
  }

#define CHECK_ATTACHMENT_DESERIALIZE_INT64(data, value)			\
  {									\
    z_result_t ret = ze_deserializer_deserialize_int64(&data, &value);	\
    if(_Z_IS_ERR(ret)) {						\
      return(ret);							\
    }									\
  }

#define CHECK_ATTACHMENT_DESERIALIZE_SLICE(data, value)			\
  {									\
    z_result_t ret = ze_deserializer_deserialize_slice(&data, &value);	\
    if(_Z_IS_ERR(ret)) {						\
      return(ret);							\
    }									\
  }

z_result_t attachment_data_get(const z_loaned_bytes_t *attachment, zenoh_pico_attachemt_data *data)
{
  ze_deserializer_t deserializer = ze_deserializer_from_bytes(attachment);

  size_t item_num = 3;

  for(size_t count = 0; count < item_num; count++){
    z_owned_string_t key;

    CHECK_ATTACHMENT_DESERIALIZE_STRING(deserializer, key);

    if(strncmp("sequence_number", Z_STRING_VAL(key), Z_STRING_LEN(key)) == 0){
      z_owned_string_t sequence_num;

      z_string_empty(&sequence_num);
      CHECK_ATTACHMENT_DESERIALIZE_INT64(deserializer, data->sequence_num);

    } else if (strncmp("source_timestamp", Z_STRING_VAL(key), Z_STRING_LEN(key)) == 0){
      int64_t timestamp;

      CHECK_ATTACHMENT_DESERIALIZE_INT64(deserializer, timestamp);
      data->timestamp = timestamp;

    } else if (strncmp("source_gid", Z_STRING_VAL(key), Z_STRING_LEN(key)) == 0) {
      z_slice_empty(&data->gid);
      CHECK_ATTACHMENT_DESERIALIZE_SLICE(deserializer, data->gid);
    }

    z_drop(z_move(key));
  }

  return(Z_OK);
}

#define CHECK_ATTACHMENT_SERIALIZE_STRING(data, value)			\
  {									\
  z_result_t ret = ze_serializer_serialize_str(z_loan_mut(data), value); \
  if(_Z_IS_ERR(ret)) {							\
      return(ret);							\
    }									\
    }

#define CHECK_ATTACHMENT_SERIALIZE_INT64(data, value)			\
  {									\
    z_result_t ret = ze_serializer_serialize_int64(z_loan_mut(data), value); \
    if(_Z_IS_ERR(ret)) {						\
      return(ret);							\
    }									\
  }

#define CHECK_ATTACHMENT_SERIALIZE_SLICE(data, value)			\
  {									\
    z_result_t ret = ze_serializer_serialize_slice(z_loan_mut(data), z_loan(value)); \
    if(_Z_IS_ERR(ret)) {						\
      return(ret);							\
    }									\
  }

z_result_t attachment_gen(zenoh_pico_attachemt_data *data, z_owned_bytes_t *attachment)
{
  z_bytes_empty(attachment);

  ze_owned_serializer_t serializer;
  ze_serializer_empty(&serializer);

  // update timestamp
  uint64_t timestamp = zenoh_pico_gen_timestamp();
  data->timestamp = timestamp;

  // serialize attachment data
  CHECK_ATTACHMENT_SERIALIZE_STRING(serializer, "sequence_number");
  CHECK_ATTACHMENT_SERIALIZE_INT64(serializer, data->sequence_num);
  CHECK_ATTACHMENT_SERIALIZE_STRING(serializer, "source_timestamp");
  CHECK_ATTACHMENT_SERIALIZE_INT64(serializer, data->timestamp);
  CHECK_ATTACHMENT_SERIALIZE_STRING(serializer, "source_gid");
  CHECK_ATTACHMENT_SERIALIZE_SLICE(serializer, data->gid);

  ze_serializer_finish(z_move(serializer), attachment);

  return(Z_OK);
}

bool attachment_destroy(zenoh_pico_attachemt_data *attachmet_data)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  RMW_CHECK_ARGUMENT_FOR_NULL(attachmet_data, false);

  z_drop(z_move(attachmet_data->gid));

  return true;
}

void attachment_debug(zenoh_pico_attachemt_data *data)
{
  printf("--------- attachment data ----------\n");
#if defined(__x86_64__)
  printf("sequence_num = [%lu]\n", data->sequence_num);
  printf("timestamp    = [%lu]\n", data->timestamp);
#else
  printf("sequence_num = [%llu]\n", data->sequence_num);
  printf("timestamp    = [%llu]\n", data->timestamp);
#endif
  printf("gid          = [");
  const uint8_t *gid_ptr = z_slice_data(z_loan(data->gid));
  size_t len = z_slice_len(z_loan(data->gid));

  for(size_t index = 0; index < len; index += 4){
    printf("%02x%02x%02x%02x",
	   *(gid_ptr +index +0),
	   *(gid_ptr +index +1),
	   *(gid_ptr +index +2),
	   *(gid_ptr +index +3));
  }
#if defined(__x86_64__)
  printf("][%ld]\n", len);
#else
  printf("][%d]\n", len);
#endif
}
