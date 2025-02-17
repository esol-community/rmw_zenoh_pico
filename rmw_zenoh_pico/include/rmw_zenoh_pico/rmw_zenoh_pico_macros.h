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

#ifndef RMW_ZENOH_PICO_MACROS_H_
#define RMW_ZENOH_PICO_MACROS_H_

#include <rmw/rmw.h>
#include <zenoh-pico.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  extern z_owned_mutex_t mutex_ZenohPicoSubData;
  extern z_owned_mutex_t mutex_ZenohPicoTransportParams;
  extern z_owned_mutex_t mutex_ZenohPicoSession;
  extern z_owned_mutex_t mutex_ZenohPicoWaitSetData;
  extern z_owned_mutex_t mutex_ZenohPicoNodeData;
  extern z_owned_mutex_t mutex_ZenohPicoPubData;
  extern z_owned_mutex_t mutex_ZenohPicoClientData;
  extern z_owned_mutex_t mutex_ZenohPicoEntity;
  extern z_owned_mutex_t mutex_ZenohPicoTopicInfo;

  extern void rmw_zenoh_pico_mutex_init(void);

//
// for identifier data utilities
//
#define RMW_CHECK_TYPE_IDENTIFIERS_MATCH(identifier, ret_on_failure)	\
  {									\
    if (NULL != identifier && strcmp(identifier, rmw_get_implementation_identifier()) != 0) { \
      RMW_SET_ERROR_MSG("Implementation identifiers does not match");	\
      ret_on_failure;							\
    }									\
  }

//
// rmw_zenoh_pico private data Generater utilities
//
#define ZenohPicoGenerateData(D, T)		\
  {						\
    if ((D) == NULL) {				\
      (D) = (T *)z_malloc(sizeof(T));		\
    }						\
    if ((D) != NULL) {				\
      memset((D), 0, sizeof(T));		\
      z_mutex_lock(z_loan_mut(mutex_##T));	\
      (D)->ref = 1;				\
      z_mutex_unlock(z_loan_mut(mutex_##T));	\
    }						\
  }

#define ZenohPicoDestroyData(D, T)		\
  {						\
    if((D) != NULL) {				\
      z_mutex_lock(z_loan_mut(mutex_##T));	\
      (D)->ref -= 1;				\
    }						\
    if((D)->ref == 0) {				\
      z_mutex_unlock(z_loan_mut(mutex_##T));	\
      Z_FREE((D));				\
    }else{					\
      z_mutex_unlock(z_loan_mut(mutex_##T));	\
    }						\
  }

#define ZenohPicoLoanData(D, T)			\
  {						\
    z_mutex_lock(z_loan_mut(mutex_##T));	\
    (D)->ref += 1;				\
    z_mutex_unlock(z_loan_mut(mutex_##T));	\
  }

#define ZenohPicoReturnData(D, T)		\
  {						\
    z_mutex_lock(z_loan_mut(mutex_##T));	\
    (D)->ref -= 1;				\
    z_mutex_unlock(z_loan_mut(mutex_##T));	\
  }

//
// zenoh-pico macro
//

#define Z_MALLOC(size)       z_malloc(size)
#define Z_FREE(ptr)          z_free((void *)ptr)

#define TOPIC_MALLOC(size)   Z_MALLOC(size)
#define TOPIC_FREE(ptr)      Z_FREE(ptr)

#define Z_STRING_VAL(s)      (z_string_data(z_loan(s)))
#define Z_STRING_LEN(s)      ((int)z_string_len(z_loan(s)))

#define Z_STRING_PRINTF(_z_str, tag)					\
  {									\
    const z_loaned_string_t *__z_str = z_loan(_z_str);			\
    if(z_string_is_empty(__z_str))					\
      printf(#tag " = []\n");						\
    else								\
      printf(#tag " = [%.*s]\n", (int)z_string_len(__z_str), z_string_data(__z_str)); \
  }									\

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif  // RMW_ZENOH_PICO_MACROS_H_
