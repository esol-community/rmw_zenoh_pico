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

extern z_mutex_t mutex_ZenohPicoSubData;
extern z_mutex_t mutex_ZenohPicoTransportParams;
extern z_mutex_t mutex_ZenohPicoSession;
extern z_mutex_t mutex_ZenohPicoWaitSetData;
extern z_mutex_t mutex_ZenohPicoNodeData;
extern z_mutex_t mutex_ZenohPicoPubData;
extern z_mutex_t mutex_ZenohPicoEntity;
extern z_mutex_t mutex_ZenohPicoTopicInfo;

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
      if ((D) != NULL) {			\
	memset((D), 0, sizeof(T));		\
	z_mutex_lock(&mutex_##T);		\
	(D)->ref = 1;				\
	z_mutex_unlock(&mutex_##T);		\
      }						\
    }						\
  }						\

#define ZenohPicoDestroyData(D, T)		\
  {						\
    if((D) != NULL) {				\
      z_mutex_lock(&mutex_##T);			\
      (D)->ref -= 1;				\
      if((D)->ref == 0) {			\
	z_mutex_unlock(&mutex_##T);		\
	Z_FREE((D));				\
      }else{					\
	z_mutex_unlock(&mutex_##T);		\
      }						\
    }						\
  }

#define ZenohPicoLoanData(D, T)			\
  {						\
    z_mutex_lock(&mutex_##T);			\
    (D)->ref += 1;				\
    z_mutex_unlock(&mutex_##T);			\
  }						\

//
// zenoh-pico macro
//

#define Z_MALLOC(size) z_malloc(size)
#define Z_FREE(ptr)    z_free(ptr)

#define Z_STRING_PRINTF(_z_str, tag)		\
  {						\
    if(_z_str.len == 0)				\
      printf(#tag " = []\n");			\
    else					\
      printf(#tag "= [%s]\n", _z_str.val);	\
  }						\

#define Z_STRING_VAL(s)    (s.val)
#define Z_STRING_LEN(s)    (s.len)
#define Z_STRING_ENABLE(s) (s.val != NULL)

#define Z_STRING_FREE(s)			\
  {						\
    if(!Z_STRING_ENABLE(s))			\
      _z_string_clear(&s);			\
  }						\

#define Z_DECLARATION_FREE(s) (_z_declaration_clear(&s))

#endif  // RMW_ZENOH_PICO_MACROS_H_
