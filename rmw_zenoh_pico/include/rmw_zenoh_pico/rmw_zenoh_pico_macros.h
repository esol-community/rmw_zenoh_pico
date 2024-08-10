// Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <rmw/error_handling.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico_identifiers.h>

//
// for identifier data utilities
//
#define RMW_CHECK_TYPE_IDENTIFIERS_MATCH(identifier, ret_on_failure) \
  { \
    if (NULL != identifier && strcmp(identifier, zenoh_pico_identifier) != 0) { \
      RMW_SET_ERROR_MSG("Implementation identifiers does not match"); \
      return ret_on_failure; \
    } \
  }

//
// rmw_zenoh_pico private data Generater utilities
//
#define ZenohPicoGenerateData(d, t)		\
  {						\
    if ((d) == NULL) {				\
      (d) = (t *)z_malloc(sizeof(t));		\
      if ((d) != NULL) {			\
	memset((d), 0, sizeof(t));		\
	(d)->is_alloc_ = true;			\
      }						\
    }						\
  }						\

#define ZenohPicoDestroyData(d)			\
  {						\
    if((d) != NULL) {				\
      if((d)->is_alloc_ == true) {		\
	z_free((d));				\
      }						\
    }						\
  }

#endif  // RMW_ZENOH_PICO_MACROS_H_
