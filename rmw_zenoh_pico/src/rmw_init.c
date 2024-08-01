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

#include <time.h>

#include <rmw_zenoh_pico/rmw_c_macros.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros_internal/error_handling_internal.h>
#include <rmw_zenoh_pico/config.h>
#include <rmw/rmw.h>
#include <rmw/allocators.h>

#include <zenoh-pico.h>

extern rmw_zenoh_pico_transport_params_t rmw_zenoh_pico_transport_default_params;

rmw_ret_t
rmw_init_options_init(
  rmw_init_options_t * init_options,
  rcutils_allocator_t allocator)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR(&allocator, return RMW_RET_INVALID_ARGUMENT);

  if (NULL != init_options->implementation_identifier) {
    RMW_UROS_TRACE_MESSAGE("expected zero-initialized init_options")

    return RMW_RET_INVALID_ARGUMENT;
  }

  init_options->instance_id			= 0;
  init_options->implementation_identifier	= zenoh_pico_identifier;
  init_options->allocator			= allocator;
  init_options->enclave				= "/";
  init_options->domain_id			= 0;
  init_options->security_options		= rmw_get_default_security_options();
  init_options->localhost_only			= RMW_LOCALHOST_ONLY_DEFAULT;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_options_copy(
  const rmw_init_options_t * src,
  rmw_init_options_t * dst)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(src, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(dst, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    src->implementation_identifier,
    RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (NULL != dst->implementation_identifier) {
    RMW_UROS_TRACE_MESSAGE("expected zero-initialized dst")

    return RMW_RET_INVALID_ARGUMENT;
  }
  memcpy(dst, src, sizeof(rmw_init_options_t));

  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_options_fini(
  rmw_init_options_t * init_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR(&(init_options->allocator), return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    init_options->implementation_identifier,
    RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_init(
  const rmw_init_options_t * options,
  rmw_context_t * context)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(options->impl, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    options->implementation_identifier,
    RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  context->instance_id = options->instance_id;
  context->implementation_identifier = zenoh_pico_identifier;
  context->actual_domain_id = options->domain_id;

  {
      const char *keyexpr = "demo/example/**";
      const char *mode = "client";
      char *clocator = NULL;
      char *llocator = NULL;

      z_owned_config_t config = z_config_default();
      zp_config_insert(z_config_loan(&config), Z_CONFIG_MODE_KEY, z_string_make(mode));
      if (clocator != NULL) {
          zp_config_insert(z_config_loan(&config), Z_CONFIG_CONNECT_KEY, z_string_make(clocator));
      }
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_shutdown(
  rmw_context_t * context)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context->implementation_identifier,
    RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rmw_ret_t ret = rmw_context_fini(context);

  if (RMW_RET_OK == ret) {
    *context = rmw_get_zero_initialized_context();
  }

  return ret;
}

rmw_ret_t
rmw_context_fini(
  rmw_context_t * context)
{
  rmw_ret_t ret = RMW_RET_OK;

  context->impl = NULL;

  return ret;
}
