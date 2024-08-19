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

#include <rmw/allocators.h>
#include <rmw/rmw.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

static rmw_ret_t rmw_zenoh_pico_init_option(ZenohPicoTransportParams *params)
{
  params->mode_ = RMW_ZENOH_PICO_TRANSPORT_MODE;

#if defined(RMW_ZENOH_PICO_TRANSPORT_SERIAL)
  if(strlen(RMW_ZENOH_PICO_SERIAL_DEVICE) > sizeof(params->serial_device)){
    RMW_UROS_TRACE_MESSAGE("default ip configuration overflow")
      return RMW_RET_INVALID_ARGUMENT;
  }

  strncpy(params->serial_device, RMW_ZENOH_PICO_SERIAL_DEVICE, sizeof(params->serial_device));

#elif defined (RMW_ZENOH_PICO_TRANSPORT_UNICAST) || defined (RMW_ZENOH_PICO_TRANSPORT_MCAST)
  char buf[256];

  memset(buf, 0, sizeof(buf));

#if defined (RMW_ZENOH_PICO_TRANSPORT_UNICAST)
  snprintf(buf, sizeof(buf), "tcp/%s:%s", RMW_ZENOH_PICO_CONNECT, RMW_ZENOH_PICO_CONNECT_PORT);
  if(strlen(buf) >= sizeof(params->connect_addr_) -1) {
    RMW_UROS_TRACE_MESSAGE("default ip configuration overflow")
      return RMW_RET_INVALID_ARGUMENT;
  }
  memset(params->connect_addr_, 0, sizeof(params->connect_addr_));
  strcpy(params->connect_addr_, buf);

#elif defined (RMW_ZENOH_PICO_TRANSPORT_MCAST)
  snprintf(buf, sizeof(buf), "udp/%s:%s", RMW_ZENOH_PICO_LOCATOR, RMW_ZENOH_PICO_LOCATOR_PORT);
  if(strlen(buf) >= sizeof(params->locator_addr) -1) {
    RMW_UROS_TRACE_MESSAGE("default ip configuration overflow")
      return RMW_RET_INVALID_ARGUMENT;
  }
  memset(params->locator_addr, 0, sizeof(params->locator_addr));
  strcpy(params->locator_addr, buf);

#endif

  if(strcmp(RMW_ZENOH_PICO_LISTEN_PORT,"-1") != 0){
    memset(buf, 0, sizeof(buf));
    snprintf(buf, sizeof(buf), "tcp/%s:%s", RMW_ZENOH_PICO_LISTEN, RMW_ZENOH_PICO_LISTEN_PORT);
    if(strlen(buf) >= sizeof(params->listen_addr_) -1) {
      RMW_UROS_TRACE_MESSAGE("default ip configuration overflow")
	return RMW_RET_INVALID_ARGUMENT;
    }
    memset(params->listen_addr_, 0, sizeof(params->listen_addr_));
    strcpy(params->listen_addr_, buf);
  }

#endif

  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_options_init(
  rmw_init_options_t * init_options,
  rcutils_allocator_t allocator)
{
  _Z_DEBUG("%s : start", __func__);
  rmw_ret_t ret = RMW_RET_OK;

  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR(&allocator, return RMW_RET_INVALID_ARGUMENT);

  if (NULL != init_options->implementation_identifier) {
    RMW_UROS_TRACE_MESSAGE("expected zero-initialized init_options")
      return RMW_RET_INVALID_ARGUMENT;
  }

  init_options->instance_id		  = 0;
  init_options->implementation_identifier = rmw_get_implementation_identifier();
  init_options->allocator		  = allocator;
  init_options->enclave                   = (char *)NULL;
  init_options->domain_id		  = 0;
  init_options->security_options	  = rmw_get_default_security_options();
  init_options->localhost_only		  = RMW_LOCALHOST_ONLY_DEFAULT;

  // This can be call before rmw_init()
  ZenohPicoTransportParams *params = zenoh_pico_generate_param(NULL);

  if (params == NULL) {
    RMW_UROS_TRACE_MESSAGE("Not available memory node")
      return RMW_RET_ERROR;
  }

  if((ret = rmw_zenoh_pico_init_option(params)) != RMW_RET_OK){
    zenoh_pico_destroy_param(params);
    return ret;
  }

  init_options->impl = (rmw_init_options_impl_t *)params;

  return ret;
}

rmw_ret_t
rmw_init_options_copy(
  const rmw_init_options_t * src,
  rmw_init_options_t * dst)
{
  _Z_DEBUG("%s : start", __func__);

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

  ZenohPicoTransportParams *src_contex = (ZenohPicoTransportParams *)src->impl;
  if(src_contex == NULL){
    RMW_UROS_TRACE_MESSAGE("source context is zero");
    return RMW_RET_INVALID_ARGUMENT;
  }

  ZenohPicoTransportParams *dst_contex = zenoh_pico_generate_param(NULL);
  if(dst_contex == NULL){
    RMW_UROS_TRACE_MESSAGE("falid new allocation option area");
    return RMW_RET_ERROR;
  }

  zenoh_pico_clone_param(dst_contex, src_contex);
  dst->impl = (rmw_init_options_impl_t *)dst_contex;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_options_fini(
  rmw_init_options_t * init_options)
{
  _Z_DEBUG("%s : start", __func__);

  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR(&(init_options->allocator), return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    init_options->implementation_identifier,
    RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  if(init_options->impl != NULL){
    zenoh_pico_destroy_param((ZenohPicoTransportParams*)init_options->impl);
  }

  return RMW_RET_OK;
}

static rmw_ret_t
rmw_zenoh_pico_set_unicast_config(ZenohPicoTransportParams *params, z_owned_config_t *config)
{
  const char *mode = params->mode_;

  if(zp_config_insert(z_config_loan(config),
		      Z_CONFIG_MODE_KEY,
		      z_string_make(mode)) < 0){
    RMW_SET_ERROR_MSG("mode param setting error.");
    return RMW_RET_ERROR;
  }

  if(strlen(params->connect_addr_) > 0){
    if(zp_config_insert(z_config_loan(config),
			Z_CONFIG_CONNECT_KEY,
			z_string_make(params->connect_addr_)) < 0){
      RMW_SET_ERROR_MSG("connect address param setting error.");
      return RMW_RET_ERROR;
    }
  }

  if(strlen(params->listen_addr_) > 0){
    if(zp_config_insert(z_config_loan(config),
			Z_CONFIG_LISTEN_KEY,
			z_string_make(params->listen_addr_)) < 0){
      RMW_SET_ERROR_MSG("listen address param setting error.");
      return RMW_RET_ERROR;
    }
  }

  return RMW_RET_OK;
}

static rmw_ret_t
rmw_zenoh_pico_set_mcast_config(ZenohPicoTransportParams *params, z_owned_config_t *config)
{
  RMW_SET_ERROR_MSG("not support multicast transport, yet.");
  return RMW_RET_ERROR;
}

static rmw_ret_t
rmw_zenoh_pico_set_serial_config(ZenohPicoTransportParams *params, z_owned_config_t *config)
{
  RMW_SET_ERROR_MSG("not support serial transport, yet.");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_init(
  const rmw_init_options_t * options,
  rmw_context_t * context)
{
  _Z_DEBUG("%s : start", __func__);

  RCUTILS_CHECK_ARGUMENT_FOR_NULL(options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(options->impl, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    options->implementation_identifier,
    "expected initialized init options",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    options->implementation_identifier,
    RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    options->enclave,
    "expected non-null enclave",
    return RMW_RET_INVALID_ARGUMENT);
  if (NULL != context->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected a zero-initialized context");
    return RMW_RET_INVALID_ARGUMENT;
  }

  context->instance_id = options->instance_id;
  context->implementation_identifier = rmw_get_implementation_identifier();
  // No custom handling of RMW_DEFAULT_DOMAIN_ID. Simply use a reasonable domain id.
  context->actual_domain_id =
    RMW_DEFAULT_DOMAIN_ID != options->domain_id ? options->domain_id : 0u;

  // setting zenoh-pico configuration
  ZenohPicoTransportParams *params = (ZenohPicoTransportParams *)options->impl;
  z_owned_config_t config = z_config_default();

  {
    rmw_ret_t ret = RMW_RET_OK;
    const char *mode = params->mode_;

    if(strncmp(mode, "unicast", sizeof("unicast"))) {
      ret = rmw_zenoh_pico_set_unicast_config(params, &config);

    } else if(strncmp(mode, "mcast", sizeof("mcast"))) {
      ret = rmw_zenoh_pico_set_mcast_config(params, &config);

    } else if(strncmp(mode, "serial", sizeof("serial"))) {
      ret = rmw_zenoh_pico_set_serial_config(params, &config);
    }

    if(ret != RMW_RET_OK)
      return ret;
  }

  ZenohPicoSession *session = zenoh_pico_generate_session(NULL,
							  z_move(config),
							  options->enclave);
  if(session == NULL){
    RMW_SET_ERROR_MSG("falid generate session data");
    z_drop(z_config_move(&config));
    return RMW_RET_ERROR;
  }

  context->impl = (rmw_context_impl_t *)session;

  return session_connect(session);
}

rmw_ret_t
rmw_shutdown(
  rmw_context_t * context)
{
  _Z_DEBUG("%s : start", __func__);

  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context->implementation_identifier,
    RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  ZenohPicoSession *session = (ZenohPicoSession *)context->impl;

  // stop background zenoh task
  z_owned_session_t *s = z_move(session->session_);
  zp_stop_read_task(z_session_loan(s));
  zp_stop_lease_task(z_session_loan(s));

  z_close(z_session_move(s));

  // free context area
  rmw_ret_t ret = rmw_context_fini(context);

  if (ret == RMW_RET_OK) {
    *context = rmw_get_zero_initialized_context();
  }

  return ret;
}

rmw_ret_t
rmw_context_fini(
  rmw_context_t * context)
{
  _Z_DEBUG("%s : start", __func__);
  rmw_ret_t ret = RMW_RET_OK;

  if(context->impl != NULL){
    zenoh_pico_destroy_session((ZenohPicoSession *)context->impl);
    context->impl = NULL;
  }

  return ret;
}
