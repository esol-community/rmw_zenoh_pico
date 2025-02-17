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

#include <rmw/validate_full_topic_name.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

z_owned_mutex_t mutex_ZenohPicoTransportParams;

static ZenohPicoTransportParams *zenoh_pico_generate_param(ZenohPicoTransportParams *param)
{
  ZenohPicoGenerateData(param, ZenohPicoTransportParams);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    param,
    "failed to allocate struct for the ZenohPicoTransportParams",
    return NULL);

  return param;
}

static bool zenoh_pico_destroy_param(ZenohPicoTransportParams *param)
{
  ZenohPicoDestroyData(param, ZenohPicoTransportParams);

  return true;
}

static bool zenoh_pico_clone_param(ZenohPicoTransportParams *dst, ZenohPicoTransportParams *src)
{
  if ((dst == NULL) || (src == NULL))
    return false;

  memcpy(dst, src, sizeof(ZenohPicoTransportParams));

  return true;
}

static rmw_ret_t rmw_zenoh_pico_init_option(ZenohPicoTransportParams *params)
{
  RMW_ZENOH_FUNC_ENTRY(NULL);

  params->mode = RMW_ZENOH_PICO_TRANSPORT_MODE;

#if defined(RMW_ZENOH_PICO_TRANSPORT_SERIAL)
  memset(params->serial_device, 0, sizeof(params->serial_device));
  if(strlen(RMW_ZENOH_PICO_SERIAL_DEVICE) > sizeof(params->serial_device) -1){
    RMW_ZENOH_LOG_INFO("default ip configuration overflow");
    return RMW_RET_INVALID_ARGUMENT;
  }
  strcpy(params->serial_device, RMW_ZENOH_PICO_SERIAL_DEVICE);

#elif defined (RMW_ZENOH_PICO_TRANSPORT_UNICAST) || defined (RMW_ZENOH_PICO_TRANSPORT_MULTICAST)
  char buf[256];

#if defined (RMW_ZENOH_PICO_TRANSPORT_UNICAST)

  memset(params->connect_addr, 0, sizeof(params->connect_addr));
  if(strcmp(RMW_ZENOH_PICO_CONNECT_PORT, "-1") != 0){
    memset(buf, 0, sizeof(buf));
    snprintf(buf, sizeof(buf), "tcp/%s:%s",
	     RMW_ZENOH_PICO_CONNECT,
	     RMW_ZENOH_PICO_CONNECT_PORT);
    if(strlen(buf) >= sizeof(params->connect_addr) -1) {
      RMW_ZENOH_LOG_INFO("default ip configuration overflow");
      return RMW_RET_INVALID_ARGUMENT;
    }
    strcpy(params->connect_addr, buf);
  }

  memset(params->listen_addr, 0, sizeof(params->listen_addr));
  if(strcmp(RMW_ZENOH_PICO_LISTEN_PORT,"-1") != 0){
    memset(buf, 0, sizeof(buf));
    snprintf(buf, sizeof(buf), "tcp/%s:%s",
	     RMW_ZENOH_PICO_LISTEN,
	     RMW_ZENOH_PICO_LISTEN_PORT);
    if(strlen(buf) >= sizeof(params->listen_addr) -1) {
      RMW_ZENOH_LOG_INFO("default ip configuration overflow");
      return RMW_RET_INVALID_ARGUMENT;
    }
    strcpy(params->listen_addr, buf);
  }

#elif defined (RMW_ZENOH_PICO_TRANSPORT_MULTICAST)

  memset(params->mcast_addr, 0, sizeof(params->mcast_addr));
  if(strcmp(RMW_ZENOH_PICO_MCAST_PORT, "-1") != 0){
    snprintf(buf, sizeof(buf), "udp/%s:%s#%s",
	     RMW_ZENOH_PICO_MCAST,
	     RMW_ZENOH_PICO_MCAST_PORT,
	     RMW_ZENOH_PICO_MCAST_DEV);
    if(strlen(buf) >= sizeof(params->mcast_addr) -1) {
      RMW_ZENOH_LOG_INFO("default ip configuration overflow");
      return RMW_RET_INVALID_ARGUMENT;
    }
    memset(params->mcast_addr, 0, sizeof(params->mcast_addr));
    strcpy(params->mcast_addr, buf);
  }

#endif
#endif

  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_options_init(rmw_init_options_t * init_options, rcutils_allocator_t allocator)
{
  RMW_ZENOH_FUNC_ENTRY(init_options);

  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR(&allocator, return RMW_RET_INVALID_ARGUMENT);

  if (NULL != init_options->implementation_identifier) {
    RMW_ZENOH_LOG_INFO("expected zero-initialized init_options");
    return RMW_RET_INVALID_ARGUMENT;
  }

  init_options->instance_id		  = 0;
  init_options->implementation_identifier = rmw_get_implementation_identifier();
  init_options->allocator		  = allocator;
  init_options->enclave                   = "/";
  init_options->domain_id		  = 0;
  init_options->security_options	  = rmw_get_default_security_options();
  init_options->localhost_only		  = RMW_LOCALHOST_ONLY_DEFAULT;

  // This can be call before rmw_init()
  ZenohPicoTransportParams *params = zenoh_pico_generate_param(NULL);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    params,
    "Not available memory node",
    return RMW_RET_ERROR);

  rmw_ret_t ret;
  if((ret = rmw_zenoh_pico_init_option(params)) != RMW_RET_OK){
    zenoh_pico_destroy_param(params);
    return ret;
  }

  init_options->impl = (rmw_init_options_impl_t *)params;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_options_copy(const rmw_init_options_t * src, rmw_init_options_t * dst)
{
  RMW_ZENOH_FUNC_ENTRY(src);

  RMW_CHECK_ARGUMENT_FOR_NULL(src, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(dst, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    src->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (NULL != dst->implementation_identifier) {
    RMW_ZENOH_LOG_INFO("expected zero-initialized dst");
    return RMW_RET_INVALID_ARGUMENT;
  }

  memcpy(dst, src, sizeof(rmw_init_options_t));

  ZenohPicoTransportParams *src_contex = (ZenohPicoTransportParams *)src->impl;
  RMW_CHECK_FOR_NULL_WITH_MSG(
    src_contex,
    "source context is zero",
    return RMW_RET_INVALID_ARGUMENT);

  ZenohPicoTransportParams *dst_contex = zenoh_pico_generate_param(NULL);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    dst_contex,
    "falid new allocation option area",
    return RMW_RET_ERROR);

  zenoh_pico_clone_param(dst_contex, src_contex);
  dst->impl = (rmw_init_options_impl_t *)dst_contex;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_options_fini(rmw_init_options_t * init_options)
{
  RMW_ZENOH_FUNC_ENTRY(init_options);

  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR(&(init_options->allocator), return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    init_options->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  if(init_options->impl != NULL){
    zenoh_pico_destroy_param((ZenohPicoTransportParams*)init_options->impl);
    init_options->impl = NULL;
  }

  return RMW_RET_OK;
}

static rmw_ret_t
rmw_zenoh_pico_set_common_config(ZenohPicoTransportParams *params, z_owned_config_t *config)
{
  // RMW_ZENOH_LOG_INFO("Z_CONFIG_MODE_KEY = %s", params->mode);
  if(_Z_IS_ERR(zp_config_insert(z_config_loan_mut(config),
				Z_CONFIG_MODE_KEY,
				params->mode))){
    RMW_SET_ERROR_MSG("mode param setting error.");
    return RMW_RET_ERROR;
  }

#if defined (RMW_ZENOH_PICO_TRANSPORT_UNICAST)

  if(strlen(params->connect_addr) > 0){

    // RMW_ZENOH_LOG_INFO("Z_CONFIG_CONNECT_KEY = %s", params->connect_addr);
    if(_Z_IS_ERR(zp_config_insert(z_config_loan_mut(config),
				  Z_CONFIG_CONNECT_KEY,
				  params->connect_addr))){
      RMW_SET_ERROR_MSG("connect address param setting error.");
      return RMW_RET_ERROR;
    }
  }

  if(strlen(params->listen_addr) > 0){

    // RMW_ZENOH_LOG_INFO("Z_CONFIG_LISTEN_KEY = %s", params->listen_addr);
    if(_Z_IS_ERR(zp_config_insert(z_config_loan_mut(config),
				  Z_CONFIG_LISTEN_KEY,
				  params->listen_addr))){
      RMW_SET_ERROR_MSG("listen address param setting error.");
      return RMW_RET_ERROR;
    }
  }

#elif defined (RMW_ZENOH_PICO_TRANSPORT_MULTICAST)
  if(strlen(params->mcast_addr) > 0){

    // RMW_ZENOH_LOG_INFO("Z_CONFIG_MULTICAST_LOCATOR_KEY = %s", params->mcast_addr);
    if(_Z_IS_ERR(zp_config_insert(z_config_loan_mut(config),
				  Z_CONFIG_MULTICAST_LOCATOR_KEY,
				  params->mcast_addr))){
      RMW_SET_ERROR_MSG("multucast address param setting error.");
      return RMW_RET_ERROR;
    }
  }
#endif

  return RMW_RET_OK;
}

static rmw_ret_t
rmw_zenoh_pico_set_unicast_config(ZenohPicoTransportParams *params, z_owned_config_t *config)
{
  if (strncmp(RMW_ZENOH_PICO_TRANSPORT_MODE, "client", strlen("client")) != 0)
    return RMW_RET_ERROR;

  return rmw_zenoh_pico_set_common_config(params, config);
}

static rmw_ret_t
rmw_zenoh_pico_set_mcast_config(ZenohPicoTransportParams *params, z_owned_config_t *config)
{
  if (strncmp(RMW_ZENOH_PICO_TRANSPORT_MODE, "peer", strlen("peer")) != 0)
    return RMW_RET_ERROR;

  return rmw_zenoh_pico_set_common_config(params, config);
}

static rmw_ret_t
rmw_zenoh_pico_set_serial_config(ZenohPicoTransportParams *params, z_owned_config_t *config)
{
  RMW_SET_ERROR_MSG("not support serial transport, yet.");
  return RMW_RET_ERROR;
}

void rmw_zenoh_pico_init(void)
{
  z_mutex_init(&mutex_ZenohPicoSubData);
  z_mutex_init(&mutex_ZenohPicoTransportParams);
  z_mutex_init(&mutex_ZenohPicoSession);
  z_mutex_init(&mutex_ZenohPicoWaitSetData);
  z_mutex_init(&mutex_ZenohPicoNodeData);
  z_mutex_init(&mutex_ZenohPicoPubData);
  z_mutex_init(&mutex_ZenohPicoEntity);
  z_mutex_init(&mutex_ZenohPicoTopicInfo);

  rmw_zenoh_pico_log_init();
}

rmw_ret_t
rmw_init(const rmw_init_options_t * options, rmw_context_t * context)
{
  rmw_zenoh_pico_debug_level_init();

  RMW_ZENOH_FUNC_ENTRY(options);

  RCUTILS_CHECK_ARGUMENT_FOR_NULL(options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(options->impl, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    options->implementation_identifier,
    "expected initialized init options",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    options->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    options->enclave,
    "expected non-null enclave",
    return RMW_RET_INVALID_ARGUMENT);
  if (NULL != context->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected a zero-initialized context");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // initirize mutexs which is for private data structure.
  rmw_zenoh_pico_init();

  // set data for context data area.
  context->instance_id = options->instance_id;
  context->implementation_identifier = rmw_get_implementation_identifier();
  // No custom handling of RMW_DEFAULT_DOMAIN_ID. Simply use a reasonable domain id.
  context->actual_domain_id =
    RMW_DEFAULT_DOMAIN_ID != options->domain_id ? options->domain_id : 0u;

  // setting zenoh-pico configuration
  ZenohPicoTransportParams *params = (ZenohPicoTransportParams *)options->impl;

  z_owned_config_t config;
  z_config_default(&config);

  {
    rmw_ret_t ret = RMW_RET_OK;

#if defined (RMW_ZENOH_PICO_TRANSPORT_UNICAST)
      ret = rmw_zenoh_pico_set_unicast_config(params, &config);
#elif defined (RMW_ZENOH_PICO_TRANSPORT_MULTICAST)
      ret = rmw_zenoh_pico_set_mcast_config(params, &config);
#elif defined(RMW_ZENOH_PICO_TRANSPORT_SERIAL)
      ret = rmw_zenoh_pico_set_serial_config(params, &config);
#endif

    if(ret != RMW_RET_OK)
      return ret;
  }

  ZenohPicoSession *session = zenoh_pico_generate_session(z_loan(config), options->enclave);
  if(session == NULL){
    RMW_SET_ERROR_MSG("falid generate session data");
    z_drop(z_config_move(&config));
    return RMW_RET_ERROR;
  }

  context->impl = (rmw_context_impl_t *)session;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_shutdown(rmw_context_t * context)
{
  RMW_ZENOH_FUNC_ENTRY(context);

  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context->implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // free context area
  rmw_ret_t ret = rmw_context_fini(context);

  if (ret == RMW_RET_OK) {
    *context = rmw_get_zero_initialized_context();
  }

  return ret;
}

rmw_ret_t
rmw_context_fini(rmw_context_t * context)
{
  RMW_ZENOH_FUNC_ENTRY(context);
  rmw_ret_t ret = RMW_RET_OK;

  if(context->impl != NULL){
    zenoh_pico_destroy_session((ZenohPicoSession *)context->impl);
    context->impl = NULL;
  }

  return ret;
}
