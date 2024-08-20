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

#include "rmw_zenoh_pico/rmw_zenoh_pico_session.h"
#include <rmw_zenoh_pico/config.h>

#ifdef HAVE_C_TYPESUPPORT
#include <rosidl_typesupport_microxrcedds_c/identifier.h>
#endif /* ifdef HAVE_C_TYPESUPPORT */
#ifdef HAVE_CPP_TYPESUPPORT
#include <rosidl_typesupport_microxrcedds_cpp/identifier.h>
#endif /* ifdef HAVE_CPP_TYPESUPPORT */

#include <rmw/rmw.h>
#include <rmw/types.h>
#include <rmw/allocators.h>
#include <rmw/error_handling.h>

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_typesupport_microxrcedds_c/message_type_support.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

#define RMW_ZENOH_PICO_TYPESUPPORT_C rosidl_typesupport_rmw_zenoh_identifier

static rmw_subscription_t *rmw_subscription_generate(rmw_context_t *context,
						     ZenohPicoSubData *sub_data,
						     const rmw_subscription_options_t *options)
{
  _Z_DEBUG("%s : start()", __func__);

  rmw_subscription_t * rmw_subscription = z_malloc(sizeof(rmw_subscription_t));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_subscription,
    "failed to allocate memory for the subscription",
    return NULL);

  rmw_subscription->implementation_identifier	= zenoh_pico_identifier;
  rmw_subscription->topic_name			= Z_STRING_VAL(sub_data->entity_->topic_info_->name_);
  rmw_subscription->options			= *options;
  rmw_subscription->can_loan_messages		= false;
  rmw_subscription->is_cft_enabled		= false;

  rmw_subscription->data                        = (void *)sub_data;

  return rmw_subscription;
}

static rmw_ret_t rmw_subscription_destroy(rmw_subscription_t * sub)
{
  _Z_DEBUG("%s : start()", __func__);

  return RMW_RET_OK;
}

const rosidl_message_type_support_t * find_message_type_support(
  const rosidl_message_type_support_t * type_supports)
{
  const rosidl_message_type_support_t * type_support = get_message_typesupport_handle(
    type_supports, RMW_ZENOH_PICO_TYPESUPPORT_C);
  if (!type_support) {
    rcutils_error_string_t error_string = rcutils_get_error_string();
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Type support not from this implementation. Got:\n"
      "    %s\n"
      "while fetching it",
      error_string.str);
    return NULL;
  }

  return type_support;
}

rmw_ret_t
rmw_init_subscription_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_subscription_allocation_t * allocation)
{
  _Z_DEBUG("%s : start()", __func__);

  (void)type_support;
  (void)message_bounds;
  (void)allocation;
  RMW_UROS_TRACE_MESSAGE("function not implemented")
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_fini_subscription_allocation(
  rmw_subscription_allocation_t * allocation)
{
  _Z_DEBUG("%s : start()", __func__);
  (void)allocation;
  RMW_UROS_TRACE_MESSAGE("function not implemented")
  return RMW_RET_UNSUPPORTED;
}

static const char RIHS01_PREFIX[]	= "RIHS01_";
static const size_t RIHS_VERSION_IDX	= 4;
static const size_t RIHS_PREFIX_LEN	= 7;
static const size_t RIHS01_STRING_LEN	= 71;  // RIHS_PREFIX_LEN + (ROSIDL_TYPE_HASH_SIZE * 2);
static const uint8_t INVALID_NIBBLE	= 0xff;

static void test_qos_profile(rmw_qos_profile_t *qos) {
  // Reliability.
  qos->reliability	= RMW_QOS_POLICY_RELIABILITY_RELIABLE; // 1;
  // Durability.
  qos->durability	= RMW_QOS_POLICY_DURABILITY_VOLATILE;  // 2;
  // History.
  qos->history	= RMW_QOS_POLICY_HISTORY_KEEP_LAST;    // 1;
  qos->depth		= 10;
  // Deadline.
  qos->deadline.sec	= 0;
  qos->deadline.nsec	= 0;
  // Lifespan.
  qos->lifespan.sec	= 0;
  qos->lifespan.nsec	= 0;
  // Liveliness.
  qos->liveliness     = 0;
  qos->liveliness_lease_duration.sec = 0;
  qos->liveliness_lease_duration.nsec = 0;
}

rmw_subscription_t *
rmw_create_subscription(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_subscription_options_t * subscription_options)
{
  _Z_DEBUG("%s : start(%s)", __func__, topic_name);
  RMW_CHECK_ARGUMENT_FOR_NULL(node, NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, NULL);
  if (topic_name[0] == '\0') {
    RMW_SET_ERROR_MSG("topic_name argument is an empty string");
    return NULL;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_options, NULL);

  RMW_CHECK_ARGUMENT_FOR_NULL(node->data, NULL);

  ZenohPicoNodeData *node_data = (ZenohPicoNodeData *)node->data;
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node_data, "unable to create subscription as node_data is invalid.",
    return NULL);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context,
    "expected initialized context",
    return NULL);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context->impl,
    "expected initialized context impl",
    return NULL);

  // scan hash data by type_support
  const rosidl_message_type_support_t * type_support = find_message_type_support(type_supports);
  if (type_support == NULL) {
    // error was already set by find_message_type_support
    RMW_UROS_TRACE_MESSAGE("type_support is null");
    return NULL;
  }
  const rosidl_type_hash_t * type_hash = type_support->get_type_hash_func(type_support);

  // convert hash
  _z_string_t _hash_data = convert_hash(type_hash);
  _Z_INFO("%s : hash = [%s][%s]", __func__, topic_name, _hash_data.val);

  // generate message type
  const message_type_support_callbacks_t *callbacks
    = (const message_type_support_callbacks_t *)(type_support->data);

  z_string_t _type_name = convert_message_type(callbacks);
  _Z_INFO("%s : type name = [%s]", __func__, _type_name.val);

  // generate Qos
  // rmw_qos_profile_t _qos_profile = *qos_profile;
  rmw_qos_profile_t _qos_profile;
  memset(&_qos_profile, 0, sizeof(_qos_profile));
  test_qos_profile(&_qos_profile);

  z_string_t qos_key = qos_to_keyexpr(&_qos_profile);
  _Z_INFO("%s : qos = [%s]", __func__, qos_key.val);

  _z_string_t _topic_name = _z_string_make(topic_name);
  ZenohPicoTopicInfo_t *_topic_info = zenoh_pico_generate_topic_info(&_topic_name,
								     &_type_name,
								     &_hash_data,
								     &qos_key);
  // clone node_info
  ZenohPicoNodeInfo_t *_node_info = zenoh_pico_clone_node_info(node_data->entity_->node_info_);

  // generate entity data
  size_t _entity_id = zenoh_pico_get_next_entity_id();
  ZenohPicoSession *session = node_data->session_;

  ZenohPicoEntity *_entity = zenoh_pico_generate_entitiy(NULL,
							 z_info_zid(z_loan(session->session_)),
							 _entity_id,
							 node_data->id_,
							 Subscription,
							 _node_info,
							 _topic_info);
  // zenoh_pico_debug_entitiy(_entity);

  // generate subscription data
  ZenohPicoNodeData *_node = zenoh_pico_loan_node_data(node_data);
  ZenohPicoSubData *_sub_data = zenoh_pico_generate_sub_data(_entity_id,
							     _node,
							     _entity,
							     type_support,
							     &_qos_profile);

  zenoh_pico_debug_sub_data(_sub_data);

  rmw_subscription_t * rmw_subscription = rmw_subscription_generate(node->context,
								    _sub_data,
								    subscription_options);
  declaration_sub_data(_sub_data);

  // return rmw_subscription;
  return rmw_subscription;
}

rmw_ret_t
rmw_subscription_count_matched_publishers(
  const rmw_subscription_t * subscription,
  size_t * publisher_count)
{
  _Z_DEBUG("%s : start()", __func__);
  (void)subscription;
  (void)publisher_count;
  RMW_UROS_TRACE_MESSAGE(
    "Function not available; enable RMW_UXRCE_GRAPH configuration profile before using");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_subscription_get_actual_qos(
  const rmw_subscription_t * subscription,
  rmw_qos_profile_t * qos)
{
  _Z_DEBUG("%s : start()", __func__);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_destroy_subscription(
  rmw_node_t * node,
  rmw_subscription_t * subscription)
{
  _Z_DEBUG("%s : start()", __func__);
  rmw_ret_t result_ret = RMW_RET_OK;
  if (!node) {
    RMW_UROS_TRACE_MESSAGE("node handle is null")
      result_ret = RMW_RET_ERROR;
    result_ret = RMW_RET_ERROR;
  } else if (!node->data) {
    RMW_UROS_TRACE_MESSAGE("node imp is null")
      result_ret = RMW_RET_ERROR;
  } else if (!subscription) {
    RMW_UROS_TRACE_MESSAGE("subscription handle is null")
      result_ret = RMW_RET_ERROR;
    result_ret = RMW_RET_ERROR;
  } else if (!subscription->data) {
    RMW_UROS_TRACE_MESSAGE("subscription imp is null")
      result_ret = RMW_RET_ERROR;
  } else {
    rmw_subscription_destroy(subscription);
  }

  return result_ret;
}

rmw_ret_t
rmw_subscription_set_content_filter(
  rmw_subscription_t * subscription,
  const rmw_subscription_content_filter_options_t * options)
{
  _Z_DEBUG("%s : start()", __func__);
  (void) subscription;
  (void) options;

  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_subscription_get_content_filter(
  const rmw_subscription_t * subscription,
  rcutils_allocator_t * allocator,
  rmw_subscription_content_filter_options_t * options)
{
  _Z_DEBUG("%s : start()", __func__);
  (void) subscription;
  (void) allocator;
  (void) options;

  return RMW_RET_UNSUPPORTED;
}
