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

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>

#include <stdio.h>

#define USE_ROS_DOMAIN_ID
#ifdef USE_ROS_DOMAIN_ID
#define ROS_DOMAIN_ID 64
#endif

#define ARRAY_LEN 200

#define RCCHECK(fn) {                                           \
    rcl_ret_t temp_rc = fn;					\
    if((temp_rc != RCL_RET_OK))	{				\
	printf("Failed status on line %d: %d. Aborting.\n",	\
	       __LINE__,(int)temp_rc);				\
	return 1;						\
      }								\
  }

#define RCSOFTCHECK(fn) {					\
    rcl_ret_t temp_rc = fn;					\
    if((temp_rc != RCL_RET_OK)) {				\
      printf("Failed status on line %d: %d. Continuing.\n",	\
	     __LINE__,(int)temp_rc);				\
    }								\
  }

rcl_subscription_t subscriber;
std_msgs__msg__String msg;
char test_array[ARRAY_LEN];

void subscription_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  printf("I have heard: \"%s\"\n", msg->data.data);
}

int main(int argc, const char * const * argv)
{
  memset(test_array,'z',ARRAY_LEN);

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  // create init_options
  RCCHECK(rclc_support_init(&support, argc, argv, &allocator));

  // create node
  rcl_node_t node;

#ifdef USE_ROS_DOMAIN_ID
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
  RCCHECK(rclc_support_init_with_options(&support,
					 0,
					 NULL,
					 &init_options, &allocator));
#endif

  RCCHECK(rclc_node_init_default(&node,
				 "listener_node",
				 "",
				 &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
	    &subscriber,
	    &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
	    "chatter"));

  // create executor
  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(
	    &executor,
	    &support.context,
	    1,
	    &allocator));
  RCCHECK(rclc_executor_add_subscription(
	    &executor,
	    &subscriber,
	    &msg,
	    &subscription_callback,
	    ON_NEW_DATA));

  msg.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
  msg.data.size = 0;
  msg.data.capacity = ARRAY_LEN;

  rclc_executor_spin(&executor);

  RCCHECK(rcl_subscription_fini(&subscriber, &node));
  RCCHECK(rcl_node_fini(&node));
}
