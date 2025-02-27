// -*- tab-width : 8 , c-indentation-style : bsd -*-
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

#include "example_interfaces/srv/add_two_ints.h"
#include "rcl/time.h"

#include <stdio.h>
#include <unistd.h>

#define USE_ROS_DOMAIN_ID
#ifdef USE_ROS_DOMAIN_ID
#define ROS_DOMAIN_ID 64
#endif

#define RCCHECK(fn) {                                           \
    rcl_ret_t temp_rc = fn;					\
    if((temp_rc != RCL_RET_OK))					\
      {								\
	printf("Failed status on line %d: %d. Aborting.\n",	\
	       __LINE__,(int)temp_rc);				\
	return 1;						\
      }								\
  }

#define RCSOFTCHECK(fn) {					\
    rcl_ret_t temp_rc = fn;					\
    if((temp_rc != RCL_RET_OK)){				\
      printf("Failed status on line %d: %d. Continuing.\n",	\
	     __LINE__,(int)temp_rc);				\
    }								\
  }

example_interfaces__srv__AddTwoInts_Request req;
example_interfaces__srv__AddTwoInts_Response res;

rcl_node_t node;
rcl_client_t client;

void client_callback(const void * msg) {
  example_interfaces__srv__AddTwoInts_Response * msgin = (example_interfaces__srv__AddTwoInts_Response * ) msg;

  printf("Result of add_two_ints: %lld + %lld = %lld\n", req.a, req.b, msgin->sum);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;

  if (timer != NULL) {
    int64_t seq;

    static int64_t val = 0x10;
    req.a = val;
    req.b = val;
    (void)rcl_send_request(&client, &req, &seq);
    val++;
  }
}

int main(int argc, const char * argv[])
{
  RCLC_UNUSED(argc);
  RCLC_UNUSED(argv);
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  // create init_options
  RCCHECK(rclc_support_init(&support,
			    0, NULL,
			    &allocator));

  // create node
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
				 "add_two_ints_client_rclc",
				 "",
				 &support));

  // create client
  RCCHECK(rclc_client_init_default(&client,
				   &node,
				   ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts),
				   "add_two_ints"));
  // create executor
  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor,
			     &support.context,
			     2,
			     &allocator));

  RCCHECK(rclc_executor_add_client(&executor,
				   &client,
				   &res,
				   client_callback));
  // create timer,
  rcl_timer_t timer;
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
	    &timer,
	    &support,
	    RCL_MS_TO_NS(timer_timeout),
	    timer_callback));

  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // init topic data
  example_interfaces__srv__AddTwoInts_Request__init(&req);

  rclc_executor_spin(&executor);

  RCCHECK(rcl_client_fini(&client, &node));
  RCCHECK(rcl_node_fini(&node));
}
