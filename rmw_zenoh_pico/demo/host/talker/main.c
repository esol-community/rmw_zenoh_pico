// -*- tab-width : 8 , c-indentation-style : bsd -*-
/*
 * Copyright (C)
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
#include <unistd.h>

#define ARRAY_LEN 200

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
std_msgs__msg__String msg;

int counter = 0;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
	    sprintf(msg.data.data, "Hello World: %d", counter++);
		msg.data.size = strlen(msg.data.data);
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		printf("I have publish: \"%s\"\n", msg.data.data);
	}
}

int main(int argc, const char * const * argv)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, argc, argv, &allocator));

	// create node
	rcl_node_t node;

	char *domain_id_ptr;
		if((domain_id_ptr = getenv("ROS_DOMAIN_ID")) != NULL){
		char *endl;
		int domain_id = strtol(domain_id_ptr, &endl, 10);

		if(endl != '\0'){
			rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
			rcl_init_options_init(&init_options, allocator);
			RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));
			RCCHECK(rclc_support_init_with_options(&support,
							       0,
							       NULL,
							       &init_options,
							       &allocator));
		}
	}

	RCCHECK(rclc_node_init_default(&node, "talker_node", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
		"/chatter"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	// Fill the array with a known sequence
	msg.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
	msg.data.size = 0;
	msg.data.capacity = ARRAY_LEN;

	rclc_executor_spin(&executor);

	RCCHECK(rcl_publisher_fini(&publisher, &node))
	RCCHECK(rcl_node_fini(&node))
}
