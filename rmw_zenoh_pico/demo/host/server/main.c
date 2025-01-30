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

#include <stdio.h>
#include <unistd.h>

#define RCCHECK(fn) {                                                   \
        rcl_ret_t temp_rc = fn;                                         \
        if((temp_rc != RCL_RET_OK))                                     \
        {                                                               \
            printf("Failed status on line %d: %d. Aborting.\n",         \
                   __LINE__,(int)temp_rc);                              \
        }                                                               \
    }

#define RCSOFTCHECK(fn) {                                               \
        rcl_ret_t temp_rc = fn;                                         \
        if((temp_rc != RCL_RET_OK))                                     \
        {                                                               \
            printf("Failed status on line %d: %d. Continuing.\n",       \
                   __LINE__,(int)temp_rc);                              \
        }                                                               \
    }

void service_callback(const void * req, void * res){
  example_interfaces__srv__AddTwoInts_Request * req_in = (example_interfaces__srv__AddTwoInts_Request *) req;
  example_interfaces__srv__AddTwoInts_Response * res_in = (example_interfaces__srv__AddTwoInts_Response *) res;

  res_in->sum = req_in->a + req_in->b;

  printf("Incoming request [a:%ld + b:%ld] => %ld\n", req_in->a, req_in->b, res_in->sum);
}

int main(int argc, char *argv[])
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // create init_options
    rclc_support_t support;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;

    char *domain_id_ptr;
    if((domain_id_ptr = getenv("ROS_DOMAIN_ID")) != NULL){
      char *endl;
      int domain_id = strtol(domain_id_ptr, &endl, 10);

      if(endl != NULL){
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
	RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));
	RCCHECK(rclc_support_init_with_options(&support,
					       0,
					       NULL,
					       &init_options,
					       &allocator));
      }
    }

    RCCHECK(rclc_node_init_default(&node,
				   "add_two_ints_server_rclc",
				   "",
				   &support));

    // create service
    rcl_service_t service;
    RCCHECK(rclc_service_init_default(
                &service,
                &node,
                ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts),
                "add_two_ints"));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor,
                               &support.context,
                               1,
                               &allocator));

    example_interfaces__srv__AddTwoInts_Response res;
    example_interfaces__srv__AddTwoInts_Request req;
    RCCHECK(rclc_executor_add_service(&executor,
                                      &service,
                                      &req,
                                      &res,
                                      service_callback));

    rclc_executor_spin(&executor);

    RCCHECK(rcl_service_fini(&service, &node));
    RCCHECK(rcl_node_fini(&node));

    return(0);
}
