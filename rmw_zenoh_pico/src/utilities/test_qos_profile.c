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

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

void test_qos_profile(rmw_qos_profile_t *qos) {
  // Reliability.
  qos->reliability	= RMW_QOS_POLICY_RELIABILITY_RELIABLE; // 1;
  // Durability.
  qos->durability	= RMW_QOS_POLICY_DURABILITY_VOLATILE;  // 2;
  // History.
  qos->history	        = RMW_QOS_POLICY_HISTORY_KEEP_LAST;    // 1;
  qos->depth		= 10;
  // Deadline.
  qos->deadline.sec	= 0;
  qos->deadline.nsec	= 0;
  // Lifespan.
  qos->lifespan.sec	= 0;
  qos->lifespan.nsec	= 0;
  // Liveliness.
  qos->liveliness       = 0;
  qos->liveliness_lease_duration.sec = 0;
  qos->liveliness_lease_duration.nsec = 0;
}
