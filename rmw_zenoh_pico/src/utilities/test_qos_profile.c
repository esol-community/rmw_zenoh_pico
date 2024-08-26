
#include <rmw/rmw.h>
#include <rmw/types.h>
#include <rmw/allocators.h>
#include <rmw/error_handling.h>

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
