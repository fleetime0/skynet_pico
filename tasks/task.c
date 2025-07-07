#include "task.h"

#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"

#include "my_micro_ros.h"

rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;

bool task_init(void) {
  my_micro_ros_init();
  allocator = rcl_get_default_allocator();

  // Wait for agent successful ping for 2 minutes.
  const int timeout_ms = 1000;
  const uint8_t attempts = 120;

  rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

  if (ret != RCL_RET_OK) {
    // Unreachable agent, exiting program.
    return false;
  }

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, NODE_NAME, "", &support);
}
