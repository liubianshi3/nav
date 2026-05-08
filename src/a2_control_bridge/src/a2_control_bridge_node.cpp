#include "a2_control_bridge/a2_control_bridge_node.hpp"

#ifndef A2_CONTROL_BRIDGE_TEST_BUILD
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<A2ControlBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
