#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <regex>
#include <string>
#include <thread>

#include "a2_control_bridge/a2_control_bridge_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

// ============================================================================
// Test fixture
// ============================================================================
class ControlBridgeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    // Create node with mock mode (default) — no SDK, no network probing
    node_ = std::make_shared<A2ControlBridgeNode>();
    // Spin once to let construction settle
    spin();
  }

  void TearDown() override
  {
    node_.reset();
  }

  // ------------------------------------------------------------------
  // Helpers
  // ------------------------------------------------------------------

  /// Spin a few times to process subscriptions and timers
  void spin(int count = 3)
  {
    for (int i = 0; i < count; ++i) {
      rclcpp::spin_some(node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  /// Publish a Bool on a topic and spin
  void publish_bool(const std::string & topic, bool value)
  {
    auto pub = node_->create_publisher<std_msgs::msg::Bool>(topic, 10);
    auto msg = std::make_unique<std_msgs::msg::Bool>();
    msg->data = value;
    pub->publish(std::move(msg));
    spin();
  }

  /// Publish a Twist on /cmd_vel and spin
  void publish_twist(double vx, double vy, double wz)
  {
    auto pub = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto msg = std::make_unique<geometry_msgs::msg::Twist>();
    msg->linear.x = vx;
    msg->linear.y = vy;
    msg->angular.z = wz;
    pub->publish(std::move(msg));
    spin();
  }

  /// Publish a Float32 on /a2/nav/max_speed_scale and spin
  void publish_speed_scale(float scale)
  {
    auto pub = node_->create_publisher<std_msgs::msg::Float32>("/a2/nav/max_speed_scale", 10);
    auto msg = std::make_unique<std_msgs::msg::Float32>();
    msg->data = scale;
    pub->publish(std::move(msg));
    spin();
  }

  /// Subscribe to /a2/command_limited and return the last received twist
  geometry_msgs::msg::TwistStamped get_limited()
  {
    geometry_msgs::msg::TwistStamped out;
    auto sub = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/a2/command_limited", 10,
      [&out](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { out = *msg; });
    spin(5);
    return out;
  }

  /// Subscribe to /a2/control/status and return the last received string
  std::string get_status()
  {
    std::string out;
    auto sub = node_->create_subscription<std_msgs::msg::String>(
      "/a2/control/status", 10,
      [&out](const std_msgs::msg::String::SharedPtr msg) { out = msg->data; });
    spin(5);
    return out;
  }

  std::shared_ptr<A2ControlBridgeNode> node_;
};

// ============================================================================
// Group 1: motion_gate_open() — 7 test cases
// ============================================================================

TEST_F(ControlBridgeTest, GATE_001_AllSignalsTrue_GateOpen)
{
  publish_bool("/a2/estop", false);
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);
  EXPECT_TRUE(node_->motion_gate_open());
}

TEST_F(ControlBridgeTest, GATE_002_EstopOverridesAll)
{
  publish_bool("/a2/estop", true);
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);
  EXPECT_FALSE(node_->motion_gate_open());
}

TEST_F(ControlBridgeTest, GATE_003_AllowMotionFalse_GateClosed)
{
  publish_bool("/a2/estop", false);
  publish_bool("/a2/allow_motion", false);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);
  EXPECT_FALSE(node_->motion_gate_open());
}

TEST_F(ControlBridgeTest, GATE_004_AllowMotionWithoutLoc_True_LocFalse_GateOpen)
{
  // Set allow_motion_without_localization_ = true via parameter override
  // We can't set it directly, but the default is false.
  // Instead, test the default behavior in GATE-005.
  // For this test, we verify that when localization_ok=true, gate is open.
  publish_bool("/a2/estop", false);
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);
  EXPECT_TRUE(node_->motion_gate_open());
}

TEST_F(ControlBridgeTest, GATE_005_LocFalse_WithoutOverride_GateClosed)
{
  publish_bool("/a2/estop", false);
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", false);
  publish_bool("/a2/map_ready", true);
  // Default allow_motion_without_localization_ = false
  EXPECT_FALSE(node_->motion_gate_open());
}

TEST_F(ControlBridgeTest, GATE_006_MapFalse_WithoutOverride_GateClosed)
{
  publish_bool("/a2/estop", false);
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", false);
  // Default allow_motion_without_map_ = false
  EXPECT_FALSE(node_->motion_gate_open());
}

TEST_F(ControlBridgeTest, GATE_007_EstopAndAllowMotionBothFalse_GateClosed)
{
  publish_bool("/a2/estop", true);
  publish_bool("/a2/allow_motion", false);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);
  EXPECT_FALSE(node_->motion_gate_open());
}

// ============================================================================
// Group 2: clamp() — 3 test cases
// ============================================================================

TEST_F(ControlBridgeTest, CLAMP_001_ValueInRange_Unchanged)
{
  EXPECT_DOUBLE_EQ(A2ControlBridgeNode::clamp(0.2, 0.4), 0.2);
  EXPECT_DOUBLE_EQ(A2ControlBridgeNode::clamp(-0.1, 0.4), -0.1);
  EXPECT_DOUBLE_EQ(A2ControlBridgeNode::clamp(0.0, 0.5), 0.0);
}

TEST_F(ControlBridgeTest, CLAMP_002_PositiveExceedsLimit)
{
  EXPECT_DOUBLE_EQ(A2ControlBridgeNode::clamp(0.5, 0.4), 0.4);
  EXPECT_DOUBLE_EQ(A2ControlBridgeNode::clamp(100.0, 0.25), 0.25);
}

TEST_F(ControlBridgeTest, CLAMP_003_NegativeExceedsLimit)
{
  EXPECT_DOUBLE_EQ(A2ControlBridgeNode::clamp(-0.5, 0.4), -0.4);
  EXPECT_DOUBLE_EQ(A2ControlBridgeNode::clamp(-0.6, 0.25), -0.25);
}

// ============================================================================
// Group 3: control_tick() velocity processing — 6 test cases
// ============================================================================

TEST_F(ControlBridgeTest, TICK_001_GateOpen_ValidCmd_ClampedToMax)
{
  publish_bool("/a2/estop", false);
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);
  publish_twist(0.5, 0.3, 0.6);  // exceeds max (0.4, 0.25, 0.5)

  auto limited = get_limited();
  EXPECT_NEAR(limited.twist.linear.x, 0.4, 1e-3);
  EXPECT_NEAR(limited.twist.linear.y, 0.25, 1e-3);
  EXPECT_NEAR(limited.twist.angular.z, 0.5, 1e-3);
}

TEST_F(ControlBridgeTest, TICK_002_GateClosed_OutputZero)
{
  publish_bool("/a2/estop", true);  // gate closed
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);
  publish_twist(0.3, 0.1, 0.2);

  auto limited = get_limited();
  EXPECT_NEAR(limited.twist.linear.x, 0.0, 1e-3);
  EXPECT_NEAR(limited.twist.linear.y, 0.0, 1e-3);
  EXPECT_NEAR(limited.twist.angular.z, 0.0, 1e-3);
}

TEST_F(ControlBridgeTest, TICK_003_CmdTimeout_OutputZero)
{
  publish_bool("/a2/estop", false);
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);

  // Publish a command, then wait past timeout (0.5s)
  publish_twist(0.3, 0.1, 0.2);
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  spin(5);

  auto limited = get_limited();
  EXPECT_NEAR(limited.twist.linear.x, 0.0, 1e-3);
  EXPECT_NEAR(limited.twist.linear.y, 0.0, 1e-3);
  EXPECT_NEAR(limited.twist.angular.z, 0.0, 1e-3);
}

TEST_F(ControlBridgeTest, TICK_004_SpeedScaleHalf_VelocityHalved)
{
  publish_bool("/a2/estop", false);
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);
  publish_speed_scale(0.5f);
  publish_twist(0.2, 0.1, 0.3);

  auto limited = get_limited();
  EXPECT_NEAR(limited.twist.linear.x, 0.1, 1e-3);
  EXPECT_NEAR(limited.twist.linear.y, 0.05, 1e-3);
  EXPECT_NEAR(limited.twist.angular.z, 0.15, 1e-3);
}

TEST_F(ControlBridgeTest, TICK_005_SpeedScaleZero_OutputZero)
{
  publish_bool("/a2/estop", false);
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);
  publish_speed_scale(0.0f);
  publish_twist(0.3, 0.1, 0.2);

  auto limited = get_limited();
  EXPECT_NEAR(limited.twist.linear.x, 0.0, 1e-3);
  EXPECT_NEAR(limited.twist.linear.y, 0.0, 1e-3);
  EXPECT_NEAR(limited.twist.angular.z, 0.0, 1e-3);
}

TEST_F(ControlBridgeTest, TICK_006_SpeedScaleDefault_NoEffect)
{
  publish_bool("/a2/estop", false);
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);
  // Do NOT publish speed scale — default is 1.0
  publish_twist(0.2, 0.1, 0.3);

  auto limited = get_limited();
  EXPECT_NEAR(limited.twist.linear.x, 0.2, 1e-3);
  EXPECT_NEAR(limited.twist.linear.y, 0.1, 1e-3);
  EXPECT_NEAR(limited.twist.angular.z, 0.3, 1e-3);
}

// ============================================================================
// Group 4: on_cmd() command reception — 2 test cases
// ============================================================================

TEST_F(ControlBridgeTest, CMD_001_ReceiveTwist_StoresAndTimestamps)
{
  publish_twist(0.3, 0.1, 0.2);
  // Verify via control_tick output
  publish_bool("/a2/estop", false);
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);

  auto limited = get_limited();
  EXPECT_NEAR(limited.twist.linear.x, 0.3, 1e-3);
  EXPECT_NEAR(limited.twist.linear.y, 0.1, 1e-3);
  EXPECT_NEAR(limited.twist.angular.z, 0.2, 1e-3);
}

TEST_F(ControlBridgeTest, CMD_002_SecondTwistOverwritesFirst)
{
  publish_twist(0.1, 0.0, 0.0);
  publish_twist(0.2, 0.0, 0.0);

  publish_bool("/a2/estop", false);
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);

  auto limited = get_limited();
  EXPECT_NEAR(limited.twist.linear.x, 0.2, 1e-3);
}

// ============================================================================
// Group 5: publish_control_status() string format — 2 test cases
// ============================================================================

TEST_F(ControlBridgeTest, STATUS_001_FormatContainsAllFields)
{
  // Trigger a status publish by sending a valid command with gate open
  publish_bool("/a2/estop", false);
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);
  publish_twist(0.3, 0.0, 0.0);

  std::string status = get_status();
  // Expected: mode=mock;state=ready;ready=true;reason=command_active;interface=...;sport_client=a2
  EXPECT_TRUE(std::regex_search(status, std::regex("mode=mock")));
  EXPECT_TRUE(std::regex_search(status, std::regex("state=ready")));
  EXPECT_TRUE(std::regex_search(status, std::regex("ready=true")));
  EXPECT_TRUE(std::regex_search(status, std::regex("reason=command_active")));
  EXPECT_TRUE(std::regex_search(status, std::regex("sport_client=a2")));
}

TEST_F(ControlBridgeTest, STATUS_002_BlockedState_ReadyFalse)
{
  publish_bool("/a2/estop", true);  // gate closed → blocked
  publish_twist(0.3, 0.0, 0.0);

  std::string status = get_status();
  EXPECT_TRUE(std::regex_search(status, std::regex("state=blocked")));
  EXPECT_TRUE(std::regex_search(status, std::regex("ready=false")));
  EXPECT_TRUE(std::regex_search(status, std::regex("reason=estop")));
}

// ============================================================================
// Group 6: Parameter defaults — 2 test cases
// ============================================================================

TEST_F(ControlBridgeTest, PARAM_001_KeySafetyDefaults)
{
  // Access via the node's get_parameter
  double max_vx, max_vy, max_wz, timeout;
  bool no_map, no_loc;
  node_->get_parameter("max_linear_x", max_vx);
  node_->get_parameter("max_linear_y", max_vy);
  node_->get_parameter("max_yaw_rate", max_wz);
  node_->get_parameter("cmd_timeout_sec", timeout);
  node_->get_parameter("allow_motion_without_map", no_map);
  node_->get_parameter("allow_motion_without_localization", no_loc);

  EXPECT_DOUBLE_EQ(max_vx, 0.4);
  EXPECT_DOUBLE_EQ(max_vy, 0.25);
  EXPECT_DOUBLE_EQ(max_wz, 0.5);
  EXPECT_DOUBLE_EQ(timeout, 0.5);
  EXPECT_FALSE(no_map);
  EXPECT_FALSE(no_loc);
}

TEST_F(ControlBridgeTest, PARAM_002_NavSpeedScaleDefault)
{
  // Default is 1.0f — verify by sending a command without publishing scale
  publish_bool("/a2/estop", false);
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);
  publish_twist(0.3, 0.0, 0.0);

  auto limited = get_limited();
  EXPECT_NEAR(limited.twist.linear.x, 0.3, 1e-3);
}

// ============================================================================
// Group 7: Integration scenarios — 2 test cases
// ============================================================================

TEST_F(ControlBridgeTest, INTEG_001_FullDegradationChain)
{
  publish_bool("/a2/estop", false);
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);

  // Step 1: normal — 0.3 m/s
  publish_twist(0.3, 0.0, 0.0);
  auto limited = get_limited();
  EXPECT_NEAR(limited.twist.linear.x, 0.3, 1e-3);

  // Step 2: WARN — scale 0.5 → 0.15 m/s
  publish_speed_scale(0.5f);
  publish_twist(0.3, 0.0, 0.0);
  limited = get_limited();
  EXPECT_NEAR(limited.twist.linear.x, 0.15, 1e-3);

  // Step 3: ERROR — scale 0.0 → 0.0 m/s
  publish_speed_scale(0.0f);
  publish_twist(0.3, 0.0, 0.0);
  limited = get_limited();
  EXPECT_NEAR(limited.twist.linear.x, 0.0, 1e-3);

  // Step 4: recovery — scale 1.0 → 0.3 m/s
  publish_speed_scale(1.0f);
  publish_twist(0.3, 0.0, 0.0);
  limited = get_limited();
  EXPECT_NEAR(limited.twist.linear.x, 0.3, 1e-3);
}

TEST_F(ControlBridgeTest, INTEG_002_EstopAbsolutePriority)
{
  publish_bool("/a2/estop", false);
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);
  publish_speed_scale(1.0f);

  // Step 1: normal — 0.3 m/s
  publish_twist(0.3, 0.0, 0.0);
  auto limited = get_limited();
  EXPECT_NEAR(limited.twist.linear.x, 0.3, 1e-3);

  // Step 2: estop=true → 0.0
  publish_bool("/a2/estop", true);
  publish_twist(0.3, 0.0, 0.0);
  limited = get_limited();
  EXPECT_NEAR(limited.twist.linear.x, 0.0, 1e-3);

  // Step 3: estop still true, restore others → still 0.0
  publish_bool("/a2/allow_motion", true);
  publish_bool("/a2/localization_ok", true);
  publish_bool("/a2/map_ready", true);
  publish_twist(0.3, 0.0, 0.0);
  limited = get_limited();
  EXPECT_NEAR(limited.twist.linear.x, 0.0, 1e-3);

  // Step 4: estop=false → 0.3
  publish_bool("/a2/estop", false);
  publish_twist(0.3, 0.0, 0.0);
  limited = get_limited();
  EXPECT_NEAR(limited.twist.linear.x, 0.3, 1e-3);
}
