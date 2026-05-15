#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "a2_interfaces/msg/robot_state.hpp"
#include "a2_system/network_utils.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#if A2_ENABLE_UNITREE_SDK
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#endif

class A2SdkBridgeNode : public rclcpp::Node
{
public:
  A2SdkBridgeNode()
  : Node("a2_sdk_bridge")
  {
    use_mock_ = declare_parameter<bool>("use_mock", true);
    robot_profile_ = declare_parameter<std::string>("robot_profile", "");
    robot_model_ = declare_parameter<std::string>("robot_model", "");
    auto_detect_interface_ = declare_parameter<bool>("auto_detect_interface", true);
    allow_loopback_ = declare_parameter<bool>("allow_loopback", true);
    network_interface_ = declare_parameter<std::string>("network_interface", "");
    interface_candidates_ = declare_parameter<std::vector<std::string>>(
      "interface_candidates", std::vector<std::string>{});
    state_topic_ = declare_parameter<std::string>("state_topic", "/a2/raw_state");
    sport_state_topic_ = declare_parameter<std::string>("sport_state_topic", "rt/lf/sportmodestate");
    timer_hz_ = declare_parameter<double>("timer_hz", 50.0);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.5);
    mock_cmd_topic_ = declare_parameter<std::string>("mock_cmd_topic", "/cmd_vel");
    mock_cmd_timeout_sec_ = declare_parameter<double>("mock_cmd_timeout_sec", 0.5);
    mock_linear_speed_ = declare_parameter<double>("mock_linear_speed", 0.1);
    mock_yaw_rate_ = declare_parameter<double>("mock_yaw_rate", 0.15);

    state_pub_ = create_publisher<a2_interfaces::msg::RobotState>(state_topic_, 20);
    sdk_connected_pub_ = create_publisher<std_msgs::msg::Bool>("/a2/sdk/connected", 10);
    sdk_status_pub_ = create_publisher<std_msgs::msg::String>("/a2/sdk/status", 10);
    resolved_interface_ = resolve_interface();

    if (use_mock_) {
      RCLCPP_INFO(
        get_logger(),
        "Starting in mock mode. interface='%s', candidates=%s",
        resolved_interface_.c_str(),
        a2_system::describe_interfaces().c_str());
      mock_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        mock_cmd_topic_, 20,
        std::bind(&A2SdkBridgeNode::on_mock_cmd, this, std::placeholders::_1));
      const auto period = std::chrono::duration<double>(1.0 / std::max(timer_hz_, 1.0));
      mock_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&A2SdkBridgeNode::publish_mock_state, this));
      watchdog_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&A2SdkBridgeNode::watchdog_tick, this));
      return;
    }

#if A2_ENABLE_UNITREE_SDK
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&A2SdkBridgeNode::watchdog_tick, this));
    last_state_time_ = now();
    if (resolved_interface_.empty()) {
      sdk_interface_ready_ = false;
      RCLCPP_ERROR(get_logger(), "No usable network interface found for SDK mode.");
      publish_sdk_status(false, "no_interface");
      return;
    }
    if (!a2_system::interface_is_ready_for_real(resolved_interface_)) {
      sdk_interface_ready_ = false;
      RCLCPP_WARN(
        get_logger(),
        "Interface '%s' is not ready for real A2 traffic. State bridge will stay armed but disconnected.",
        resolved_interface_.c_str());
      publish_sdk_status(false, "interface_not_ready");
      return;
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, resolved_interface_);
    suber_ = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(sport_state_topic_);
    suber_->InitChannel(std::bind(&A2SdkBridgeNode::on_sport_state, this, std::placeholders::_1), 1);
    RCLCPP_INFO(
      get_logger(), "SDK mode armed on interface '%s', topic '%s'.",
      resolved_interface_.c_str(), sport_state_topic_.c_str());
    publish_sdk_status(false, "waiting_for_a2_state");
#else
    RCLCPP_ERROR(get_logger(), "This binary was built without unitree_sdk2. Rebuild with UNITREE_SDK2_ROOT available or use mock mode.");
    publish_sdk_status(false, "sdk_library_missing");
#endif
  }

private:
  static builtin_interfaces::msg::Time to_builtin_time(const rclcpp::Time & time)
  {
    builtin_interfaces::msg::Time stamp;
    const auto nanoseconds = time.nanoseconds();
    stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return stamp;
  }

  std::string resolve_interface() const
  {
    const bool allow_loopback = use_mock_ && allow_loopback_;
    if (!network_interface_.empty() && a2_system::interface_exists(network_interface_)) {
      return network_interface_;
    }
    if (auto_detect_interface_) {
      return a2_system::select_interface(network_interface_, interface_candidates_, allow_loopback);
    }
    return network_interface_;
  }

  void publish_mock_state()
  {
    const auto current_time = now();
    const double dt = last_mock_update_.nanoseconds() == 0 ?
      1.0 / std::max(timer_hz_, 1.0) :
      std::max(1e-3, (current_time - last_mock_update_).seconds());
    last_mock_update_ = current_time;
    mock_elapsed_sec_ += dt;

    double cmd_x = 0.0;
    double cmd_y = 0.0;
    double cmd_yaw = 0.0;
    if ((current_time - last_mock_cmd_time_).seconds() <= mock_cmd_timeout_sec_) {
      mock_external_command_ = true;
      cmd_x = mock_cmd_.linear.x;
      cmd_y = mock_cmd_.linear.y;
      cmd_yaw = mock_cmd_.angular.z;
    }

    if (mock_external_command_) {
      mock_yaw_ += cmd_yaw * dt;
      mock_x_ += (std::cos(mock_yaw_) * cmd_x - std::sin(mock_yaw_) * cmd_y) * dt;
      mock_y_ += (std::sin(mock_yaw_) * cmd_x + std::cos(mock_yaw_) * cmd_y) * dt;
    } else {
      mock_x_ = std::sin(mock_elapsed_sec_ * 0.1) * 0.5;
      mock_y_ = std::cos(mock_elapsed_sec_ * 0.07) * 0.3;
      mock_yaw_ = mock_elapsed_sec_ * mock_yaw_rate_ * 0.2;
      cmd_x = mock_linear_speed_;
      cmd_yaw = mock_yaw_rate_;
    }
    mock_yaw_ = std::atan2(std::sin(mock_yaw_), std::cos(mock_yaw_));

    a2_interfaces::msg::RobotState msg;
    msg.stamp = to_builtin_time(current_time);
    msg.source_mode = "mock";
    msg.frame_id = "base_link";
    msg.connected = true;
    msg.imu_valid = true;
    msg.odom_valid = true;
    msg.position[0] = static_cast<float>(mock_x_);
    msg.position[1] = static_cast<float>(mock_y_);
    msg.position[2] = 0.28F;
    msg.velocity[0] = static_cast<float>(cmd_x);
    msg.velocity[1] = static_cast<float>(cmd_y);
    msg.velocity[2] = 0.0F;
    msg.rpy[0] = static_cast<float>(std::sin(mock_elapsed_sec_ * 0.5) * 0.02);
    msg.rpy[1] = static_cast<float>(std::cos(mock_elapsed_sec_ * 0.4) * 0.02);
    msg.rpy[2] = static_cast<float>(mock_yaw_);
    msg.orientation_xyzw[0] = 0.0F;
    msg.orientation_xyzw[1] = 0.0F;
    msg.orientation_xyzw[2] = static_cast<float>(std::sin(msg.rpy[2] * 0.5));
    msg.orientation_xyzw[3] = static_cast<float>(std::cos(msg.rpy[2] * 0.5));
    msg.linear_acceleration[0] = 0.0F;
    msg.linear_acceleration[1] = 0.0F;
    msg.linear_acceleration[2] = 9.81F;
    msg.angular_velocity[2] = static_cast<float>(cmd_yaw);
    msg.body_height = 0.28F;
    msg.yaw_speed = static_cast<float>(cmd_yaw);
    msg.motion_mode = 1U;
    msg.progress = 0.0F;
    msg.gait_type = 1U;
    state_pub_->publish(msg);
    last_state_time_ = current_time;
    publish_sdk_status(true, "mock_state_ok");
  }

#if A2_ENABLE_UNITREE_SDK
  void on_sport_state(const void * message)
  {
    const auto & state = *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);
    const auto current_time = now();

    a2_interfaces::msg::RobotState msg;
    msg.stamp = to_builtin_time(current_time);
    msg.source_mode = "real";
    msg.frame_id = "base_link";
    msg.connected = true;
    msg.imu_valid = true;
    msg.odom_valid = true;
    for (std::size_t index = 0; index < 3U; ++index) {
      msg.position[index] = state.position()[index];
      msg.velocity[index] = state.velocity()[index];
      msg.rpy[index] = state.imu_state().rpy()[index];
      msg.linear_acceleration[index] = state.imu_state().accelerometer()[index];
      msg.angular_velocity[index] = state.imu_state().gyroscope()[index];
    }
    for (std::size_t index = 0; index < 4U; ++index) {
      msg.orientation_xyzw[index] = state.imu_state().quaternion()[index];
    }
    msg.body_height = state.body_height();
    msg.yaw_speed = state.yaw_speed();
    msg.motion_mode = state.mode();
    msg.progress = state.progress();
    msg.gait_type = state.gait_type();
    last_state_time_ = current_time;
    state_pub_->publish(msg);
    publish_sdk_status(true, "a2_state_ok");
  }
#endif

  void watchdog_tick()
  {
    if (!use_mock_ && !sdk_interface_ready_) {
      const std::string reason = resolved_interface_.empty() ? "no_interface" : "interface_not_ready";
      publish_sdk_status(false, reason);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "A2 SDK bridge is waiting for a ready wired interface. selected='%s' visible=%s",
        resolved_interface_.empty() ? "none" : resolved_interface_.c_str(),
        a2_system::describe_interfaces().c_str());
      return;
    }

    if (last_state_time_.nanoseconds() == 0) {
      publish_sdk_status(false, use_mock_ ? "mock_not_initialized" : "waiting_for_a2_state");
      return;
    }

    const auto age = (now() - last_state_time_).seconds();
    if (age > stale_timeout_sec_) {
      std::string reason = use_mock_ ? "mock_state_stale" : "a2_state_stale";
      if (!use_mock_ && !a2_system::interface_is_ready_for_real(resolved_interface_)) {
        reason = "interface_not_ready";
      }
      publish_sdk_status(false, reason);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "A2 sport state stale for %.2f sec on interface '%s'. Visible interfaces: %s",
        age,
        resolved_interface_.c_str(),
        a2_system::describe_interfaces().c_str());
      return;
    }
    if (!resolved_interface_.empty()) {
      publish_sdk_status(true, use_mock_ ? "mock_state_ok" : "a2_state_ok");
    }
  }

  void on_mock_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    mock_cmd_ = *msg;
    last_mock_cmd_time_ = now();
  }

  void publish_sdk_status(bool connected, const std::string & status)
  {
    std_msgs::msg::Bool connected_msg;
    connected_msg.data = connected;
    sdk_connected_pub_->publish(connected_msg);

    std_msgs::msg::String status_msg;
    const std::string mode = use_mock_ ? "mock" : "real";
    std::string state = connected ? "ready" : "waiting";
    if (status == "no_interface" || status == "interface_not_ready") {
      state = "waiting_interface";
    } else if (status == "sdk_library_missing") {
      state = "error";
    } else if (status == "mock_state_stale" || status == "a2_state_stale") {
      state = "stale";
    } else if (status == "waiting_for_a2_state" || status == "mock_not_initialized") {
      state = "waiting_state";
    }
    status_msg.data =
      "mode=" + mode +
      ";state=" + state +
      ";ready=" + std::string(connected ? "true" : "false") +
      ";reason=" + status +
      ";interface=" + (resolved_interface_.empty() ? "none" : resolved_interface_) +
      ";sport_state_topic=" + sport_state_topic_ +
      ";robot_profile=" + robot_profile_ +
      ";robot_model=" + robot_model_;
    sdk_status_pub_->publish(status_msg);
  }

  bool use_mock_{true};
  bool auto_detect_interface_{true};
  bool allow_loopback_{true};
  std::string network_interface_;
  std::vector<std::string> interface_candidates_;
  std::string robot_profile_;
  std::string robot_model_;
  std::string state_topic_;
  std::string sport_state_topic_;
  std::string mock_cmd_topic_;
  std::string resolved_interface_;
  double timer_hz_{50.0};
  double stale_timeout_sec_{0.5};
  double mock_cmd_timeout_sec_{0.5};
  bool sdk_interface_ready_{true};
  double mock_linear_speed_{0.1};
  double mock_yaw_rate_{0.15};
  double mock_x_{0.0};
  double mock_y_{0.0};
  double mock_yaw_{0.0};
  double mock_elapsed_sec_{0.0};
  bool mock_external_command_{false};

  rclcpp::Publisher<a2_interfaces::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sdk_connected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sdk_status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mock_cmd_sub_;
  rclcpp::TimerBase::SharedPtr mock_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_mock_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_mock_cmd_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist mock_cmd_;

#if A2_ENABLE_UNITREE_SDK
  std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> suber_;
#endif
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<A2SdkBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
