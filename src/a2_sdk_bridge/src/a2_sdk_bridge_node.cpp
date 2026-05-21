#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "a2_interfaces/msg/robot_state.hpp"
#include "a2_unitree_ipc/client.hpp"
#include "a2_unitree_ipc/protocol.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

class A2SdkBridgeNode : public rclcpp::Node
{
public:
  A2SdkBridgeNode()
  : Node("a2_sdk_bridge")
  {
    use_mock_ = declare_parameter<bool>("use_mock", true);
    robot_profile_ = declare_parameter<std::string>("robot_profile", "");
    robot_model_ = declare_parameter<std::string>("robot_model", "");
    state_topic_ = declare_parameter<std::string>("state_topic", "/a2/raw_state");
    battery_topic_ = declare_parameter<std::string>("battery_topic", "/a2/battery");
    status_topic_ = declare_parameter<std::string>("status_topic", "/a2/status");
    ipc_socket_path_ = declare_parameter<std::string>("ipc_socket_path", a2_unitree_ipc::kDefaultSocketPath);
    ipc_timeout_ms_ = declare_parameter<int>("ipc_timeout_ms", 100);
    timer_hz_ = declare_parameter<double>("timer_hz", 50.0);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.5);
    battery_publish_hz_ = std::max(0.2, declare_parameter<double>("battery_publish_hz", 1.0));
    battery_stale_timeout_sec_ = std::max(0.5, declare_parameter<double>("battery_stale_timeout_sec", 5.0));
    declare_parameter<bool>("auto_detect_interface", true);
    declare_parameter<bool>("allow_loopback", false);
    declare_parameter<std::string>("network_interface", "");
    declare_parameter<std::vector<std::string>>("interface_candidates", std::vector<std::string>{});
    declare_parameter<std::string>("sport_state_topic", "rt/lf/sportmodestate");
    declare_parameter<std::vector<std::string>>(
      "low_state_topic_candidates",
      std::vector<std::string>{"rt/lf/lowstate", "rt/lowstate", "lf/lowstate", "lowstate"});
    declare_parameter<std::string>("low_state_topic", "rt/lf/lowstate");
    declare_parameter<std::vector<std::string>>(
      "bms_state_topic_candidates",
      std::vector<std::string>{"lf/bmsstate", "rt/lf/bmsstate", "rt/bmsstate", "bmsstate"});
    declare_parameter<std::string>("bms_state_topic", "lf/bmsstate");
    mock_battery_percent_ = std::max(
      0.0,
      std::min(100.0, declare_parameter<double>("mock_battery_percent", 85.0)));
    mock_cmd_topic_ = declare_parameter<std::string>("mock_cmd_topic", "/cmd_vel");
    mock_cmd_timeout_sec_ = declare_parameter<double>("mock_cmd_timeout_sec", 0.5);
    mock_linear_speed_ = declare_parameter<double>("mock_linear_speed", 0.1);
    mock_yaw_rate_ = declare_parameter<double>("mock_yaw_rate", 0.15);

    state_pub_ = create_publisher<a2_interfaces::msg::RobotState>(state_topic_, 20);
    sdk_connected_pub_ = create_publisher<std_msgs::msg::Bool>("/a2/sdk/connected", 10);
    sdk_status_pub_ = create_publisher<std_msgs::msg::String>("/a2/sdk/status", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>(battery_topic_, 10);

    if (use_mock_) {
      RCLCPP_INFO(get_logger(), "Starting a2_sdk_bridge in mock mode.");
      mock_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        mock_cmd_topic_, 20,
        std::bind(&A2SdkBridgeNode::on_mock_cmd, this, std::placeholders::_1));
    } else {
      RCLCPP_INFO(
        get_logger(), "Starting a2_sdk_bridge as UDS client. socket='%s'", ipc_socket_path_.c_str());
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(timer_hz_, 1.0));
    if (use_mock_) {
      state_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&A2SdkBridgeNode::publish_mock_state, this));
    } else {
      state_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&A2SdkBridgeNode::poll_agent_state, this));
    }

    const auto battery_period = std::chrono::duration<double>(1.0 / battery_publish_hz_);
    battery_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(battery_period),
      std::bind(&A2SdkBridgeNode::publish_battery_state, this));
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&A2SdkBridgeNode::watchdog_tick, this));
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

  a2_unitree_ipc::UnixSocketClient & ipc_client()
  {
    if (!ipc_client_) {
      ipc_client_ = std::make_unique<a2_unitree_ipc::UnixSocketClient>(ipc_socket_path_, ipc_timeout_ms_);
    }
    return *ipc_client_;
  }

  bool ensure_agent_subscription(std::string * error_message)
  {
    auto & client = ipc_client();
    if (!client.ensure_connected(error_message)) {
      return false;
    }
    if (state_subscribed_) {
      return true;
    }
    if (!client.send_message(a2_unitree_ipc::encode_state_subscribe(), error_message)) {
      client.close();
      return false;
    }
    state_subscribed_ = true;
    return true;
  }

  void poll_agent_state()
  {
    std::lock_guard<std::mutex> guard(ipc_mutex_);
    std::string error;
    if (!ensure_agent_subscription(&error)) {
      state_subscribed_ = false;
      publish_sdk_status(false, "ipc_unavailable:" + error);
      return;
    }

    std::string line;
    if (!ipc_client().read_message(&line, 0, &error)) {
      if (error == "read timeout") {
        return;
      }
      state_subscribed_ = false;
      ipc_client().close();
      publish_sdk_status(false, "ipc_read_failed:" + error);
      return;
    }

    a2_unitree_ipc::StateStream state;
    if (a2_unitree_ipc::decode_state_stream(line, &state)) {
      publish_state(state);
      publish_sdk_status(state.connected, state.connected ? "a2_state_ok" : "agent_disconnected");
      return;
    }

    a2_unitree_ipc::HealthStatus health;
    if (a2_unitree_ipc::decode_health_status(line, &health)) {
      publish_sdk_status(health.connected && health.sdk_ready, health.reason);
      return;
    }

    publish_sdk_status(false, "invalid_agent_message");
  }

  void publish_state(const a2_unitree_ipc::StateStream & state)
  {
    const auto current_time = now();
    a2_interfaces::msg::RobotState msg;
    msg.stamp = to_builtin_time(current_time);
    msg.source_mode = state.source_mode;
    msg.frame_id = "base_link";
    msg.connected = state.connected;
    msg.imu_valid = state.imu_valid;
    msg.odom_valid = state.odom_valid;
    for (std::size_t index = 0; index < 3U; ++index) {
      msg.position[index] = state.position[index];
      msg.velocity[index] = state.velocity[index];
      msg.rpy[index] = state.rpy[index];
      msg.linear_acceleration[index] = state.linear_acceleration[index];
      msg.angular_velocity[index] = state.angular_velocity[index];
    }
    for (std::size_t index = 0; index < 4U; ++index) {
      msg.orientation_xyzw[index] = state.orientation_xyzw[index];
    }
    msg.body_height = state.body_height;
    msg.yaw_speed = state.yaw_speed;
    msg.motion_mode = state.motion_mode;
    msg.progress = state.progress;
    msg.gait_type = state.gait_type;
    state_pub_->publish(msg);
    last_state_time_ = current_time;

    {
      std::lock_guard<std::mutex> lock(battery_mutex_);
      battery_present_ = state.battery_present;
      battery_percentage_ratio_ = state.battery_percentage;
      battery_voltage_ = state.battery_voltage;
      battery_current_a_ = state.battery_current;
      battery_charging_ = state.battery_charging;
      if (state.battery_present) {
        last_battery_time_ = current_time;
      }
    }
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
    msg.linear_acceleration[2] = 9.81F;
    msg.angular_velocity[2] = static_cast<float>(cmd_yaw);
    msg.body_height = 0.28F;
    msg.yaw_speed = static_cast<float>(cmd_yaw);
    msg.motion_mode = 1U;
    msg.gait_type = 1U;
    state_pub_->publish(msg);
    last_state_time_ = current_time;
    publish_sdk_status(true, "mock_state_ok");
  }

  void publish_battery_state()
  {
    const auto current_time = now();
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = to_builtin_time(current_time);

    if (use_mock_) {
      msg.present = true;
      msg.percentage = static_cast<float>(std::max(0.0, std::min(1.0, mock_battery_percent_ / 100.0)));
      msg.voltage = 29.4F;
      msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    } else {
      std::lock_guard<std::mutex> lock(battery_mutex_);
      const bool has_battery = battery_present_ && last_battery_time_.nanoseconds() != 0;
      const double age = has_battery ? (current_time - last_battery_time_).seconds() : 1e9;
      if (!has_battery || age > battery_stale_timeout_sec_) {
        msg.present = false;
        msg.percentage = std::numeric_limits<float>::quiet_NaN();
        msg.voltage = std::numeric_limits<float>::quiet_NaN();
        msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
      } else {
        msg.present = true;
        msg.percentage = static_cast<float>(battery_percentage_ratio_);
        msg.voltage = static_cast<float>(battery_voltage_);
        msg.current = static_cast<float>(battery_current_a_);
        msg.power_supply_status = battery_charging_ ?
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING :
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
      }
    }

    msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    battery_pub_->publish(msg);
  }

  void watchdog_tick()
  {
    if (last_state_time_.nanoseconds() == 0) {
      publish_sdk_status(false, use_mock_ ? "mock_not_initialized" : "waiting_for_agent_state");
      return;
    }

    const auto age = (now() - last_state_time_).seconds();
    if (age > stale_timeout_sec_) {
      publish_sdk_status(false, use_mock_ ? "mock_state_stale" : "agent_state_stale");
      return;
    }
    publish_sdk_status(true, use_mock_ ? "mock_state_ok" : "a2_state_ok");
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

    const std::string mode = use_mock_ ? "mock" : "real";
    std::string state = connected ? "ready" : "waiting";
    if (status.find("ipc_") == 0 || status == "waiting_for_agent_state") {
      state = "waiting_agent";
    } else if (status.find("invalid_") == 0) {
      state = "error";
    } else if (status == "mock_state_stale" || status == "agent_state_stale") {
      state = "stale";
    }

    std_msgs::msg::String status_msg;
    status_msg.data =
      "mode=" + mode +
      ";state=" + state +
      ";ready=" + std::string(connected ? "true" : "false") +
      ";reason=" + status +
      ";ipc_socket=" + ipc_socket_path_ +
      ";sdk_owner=unitree_agent" +
      ";robot_profile=" + robot_profile_ +
      ";robot_model=" + robot_model_;
    sdk_status_pub_->publish(status_msg);
    status_pub_->publish(status_msg);
  }

  bool use_mock_{true};
  bool state_subscribed_{false};
  std::string robot_profile_;
  std::string robot_model_;
  std::string state_topic_;
  std::string battery_topic_;
  std::string status_topic_;
  std::string mock_cmd_topic_;
  std::string ipc_socket_path_{a2_unitree_ipc::kDefaultSocketPath};
  int ipc_timeout_ms_{100};
  double timer_hz_{50.0};
  double stale_timeout_sec_{0.5};
  double battery_publish_hz_{1.0};
  double battery_stale_timeout_sec_{5.0};
  double mock_battery_percent_{85.0};
  double mock_cmd_timeout_sec_{0.5};
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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mock_cmd_sub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr battery_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_battery_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_mock_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_mock_cmd_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist mock_cmd_;

  std::mutex ipc_mutex_;
  std::unique_ptr<a2_unitree_ipc::UnixSocketClient> ipc_client_;

  std::mutex battery_mutex_;
  bool battery_present_{false};
  double battery_percentage_ratio_{0.0};
  double battery_voltage_{0.0};
  double battery_current_a_{0.0};
  bool battery_charging_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<A2SdkBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
