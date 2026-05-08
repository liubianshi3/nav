#ifndef A2_CONTROL_BRIDGE_NODE_HPP
#define A2_CONTROL_BRIDGE_NODE_HPP

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "a2_system/network_utils.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#if A2_ENABLE_UNITREE_SDK
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/a2/sport/sport_client.hpp>
#endif

class A2ControlBridgeNode : public rclcpp::Node
{
public:
  A2ControlBridgeNode()
  : Node("a2_control_bridge")
  {
    use_mock_ = declare_parameter<bool>("use_mock", true);
    runtime_mode_ = declare_parameter<std::string>("runtime_mode", use_mock_ ? "mock" : "real");
    auto_detect_interface_ = declare_parameter<bool>("auto_detect_interface", true);
    allow_loopback_ = declare_parameter<bool>("allow_loopback", true);
    network_interface_ = declare_parameter<std::string>("network_interface", "");
    interface_candidates_ = declare_parameter<std::vector<std::string>>(
      "interface_candidates", std::vector<std::string>{});
    cmd_topic_ = declare_parameter<std::string>("cmd_topic", "/cmd_vel");
    estop_topic_ = declare_parameter<std::string>("estop_topic", "/a2/estop");
    localization_ok_topic_ = declare_parameter<std::string>("localization_ok_topic", "/a2/localization_ok");
    map_ready_topic_ = declare_parameter<std::string>("map_ready_topic", "/a2/map_ready");
    allow_motion_topic_ = declare_parameter<std::string>("allow_motion_topic", "/a2/allow_motion");
    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.4);
    max_linear_y_ = declare_parameter<double>("max_linear_y", 0.25);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.5);
    cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.5);
    control_hz_ = declare_parameter<double>("control_hz", 20.0);
    allow_motion_without_map_ = declare_parameter<bool>("allow_motion_without_map", false);
    allow_motion_without_localization_ = declare_parameter<bool>("allow_motion_without_localization", false);
    prepare_balance_stand_ = declare_parameter<bool>("prepare_balance_stand", runtime_mode_ == "real");
    prepare_balance_wait_sec_ = declare_parameter<double>(
      "prepare_balance_wait_sec", runtime_mode_ == "real" ? 2.0 : 0.0);
    sim_cmd_topic_ = declare_parameter<std::string>("sim_cmd_topic", "");

    debug_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/a2/command_limited", 10);
    control_status_pub_ = create_publisher<std_msgs::msg::String>("/a2/control/status", 10);
    if (!sim_cmd_topic_.empty()) {
      sim_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(sim_cmd_topic_, 10);
    }
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_, 10, std::bind(&A2ControlBridgeNode::on_cmd, this, std::placeholders::_1));
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      estop_topic_, 10, [this](const std_msgs::msg::Bool::SharedPtr msg) { estop_ = msg->data; });
    localization_sub_ = create_subscription<std_msgs::msg::Bool>(
      localization_ok_topic_, 10, [this](const std_msgs::msg::Bool::SharedPtr msg) { localization_ok_ = msg->data; });
    map_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      map_ready_topic_, 10, [this](const std_msgs::msg::Bool::SharedPtr msg) { map_ready_ = msg->data; });
    allow_motion_sub_ = create_subscription<std_msgs::msg::Bool>(
      allow_motion_topic_, 10, [this](const std_msgs::msg::Bool::SharedPtr msg) { allow_motion_ = msg->data; });

    // 订阅导航健康监控的速度缩放因子
    auto speed_scale_cb = [this](const std_msgs::msg::Float32::SharedPtr msg) {
      nav_speed_scale_ = std::max(0.0f, std::min(1.0f, msg->data));
    };
    nav_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/a2/nav/max_speed_scale", 10, speed_scale_cb);

    resolved_interface_ = resolve_interface();

#if A2_ENABLE_UNITREE_SDK
    if (runtime_mode_ == "real") {
      if (resolved_interface_.empty()) {
        real_interface_ready_ = false;
        RCLCPP_ERROR(get_logger(), "No usable network interface available for real A2 control.");
      } else if (!a2_system::interface_is_ready_for_real(resolved_interface_)) {
        real_interface_ready_ = false;
        RCLCPP_WARN(
          get_logger(),
          "Interface '%s' is not ready for real A2 control. Control bridge will stay in safe no-op mode.",
          resolved_interface_.c_str());
      } else {
        unitree::robot::ChannelFactory::Instance()->Init(0, resolved_interface_);
        sport_client_ = std::make_unique<unitree::robot::a2::SportClient>();
        sport_client_->SetTimeout(25.0F);
        sport_client_->Init();
        RCLCPP_INFO(
          get_logger(), "A2 control bridge initialized with A2 SportClient on interface '%s'.",
          resolved_interface_.c_str());
      }
    }
#endif

    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(control_hz_, 1.0))),
      std::bind(&A2ControlBridgeNode::control_tick, this));
  }

  // Public for testing
  static double clamp(double value, double limit)
  {
    return std::max(-limit, std::min(value, limit));
  }

  bool motion_gate_open() const
  {
    if (estop_) {
      return false;
    }
    if (!allow_motion_) {
      return false;
    }
    if (!allow_motion_without_localization_ && !localization_ok_) {
      return false;
    }
    if (!allow_motion_without_map_ && !map_ready_) {
      return false;
    }
    return true;
  }

  // Expose member access for testing via topic injection only.
  // All private members remain private; tests interact through ROS topics.

private:
  std::string resolve_interface() const
  {
    if (runtime_mode_ == "gazebo") {
      return "gazebo";
    }
    const bool simulated_mode = runtime_mode_ == "mock" || runtime_mode_ == "gazebo";
    const bool allow_loopback = simulated_mode && allow_loopback_;
    if (!network_interface_.empty() && a2_system::interface_exists(network_interface_)) {
      return network_interface_;
    }
    if (auto_detect_interface_) {
      return a2_system::select_interface(network_interface_, interface_candidates_, allow_loopback);
    }
    return network_interface_;
  }

  void publish_control_status(
    const std::string & state,
    bool ready,
    const std::string & reason)
  {
    std_msgs::msg::String status_msg;
    status_msg.data =
      "mode=" + runtime_mode_ +
      ";state=" + state +
      ";ready=" + std::string(ready ? "true" : "false") +
      ";reason=" + reason +
      ";interface=" + (resolved_interface_.empty() ? "none" : resolved_interface_) +
      ";sport_client=a2";
    control_status_pub_->publish(status_msg);
    if (status_msg.data != last_control_status_) {
      last_control_status_ = status_msg.data;
      RCLCPP_INFO(get_logger(), "control status: %s", status_msg.data.c_str());
    }
  }

  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_ = *msg;
    last_cmd_time_ = now();
    have_cmd_ = true;
  }

  void control_tick()
  {
    geometry_msgs::msg::Twist limited;
    const bool timed_out = !have_cmd_ || (now() - last_cmd_time_).seconds() > cmd_timeout_sec_;
    const bool gate_open = motion_gate_open();

    if (!timed_out && gate_open) {
      limited.linear.x = clamp(latest_cmd_.linear.x, max_linear_x_);
      limited.linear.y = clamp(latest_cmd_.linear.y, max_linear_y_);
      limited.angular.z = clamp(latest_cmd_.angular.z, max_yaw_rate_);
      // 应用导航健康监控的速度缩放
      limited.linear.x *= nav_speed_scale_;
      limited.linear.y *= nav_speed_scale_;
      limited.angular.z *= nav_speed_scale_;
    }

    geometry_msgs::msg::TwistStamped debug;
    debug.header.stamp = now();
    debug.header.frame_id = "base_link";
    debug.twist = limited;
    debug_pub_->publish(debug);
    if (sim_cmd_pub_) {
      sim_cmd_pub_->publish(limited);
    }

    std::string status_state = "idle";
    std::string status_reason = "cmd_timeout";
    bool status_ready = true;
    if (!gate_open) {
      status_state = "blocked";
      status_reason = estop_ ? "estop" :
        (!allow_motion_ ? "allow_motion_false" :
        (!localization_ok_ ? "localization_not_ready" : "map_not_ready"));
      status_ready = false;
    } else if (!timed_out) {
      const bool active = std::fabs(limited.linear.x) > 1e-3 || std::fabs(limited.linear.y) > 1e-3 ||
        std::fabs(limited.angular.z) > 1e-3;
      status_state = active ? "ready" : "idle";
      status_reason = active ? "command_active" : "command_zero";
    }

    if (!gate_open) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Motion rejected. estop=%s allow_motion=%s localization_ok=%s map_ready=%s",
        estop_ ? "true" : "false",
        allow_motion_ ? "true" : "false",
        localization_ok_ ? "true" : "false",
        map_ready_ ? "true" : "false");
    }

    if (runtime_mode_ != "real") {
      const bool active = std::fabs(limited.linear.x) > 1e-3 || std::fabs(limited.linear.y) > 1e-3 ||
        std::fabs(limited.angular.z) > 1e-3;
      publish_control_status(status_state, status_ready, status_reason);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Simulated control tick: mode=%s active=%s vx=%.3f vy=%.3f wz=%.3f interface='%s'",
        runtime_mode_.c_str(),
        active ? "true" : "false", limited.linear.x, limited.linear.y, limited.angular.z,
        resolved_interface_.c_str());
      return;
    }

#if A2_ENABLE_UNITREE_SDK
    if (!real_interface_ready_) {
      publish_control_status("waiting_interface", false, resolved_interface_.empty() ? "no_interface" : "interface_not_ready");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Real interface '%s' is not ready. Control bridge remains in diagnostic idle mode.",
        resolved_interface_.c_str());
      return;
    }

    if (!sport_client_) {
      publish_control_status("error", false, "sport_client_unavailable");
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "Sport client unavailable in real mode.");
      return;
    }

    const bool active = std::fabs(limited.linear.x) > 1e-3 || std::fabs(limited.linear.y) > 1e-3 ||
      std::fabs(limited.angular.z) > 1e-3;

    const auto current_time = now();

    if (prepare_balance_stand_ && !prepared_) {
      if (preparing_) {
        const double elapsed = (current_time - prepare_started_at_).seconds();
        if (elapsed >= prepare_balance_wait_sec_) {
          prepared_ = true;
          preparing_ = false;
          RCLCPP_INFO(
            get_logger(),
            "Balance stand preparation completed after %.2fs on interface '%s'.",
            elapsed, resolved_interface_.c_str());
        } else {
          publish_control_status("preparing", false, "balance_stand_wait");
          return;
        }
      } else if (active) {
        const auto balance_code = sport_client_->BalanceStand();
        if (balance_code != 0) {
          publish_control_status("error", false, "balance_stand_failed");
          RCLCPP_ERROR(
            get_logger(),
            "BalanceStand failed with code %d on interface '%s'.",
            balance_code, resolved_interface_.c_str());
          return;
        }
        preparing_ = true;
        prepare_started_at_ = current_time;
        publish_control_status("preparing", false, "balance_stand");
        RCLCPP_INFO(
          get_logger(),
          "BalanceStand triggered on interface '%s'; waiting %.2fs before Move().",
          resolved_interface_.c_str(), prepare_balance_wait_sec_);
        return;
      }
    }

    if (!active) {
      publish_control_status(status_state, true, status_reason);
      if (was_active_) {
        const auto stop_code = sport_client_->StopMove();
        if (stop_code != 0) {
          RCLCPP_WARN(
            get_logger(),
            "StopMove returned code %d on interface '%s'.",
            stop_code, resolved_interface_.c_str());
        }
        was_active_ = false;
      }
      return;
    }

    const auto move_code = sport_client_->Move(
      static_cast<float>(limited.linear.x),
      static_cast<float>(limited.linear.y),
      static_cast<float>(limited.angular.z));
    if (move_code != 0) {
      publish_control_status("error", false, "move_failed");
      RCLCPP_ERROR(
        get_logger(),
        "Move(vx=%.3f, vy=%.3f, wz=%.3f) failed with code %d on interface '%s'.",
        limited.linear.x, limited.linear.y, limited.angular.z, move_code,
        resolved_interface_.c_str());
      return;
    }
    was_active_ = true;
    publish_control_status("ready", true, "command_sent");
#else
    publish_control_status("error", false, "sdk_library_missing");
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Real control requested but this binary was built without unitree_sdk2.");
#endif
  }

  bool use_mock_{true};
  std::string runtime_mode_{"mock"};
  bool auto_detect_interface_{true};
  bool allow_loopback_{true};
  bool allow_motion_without_map_{false};
  bool allow_motion_without_localization_{false};
  bool prepare_balance_stand_{false};
  bool preparing_{false};
  bool real_interface_ready_{true};
  bool have_cmd_{false};
  bool allow_motion_{true};
  bool map_ready_{false};
  bool localization_ok_{false};
  bool estop_{false};
  bool prepared_{false};
  bool was_active_{false};

  std::string network_interface_;
  std::vector<std::string> interface_candidates_;
  std::string resolved_interface_;
  std::string cmd_topic_;
  std::string estop_topic_;
  std::string localization_ok_topic_;
  std::string map_ready_topic_;
  std::string allow_motion_topic_;
  std::string sim_cmd_topic_;

  double max_linear_x_{0.4};
  double max_linear_y_{0.25};
  double max_yaw_rate_{0.5};
  double cmd_timeout_sec_{0.5};
  double control_hz_{20.0};
  double prepare_balance_wait_sec_{0.0};
  float nav_speed_scale_{1.0f};

  geometry_msgs::msg::Twist latest_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time prepare_started_at_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr map_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr allow_motion_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr nav_speed_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr sim_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string last_control_status_;

#if A2_ENABLE_UNITREE_SDK
  std::unique_ptr<unitree::robot::a2::SportClient> sport_client_;
#endif
};

#endif  // A2_CONTROL_BRIDGE_NODE_HPP
