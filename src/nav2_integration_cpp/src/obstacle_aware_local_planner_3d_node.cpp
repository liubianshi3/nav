// Copyright 2026 a2_system_ws.
// Obstacle-aware local planner for the A2 3D pipeline.
//
// Replaces nav2_integration/pose_goal_controller_3d.py while preserving
// the same I/O contract (topics, parameter names, KV status payload).
//
// Extra inputs (additive, do not break existing consumers):
//   - sub:  obstacle_cloud_topic (default /a2/obstacle/points)
//   - sub:  recovery_cmd_topic   (default /a2/recovery/cmd_vel) — Twist
//          When the planner is in `blocked` state (no admissible trajectory)
//          the recovery command is forwarded to /cmd_vel after a basic
//          kinematic clamp + obstacle hard-clearance veto. This keeps the
//          control_bridge as the single safety gate.
//
// Extra status fields appended at the end (KV is order-tolerant):
//   admissible=<n>;rejected=<n>;clearance=<m>;source=<plan|recovery|p_servo>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "nav2_integration_cpp/dwa_planner.hpp"

namespace n2c = nav2_integration_cpp;

namespace
{
double yaw_from_quat(double x, double y, double z, double w) noexcept
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double clamp(double v, double lim) noexcept
{
  if (v > lim) {return lim;}
  if (v < -lim) {return -lim;}
  return v;
}
}  // namespace

class ObstacleAwareLocalPlanner3DNode : public rclcpp::Node
{
public:
  ObstacleAwareLocalPlanner3DNode()
  : rclcpp::Node("obstacle_aware_local_planner_3d")
  {
    declare_and_load_params();
    try {
      planner_ = std::make_unique<n2c::DwaPlanner>(planner_params_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "Invalid planner params: %s", e.what());
      throw;
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10,
      std::bind(&ObstacleAwareLocalPlanner3DNode::on_goal, this, std::placeholders::_1));
    if (!legacy_goal_topic_.empty() && legacy_goal_topic_ != goal_topic_) {
      legacy_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        legacy_goal_topic_, 10,
        std::bind(&ObstacleAwareLocalPlanner3DNode::on_goal, this, std::placeholders::_1));
    }
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, 20,
      std::bind(&ObstacleAwareLocalPlanner3DNode::on_pose, this, std::placeholders::_1));
    loc_ok_sub_ = create_subscription<std_msgs::msg::Bool>(
      localization_ok_topic_, 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {localization_ok_ = msg->data;});
    if (require_obstacle_cloud_) {
      obstacle_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        obstacle_cloud_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        std::bind(&ObstacleAwareLocalPlanner3DNode::on_obstacle_cloud, this,
        std::placeholders::_1));
    }
    recovery_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      recovery_cmd_topic_, 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        last_recovery_cmd_ = *msg;
        last_recovery_time_ = now();
      });

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, control_hz_)),
      std::bind(&ObstacleAwareLocalPlanner3DNode::tick, this));

    publish_status(false, "idle", "waiting_goal");
    RCLCPP_INFO(
      get_logger(),
      "obstacle_aware_local_planner_3d ready: goal=%s pose=%s cmd=%s obstacles=%s recovery=%s "
      "samples=%dx%dx%d sim=%.1fs",
      goal_topic_.c_str(), pose_topic_.c_str(), cmd_topic_.c_str(),
      obstacle_cloud_topic_.c_str(), recovery_cmd_topic_.c_str(),
      planner_params_.n_vx, planner_params_.n_vy, planner_params_.n_wz, planner_params_.sim_time);
  }

private:
  void declare_and_load_params()
  {
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/a2/nav3/goal_pose");
    legacy_goal_topic_ = declare_parameter<std::string>("legacy_goal_topic", "/goal_pose_");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/a2/relocalization/pose");
    cmd_topic_ = declare_parameter<std::string>("cmd_topic", "/cmd_vel");
    status_topic_ = declare_parameter<std::string>("status_topic", "/a2/nav3/status");
    localization_ok_topic_ = declare_parameter<std::string>(
      "localization_ok_topic", "/a2/localization_ok");
    obstacle_cloud_topic_ = declare_parameter<std::string>(
      "obstacle_cloud_topic", "/a2/obstacle/points");
    recovery_cmd_topic_ = declare_parameter<std::string>(
      "recovery_cmd_topic", "/a2/recovery/cmd_vel");

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    dry_run_ = declare_parameter<bool>("dry_run", false);
    require_localization_ok_ = declare_parameter<bool>("require_localization_ok", true);
    require_obstacle_cloud_ = declare_parameter<bool>("require_obstacle_cloud", true);
    obstacle_cloud_timeout_sec_ = declare_parameter<double>("obstacle_cloud_timeout_sec", 1.0);
    control_hz_ = std::max(1.0, declare_parameter<double>("control_hz", 10.0));
    pose_timeout_sec_ = declare_parameter<double>("pose_timeout_sec", 0.5);
    goal_timeout_sec_ = declare_parameter<double>("goal_timeout_sec", 60.0);
    max_goal_distance_from_current_ = declare_parameter<double>(
      "max_goal_distance_from_current", 1.5);
    obstacle_max_consider_range_ = declare_parameter<double>(
      "obstacle_max_consider_range", 4.0);
    obstacle_min_z_ = declare_parameter<double>("obstacle_min_z", -0.10);
    obstacle_max_z_ = declare_parameter<double>("obstacle_max_z", 1.50);
    recovery_cmd_timeout_sec_ = declare_parameter<double>("recovery_cmd_timeout_sec", 0.4);

    // Planner params.
    planner_params_.max_linear_x = declare_parameter<double>("max_linear_x", 0.18);
    planner_params_.max_linear_y = declare_parameter<double>("max_linear_y", 0.12);
    planner_params_.max_yaw_rate = declare_parameter<double>("max_yaw_rate", 0.30);
    planner_params_.min_linear_x = declare_parameter<double>("min_linear_x", 0.0);
    planner_params_.n_vx = static_cast<int>(declare_parameter<int>("n_vx", 7));
    planner_params_.n_vy = static_cast<int>(declare_parameter<int>("n_vy", 5));
    planner_params_.n_wz = static_cast<int>(declare_parameter<int>("n_wz", 9));
    planner_params_.sim_time = declare_parameter<double>("sim_time", 1.2);
    planner_params_.sim_step = declare_parameter<double>("sim_step", 0.1);
    planner_params_.inflation_radius = declare_parameter<double>("inflation_radius", 0.40);
    planner_params_.hard_clearance = declare_parameter<double>("hard_clearance", 0.20);
    planner_params_.obstacle_cost_weight = declare_parameter<double>("obstacle_cost_weight", 2.0);
    planner_params_.goal_dist_weight = declare_parameter<double>("goal_dist_weight", 1.0);
    planner_params_.goal_heading_weight = declare_parameter<double>("goal_heading_weight", 0.6);
    planner_params_.forward_progress_weight = declare_parameter<double>(
      "forward_progress_weight", 0.4);
    planner_params_.velocity_smoothness_weight = declare_parameter<double>(
      "velocity_smoothness_weight", 0.3);
    planner_params_.goal_tolerance_xy = declare_parameter<double>("goal_tolerance_xy", 0.15);
    planner_params_.goal_tolerance_yaw = declare_parameter<double>("goal_tolerance_yaw", 0.18);
    planner_params_.block_grace_sec = declare_parameter<double>("block_grace_sec", 3.0);
  }

  void on_goal(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!have_pose_) {
      reject_goal_locked("no_current_pose");
      return;
    }
    if (require_localization_ok_ && !localization_ok_) {
      reject_goal_locked("localization_not_ready");
      return;
    }
    if (require_obstacle_cloud_ && !obstacle_cloud_is_fresh_locked()) {
      reject_goal_locked("obstacle_cloud_stale");
      return;
    }

    std::string frame = msg->header.frame_id;
    if (frame.empty()) {
      frame = map_frame_;
    }
    if (frame != map_frame_) {
      reject_goal_locked(std::string{"bad_frame:"} + frame);
      return;
    }

    const double gx = msg->pose.position.x;
    const double gy = msg->pose.position.y;
    if (!std::isfinite(gx) || !std::isfinite(gy)) {
      reject_goal_locked("nonfinite_goal");
      return;
    }
    const double distance = std::hypot(gx - current_pose_.x, gy - current_pose_.y);
    if (distance > max_goal_distance_from_current_) {
      reject_goal_locked(
        "goal_too_far:distance=" + std::to_string(distance) +
        ",limit=" + std::to_string(max_goal_distance_from_current_));
      return;
    }

    goal_.x = gx;
    goal_.y = gy;
    goal_.yaw = yaw_from_quat(
      msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w);
    have_goal_ = true;
    goal_start_time_ = now();
    blocked_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    publish_status_locked(
      true, "goal_active",
      "accepted;distance=" + std::to_string(distance) +
      ";dry_run=" + (dry_run_ ? "true" : "false"));
  }

  void on_pose(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    current_pose_.yaw = yaw_from_quat(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    pose_time_ = now();
    have_pose_ = true;
  }

  void on_obstacle_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    // Convert to (x,y) in the planner frame. We assume the cloud is published
    // in `map_frame_` (the C++ ground_segmentation frame_id_override is "map"
    // by default). If frame mismatches, emit a throttled warning and skip.
    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "obstacle_cloud frame '%s' != planner frame '%s'; ignoring frame",
        msg->header.frame_id.c_str(), map_frame_.c_str());
    }

    std::vector<std::pair<float, float>> pts;
    pts.reserve(msg->width * msg->height / 2 + 4);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
      const std::size_t n = static_cast<std::size_t>(msg->width) * msg->height;
      for (std::size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz) {
        const float x = *ix;
        const float y = *iy;
        const float z = *iz;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        if (z < obstacle_min_z_ || z > obstacle_max_z_) {
          continue;
        }
        std::lock_guard<std::mutex> lk(mtx_);
        const double dx = static_cast<double>(x) - current_pose_.x;
        const double dy = static_cast<double>(y) - current_pose_.y;
        if ((dx * dx + dy * dy) >
          obstacle_max_consider_range_ * obstacle_max_consider_range_)
        {
          continue;
        }
        pts.emplace_back(x, y);
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "obstacle_cloud iter error: %s", e.what());
      return;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    obstacles_.resize(static_cast<Eigen::Index>(pts.size()), 2);
    for (std::size_t i = 0; i < pts.size(); ++i) {
      obstacles_(static_cast<Eigen::Index>(i), 0) = pts[i].first;
      obstacles_(static_cast<Eigen::Index>(i), 1) = pts[i].second;
    }
    obstacle_cloud_time_ = now();
    obstacle_cloud_points_ = static_cast<std::int64_t>(pts.size());
  }

  bool obstacle_cloud_is_fresh_locked() const
  {
    if (!require_obstacle_cloud_) {
      return true;
    }
    if (obstacle_cloud_points_ <= 0) {
      return false;
    }
    if (obstacle_cloud_time_.nanoseconds() == 0) {
      return false;
    }
    const double age = (now() - obstacle_cloud_time_).seconds();
    return age <= obstacle_cloud_timeout_sec_;
  }

  bool pose_is_fresh_locked() const
  {
    if (!have_pose_) {return false;}
    return (now() - pose_time_).seconds() <= pose_timeout_sec_;
  }

  void reject_goal_locked(const std::string & reason)
  {
    have_goal_ = false;
    publish_zero_locked();
    publish_status_locked(false, "goal_rejected", reason);
  }

  void publish_zero_locked()
  {
    if (!dry_run_) {
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
    }
  }

  bool maybe_use_recovery_locked(geometry_msgs::msg::Twist & out)
  {
    if (last_recovery_time_.nanoseconds() == 0) {
      return false;
    }
    if ((now() - last_recovery_time_).seconds() > recovery_cmd_timeout_sec_) {
      return false;
    }
    // Clamp + hard-clearance veto.
    geometry_msgs::msg::Twist v = last_recovery_cmd_;
    v.linear.x = clamp(v.linear.x, planner_params_.max_linear_x);
    v.linear.y = clamp(v.linear.y, planner_params_.max_linear_y);
    v.angular.z = clamp(v.angular.z, planner_params_.max_yaw_rate);

    // Hard clearance veto: simulate this command for sim_time and reject
    // if any rollout sample comes closer than hard_clearance to obstacles.
    n2c::Velocity vel{v.linear.x, v.linear.y, v.angular.z};
    const auto traj = n2c::DwaPlanner::simulate(
      current_pose_, vel, planner_params_.sim_time, planner_params_.sim_step);
    const double r2 = planner_params_.hard_clearance * planner_params_.hard_clearance;
    const auto M = static_cast<std::size_t>(obstacles_.rows());
    for (const auto & ps : traj) {
      for (std::size_t m = 0; m < M; ++m) {
        const double dx = static_cast<double>(obstacles_(static_cast<Eigen::Index>(m), 0)) - ps.x;
        const double dy = static_cast<double>(obstacles_(static_cast<Eigen::Index>(m), 1)) - ps.y;
        if ((dx * dx + dy * dy) < r2) {
          return false;  // unsafe
        }
      }
    }
    out = v;
    return true;
  }

  void tick()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!have_goal_) {
      return;
    }
    if (!pose_is_fresh_locked()) {
      publish_zero_locked();
      publish_status_locked(false, "blocked", "pose_stale");
      return;
    }
    if (require_localization_ok_ && !localization_ok_) {
      publish_zero_locked();
      publish_status_locked(false, "blocked", "localization_not_ready");
      return;
    }
    if (require_obstacle_cloud_ && !obstacle_cloud_is_fresh_locked()) {
      publish_zero_locked();
      publish_status_locked(false, "blocked", "obstacle_cloud_stale");
      return;
    }

    const double age = (now() - goal_start_time_).seconds();
    if (age > goal_timeout_sec_) {
      have_goal_ = false;
      publish_zero_locked();
      publish_status_locked(
        false, "goal_timeout", std::string{"age="} + std::to_string(age));
      return;
    }

    // Goal reached check.
    const double dx = goal_.x - current_pose_.x;
    const double dy = goal_.y - current_pose_.y;
    const double distance = std::hypot(dx, dy);
    const double yaw_err = std::atan2(
      std::sin(goal_.yaw - current_pose_.yaw),
      std::cos(goal_.yaw - current_pose_.yaw));
    if (distance <= planner_params_.goal_tolerance_xy &&
      std::fabs(yaw_err) <= planner_params_.goal_tolerance_yaw)
    {
      have_goal_ = false;
      publish_zero_locked();
      publish_status_locked(
        true, "goal_reached",
        std::string{"distance="} + std::to_string(distance) +
        ";yaw_error=" + std::to_string(yaw_err));
      return;
    }

    auto result = planner_->plan(current_pose_, goal_, obstacles_, last_cmd_);

    if (!result.success) {
      // Try recovery override before declaring blocked.
      geometry_msgs::msg::Twist rec_cmd;
      const bool use_rec = maybe_use_recovery_locked(rec_cmd);
      if (use_rec) {
        if (!dry_run_) {
          cmd_pub_->publish(rec_cmd);
        }
        last_cmd_ = n2c::Velocity{rec_cmd.linear.x, rec_cmd.linear.y, rec_cmd.angular.z};
        publish_status_locked(
          true, "avoiding",
          "recovery_override;clearance=" +
          std::to_string(result.min_clearance) +
          ";admissible=" + std::to_string(result.admissible_count) +
          ";rejected=" + std::to_string(result.rejected_count) +
          ";source=recovery");
        return;
      }
      publish_zero_locked();
      // Track block grace.
      if (blocked_since_.nanoseconds() == 0) {
        blocked_since_ = now();
      }
      const double block_age = (now() - blocked_since_).seconds();
      const std::string reason = std::string{"no_admissible_trajectory;block_age="} +
        std::to_string(block_age) +
        ";rejected=" + std::to_string(result.rejected_count);
      publish_status_locked(false, "blocked", reason);
      return;
    }

    blocked_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = result.cmd.vx;
    cmd.linear.y = result.cmd.vy;
    cmd.angular.z = result.cmd.wz;
    if (!dry_run_) {
      cmd_pub_->publish(cmd);
    }
    last_cmd_ = result.cmd;

    const std::string state = (result.min_clearance < planner_params_.inflation_radius) ?
      "avoiding" : "tracking";
    publish_status_locked(
      true, state,
      "distance=" + std::to_string(distance) +
      ";yaw_error=" + std::to_string(yaw_err) +
      ";vx=" + std::to_string(cmd.linear.x) +
      ";vy=" + std::to_string(cmd.linear.y) +
      ";wz=" + std::to_string(cmd.angular.z) +
      ";clearance=" + std::to_string(result.min_clearance) +
      ";admissible=" + std::to_string(result.admissible_count) +
      ";rejected=" + std::to_string(result.rejected_count) +
      ";source=plan;dry_run=" + (dry_run_ ? "true" : "false"));
  }

  void publish_status(bool ready, const std::string & state, const std::string & reason)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    publish_status_locked(ready, state, reason);
  }

  void publish_status_locked(bool ready, const std::string & state, const std::string & reason)
  {
    std_msgs::msg::String msg;
    msg.data =
      std::string{"state="} + state +
      ";ready=" + (ready ? "true" : "false") +
      ";reason=" + reason +
      ";goal_topic=" + goal_topic_ +
      ";pose_topic=" + pose_topic_ +
      ";cmd_topic=" + cmd_topic_ +
      ";require_obstacle_cloud=" + (require_obstacle_cloud_ ? "true" : "false") +
      ";obstacle_cloud_topic=" + obstacle_cloud_topic_ +
      ";obstacle_cloud_points=" + std::to_string(obstacle_cloud_points_) +
      ";planner=dwa_lite";
    status_pub_->publish(msg);
  }

  // Params
  std::string goal_topic_;
  std::string legacy_goal_topic_;
  std::string pose_topic_;
  std::string cmd_topic_;
  std::string status_topic_;
  std::string localization_ok_topic_;
  std::string obstacle_cloud_topic_;
  std::string recovery_cmd_topic_;
  std::string map_frame_;
  bool dry_run_{false};
  bool require_localization_ok_{true};
  bool require_obstacle_cloud_{true};
  double obstacle_cloud_timeout_sec_{1.0};
  double control_hz_{10.0};
  double pose_timeout_sec_{0.5};
  double goal_timeout_sec_{60.0};
  double max_goal_distance_from_current_{1.5};
  double obstacle_max_consider_range_{4.0};
  double obstacle_min_z_{-0.10};
  double obstacle_max_z_{1.50};
  double recovery_cmd_timeout_sec_{0.4};
  n2c::DwaParams planner_params_;

  // State (guarded by mtx_).
  mutable std::mutex mtx_;
  std::unique_ptr<n2c::DwaPlanner> planner_;
  bool have_pose_{false};
  bool localization_ok_{false};
  bool have_goal_{false};
  n2c::Pose2D current_pose_{};
  n2c::Pose2D goal_{};
  rclcpp::Time pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time goal_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time obstacle_cloud_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_recovery_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time blocked_since_{0, 0, RCL_ROS_TIME};
  std::int64_t obstacle_cloud_points_{0};
  Eigen::MatrixX2f obstacles_{0, 2};
  n2c::Velocity last_cmd_{};
  geometry_msgs::msg::Twist last_recovery_cmd_{};

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr legacy_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr loc_ok_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr recovery_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ObstacleAwareLocalPlanner3DNode>());
  } catch (const std::exception & e) {
    fprintf(stderr, "obstacle_aware_local_planner_3d fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
