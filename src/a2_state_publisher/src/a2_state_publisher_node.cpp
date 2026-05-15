#include <array>
#include <memory>
#include <cmath>
#include <string>
#include <vector>

#include "a2_interfaces/msg/robot_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

class A2StatePublisherNode : public rclcpp::Node
{
public:
  A2StatePublisherNode()
  : Node("a2_state_publisher")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/a2/raw_state");
    state_topic_ = declare_parameter<std::string>("state_topic", "/robot_state");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_joint_states_ = declare_parameter<bool>("publish_joint_states", false);
    flatten_z_in_odom_ = declare_parameter<bool>("flatten_z_in_odom", true);
    planarize_orientation_in_odom_ = declare_parameter<bool>("planarize_orientation_in_odom", true);
    pose_covariance_diagonal_ = declare_parameter<std::vector<double>>(
      "pose_covariance_diagonal", {0.03, 0.03, 0.05, 0.02, 0.02, 0.04});
    twist_covariance_diagonal_ = declare_parameter<std::vector<double>>(
      "twist_covariance_diagonal", {0.02, 0.02, 0.05, 0.02, 0.02, 0.04});
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
       "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
       "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
       "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"});

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 20);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
    state_pub_ = create_publisher<a2_interfaces::msg::RobotState>(state_topic_, 20);
    if (publish_joint_states_) {
      joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 10);
    }
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    state_sub_ = create_subscription<a2_interfaces::msg::RobotState>(
      input_topic_, 20,
      std::bind(&A2StatePublisherNode::on_state, this, std::placeholders::_1));
  }

private:
  void on_state(const a2_interfaces::msg::RobotState::SharedPtr msg)
  {
    state_pub_->publish(*msg);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = msg->stamp;
    imu.header.frame_id = imu_frame_;
    imu.orientation.x = msg->orientation_xyzw[0];
    imu.orientation.y = msg->orientation_xyzw[1];
    imu.orientation.z = msg->orientation_xyzw[2];
    imu.orientation.w = msg->orientation_xyzw[3];
    imu.angular_velocity.x = msg->angular_velocity[0];
    imu.angular_velocity.y = msg->angular_velocity[1];
    imu.angular_velocity.z = msg->angular_velocity[2];
    imu.linear_acceleration.x = msg->linear_acceleration[0];
    imu.linear_acceleration.y = msg->linear_acceleration[1];
    imu.linear_acceleration.z = msg->linear_acceleration[2];
    imu_pub_->publish(imu);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg->stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = msg->position[0];
    odom.pose.pose.position.y = msg->position[1];
    odom.pose.pose.position.z = flatten_z_in_odom_ ? 0.0 : msg->position[2];
    const auto odom_quat = odom_orientation(*msg);
    odom.pose.pose.orientation.x = odom_quat[0];
    odom.pose.pose.orientation.y = odom_quat[1];
    odom.pose.pose.orientation.z = odom_quat[2];
    odom.pose.pose.orientation.w = odom_quat[3];
    odom.twist.twist.linear.x = msg->velocity[0];
    odom.twist.twist.linear.y = msg->velocity[1];
    odom.twist.twist.linear.z = msg->velocity[2];
    odom.twist.twist.angular.z = msg->yaw_speed;
    fill_covariance(odom.pose.covariance, pose_covariance_diagonal_);
    fill_covariance(odom.twist.covariance, twist_covariance_diagonal_);
    odom_pub_->publish(odom);

    if (publish_joint_states_) {
      sensor_msgs::msg::JointState joints;
      joints.header.stamp = msg->stamp;
      joints.name = joint_names_;
      joints.position.assign(joint_names_.size(), 0.0);
      joints.velocity.assign(joint_names_.size(), 0.0);
      joints.effort.assign(joint_names_.size(), 0.0);
      joint_pub_->publish(joints);
    }

    if (publish_tf_ && tf_broadcaster_ != nullptr) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = msg->stamp;
      transform.header.frame_id = odom_frame_;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = msg->position[0];
      transform.transform.translation.y = msg->position[1];
      transform.transform.translation.z = flatten_z_in_odom_ ? 0.0 : msg->position[2];
      transform.transform.rotation.x = odom_quat[0];
      transform.transform.rotation.y = odom_quat[1];
      transform.transform.rotation.z = odom_quat[2];
      transform.transform.rotation.w = odom_quat[3];
      tf_broadcaster_->sendTransform(transform);
    }
  }

  static double yaw_from_quaternion(const a2_interfaces::msg::RobotState & msg)
  {
    const auto x = static_cast<double>(msg.orientation_xyzw[0]);
    const auto y = static_cast<double>(msg.orientation_xyzw[1]);
    const auto z = static_cast<double>(msg.orientation_xyzw[2]);
    const auto w = static_cast<double>(msg.orientation_xyzw[3]);
    const auto siny_cosp = 2.0 * (w * z + x * y);
    const auto cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static std::array<double, 4> yaw_to_quaternion(double yaw)
  {
    const auto half_yaw = yaw * 0.5;
    return {0.0, 0.0, std::sin(half_yaw), std::cos(half_yaw)};
  }

  std::array<double, 4> odom_orientation(const a2_interfaces::msg::RobotState & msg) const
  {
    if (!planarize_orientation_in_odom_) {
      return {
        static_cast<double>(msg.orientation_xyzw[0]),
        static_cast<double>(msg.orientation_xyzw[1]),
        static_cast<double>(msg.orientation_xyzw[2]),
        static_cast<double>(msg.orientation_xyzw[3])};
    }
    double yaw = static_cast<double>(msg.rpy[2]);
    if (!std::isfinite(yaw)) {
      yaw = yaw_from_quaternion(msg);
    }
    return yaw_to_quaternion(yaw);
  }

  static void fill_covariance(
    std::array<double, 36> & covariance,
    const std::vector<double> & diagonal)
  {
    covariance.fill(0.0);
    for (std::size_t index = 0; index < 6U && index < diagonal.size(); ++index) {
      covariance[index * 6U + index] = diagonal[index];
    }
  }

  std::string input_topic_;
  std::string state_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string joint_state_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  bool publish_tf_{true};
  bool publish_joint_states_{false};
  bool flatten_z_in_odom_{true};
  bool planarize_orientation_in_odom_{true};
  std::vector<double> pose_covariance_diagonal_;
  std::vector<double> twist_covariance_diagonal_;
  std::vector<std::string> joint_names_;

  rclcpp::Subscription<a2_interfaces::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Publisher<a2_interfaces::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<A2StatePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
