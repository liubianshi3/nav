import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    a2_system_share = get_package_share_directory("a2_system")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_ekf", default_value="true"),
            DeclareLaunchArgument(
                "ekf_config",
                default_value=os.path.join(a2_system_share, "config", "ekf_3d.yaml"),
            ),

            LogInfo(msg="Starting EKF sensor fusion (robot_localization ekf_node)."),

            # robot_localization EKF node
            # Fuses DLIO odometry (twist prediction) + NDT global pose (measurement).
            # Publishes /odometry/filtered (nav_msgs/Odometry, map→base_link).
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                condition=IfCondition(LaunchConfiguration("enable_ekf")),
                parameters=[
                    LaunchConfiguration("ekf_config"),
                    {"use_sim_time": use_sim_time},
                ],
                output="screen",
            ),

            # Bridge: nav_msgs/Odometry → PoseWithCovarianceStamped
            # Converts /odometry/filtered → ekf_pose_with_covariance
            # so the Autoware NDT scan matcher can consume it as initial pose.
            Node(
                package="a2_system",
                executable="odometry_to_pose_covariance.py",
                name="odometry_to_pose_covariance",
                condition=IfCondition(LaunchConfiguration("enable_ekf")),
                parameters=[
                    {
                        "input_topic": "/odometry/filtered",
                        "output_topic": "ekf_pose_with_covariance",
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
        ]
    )
