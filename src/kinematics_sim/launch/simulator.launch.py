"""
Launch the kinematics simulator in standalone mode.

Usage:
  ros2 launch kinematics_sim simulator.launch.py

  # With a specific PCD map:
  ros2 launch kinematics_sim simulator.launch.py \
    pcd_map_path:=/path/to/pointcloud_map_3d.pcd

  # With kidnap testing support:
  # Publish to /initialpose to reset robot position
  ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "..." -1
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("kinematics_sim")

    return LaunchDescription([
        DeclareLaunchArgument(
            "pcd_map_path",
            default_value="${A2_WORKSPACE}/runtime/maps/current/pointcloud_map_3d.pcd",
            description="Path to ASCII PCD map file",
        ),

        Node(
            package="kinematics_sim",
            executable="simulator_node",
            name="simulator_node",
            parameters=[f"{share}/config/simulator.yaml", {
                "pcd_map_path": LaunchConfiguration("pcd_map_path"),
            }],
            output="screen",
        ),
    ])
