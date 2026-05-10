"""
Launch collision_monitor in standalone mode for testing.

Usage:
  ros2 launch a2_bringup collision_monitor.launch.py

  # With custom safety zones:
  ros2 launch a2_bringup collision_monitor.launch.py \
    config_file:=/path/to/collision_monitor.yaml

Note: collision_monitor is a lifecycle node. It will start but remain
unconfigured until a lifecycle_manager activates it. In production, it's
managed by Nav2's lifecycle_manager_navigation. For standalone testing,
use:

  ros2 lifecycle set /collision_monitor configure
  ros2 lifecycle set /collision_monitor activate

Topic chain:
  Nav2 /cmd_vel ──→ collision_monitor ──→ /cmd_vel_safe
                       ↑
                /a2/obstacle/points
                (ground_seg non-ground)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    a2_system_share = get_package_share_directory("a2_system")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=f"{a2_system_share}/config/collision_monitor.yaml",
            description="Path to collision monitor YAML config",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        ),

        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            parameters=[LaunchConfiguration("config_file"), {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            output="screen",
        ),
    ])
