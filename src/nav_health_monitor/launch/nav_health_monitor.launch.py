from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("nav_health_monitor")
    return LaunchDescription([
        Node(
            package="nav_health_monitor",
            executable="nav_health_monitor",
            name="nav_health_monitor",
            parameters=[f"{share}/config/nav_health_monitor.yaml"],
            output="screen",
        ),
    ])
