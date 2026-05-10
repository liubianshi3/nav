from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("a2_diagnostics")

    return LaunchDescription([
        Node(
            package="a2_diagnostics",
            executable="diagnostic_aggregator",
            name="diagnostic_aggregator",
            parameters=[f"{share}/config/diagnostic_aggregator.yaml"],
            output="screen",
        ),
    ])
