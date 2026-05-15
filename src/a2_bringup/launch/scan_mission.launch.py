from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    a2_system_share = get_package_share_directory("a2_system")
    config = LaunchConfiguration("config")
    waypoints_file = LaunchConfiguration("waypoints_file")
    dry_run = LaunchConfiguration("dry_run")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=f"{a2_system_share}/config/scan_mission.yaml",
            ),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=f"{a2_system_share}/config/scan_waypoints.example.yaml",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="false",
                description="Validate mission readiness and waypoint map cells without sending navigation goals.",
            ),
            Node(
                package="a2_system",
                executable="auto_scan_mission.py",
                name="auto_scan_mission",
                output="screen",
                parameters=[
                    config,
                    {
                        "waypoints_file": waypoints_file,
                        "dry_run": ParameterValue(dry_run, value_type=bool),
                    },
                ],
            ),
        ]
    )
