from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    a2_system_share = get_package_share_directory("a2_system")
    result_mode = LaunchConfiguration("result_mode")
    dry_run = LaunchConfiguration("dry_run")
    waypoints_file = LaunchConfiguration("waypoints_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "result_mode",
                default_value="succeeded",
                description="Mock NavigateToPose result: succeeded, aborted, reject, timeout.",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="false",
                description="Run scan mission without sending goals.",
            ),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=f"{a2_system_share}/config/scan_waypoints.example.yaml",
            ),
            Node(
                package="a2_system",
                executable="mock_scan_mission_harness.py",
                name="mock_scan_mission_harness",
                output="screen",
                parameters=[{"result_mode": result_mode}],
            ),
            Node(
                package="a2_system",
                executable="auto_scan_mission.py",
                name="auto_scan_mission",
                output="screen",
                parameters=[
                    f"{a2_system_share}/config/scan_mission.yaml",
                    {
                        "waypoints_file": waypoints_file,
                        "dry_run": ParameterValue(dry_run, value_type=bool),
                        "require_real_ready": True,
                        "require_localization_ready": True,
                        "preflight_timeout_sec": 5.0,
                    },
                ],
            ),
        ]
    )
