from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_start = LaunchConfiguration("auto_start")
    use_sim_time = LaunchConfiguration("use_sim_time")
    a2_system_share = get_package_share_directory("a2_system")
    return LaunchDescription([
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="exploration_manager",
            executable="exploration_manager_node",
            name="exploration_manager",
            parameters=[f"{a2_system_share}/config/exploration.yaml", {
                "auto_start": auto_start,
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
