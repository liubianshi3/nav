from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("nav2_integration_cpp")
    cfg = f"{share}/config/obstacle_aware_local_planner_3d.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="nav2_integration_cpp",
            executable="obstacle_aware_local_planner_3d",
            name="obstacle_aware_local_planner_3d",
            parameters=[cfg, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            output="screen",
        ),
    ])
