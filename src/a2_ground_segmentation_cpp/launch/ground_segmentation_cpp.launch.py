from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("a2_ground_segmentation_cpp")
    config = f"{share}/config/ground_segmentation_cpp.yaml"

    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/jt128/front/points"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        Node(
            package="a2_ground_segmentation_cpp",
            executable="ground_segmentation_cpp_node",
            name="ground_segmentation_cpp",
            parameters=[config, {
                "input_topic": LaunchConfiguration("input_topic"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            output="screen",
        ),
    ])
