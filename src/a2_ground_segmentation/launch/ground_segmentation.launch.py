from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_py = get_package_share_directory("a2_ground_segmentation")
    share_cpp = get_package_share_directory("a2_ground_segmentation_cpp")
    config_py = f"{share_py}/config/ground_segmentation.yaml"
    config_cpp = f"{share_cpp}/config/ground_segmentation_cpp.yaml"

    return LaunchDescription([
        DeclareLaunchArgument("ground_segmentation_impl", default_value="python"),  # 'python' | 'cpp'
        DeclareLaunchArgument("input_topic", default_value="/jt128/front/points"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        # Python implementation
        Node(
            condition=UnlessCondition(LaunchConfiguration("ground_segmentation_impl") == "cpp"),
            package="a2_ground_segmentation",
            executable="ground_segmentation_node",
            name="ground_segmentation",
            parameters=[config_py, {
                "input_topic": LaunchConfiguration("input_topic"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            output="screen",
        ),

        # C++ implementation
        Node(
            condition=IfCondition(LaunchConfiguration("ground_segmentation_impl") == "cpp"),
            package="a2_ground_segmentation_cpp",
            executable="ground_segmentation_cpp_node",
            name="ground_segmentation_cpp",
            parameters=[config_cpp, {
                "input_topic": LaunchConfiguration("input_topic"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            output="screen",
        ),
    ])
