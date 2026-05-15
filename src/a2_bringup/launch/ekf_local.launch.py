import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    a2_system_share = get_package_share_directory("a2_system")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_ekf_local", default_value="true"),
            DeclareLaunchArgument(
                "ekf_local_config",
                default_value=os.path.join(a2_system_share, "config", "ekf_local.yaml"),
            ),
            DeclareLaunchArgument("output_topic", default_value="/odometry/local"),

            LogInfo(
                msg=(
                    "Starting local EKF topic-only validation: "
                    "DLIO odom + A2 body odom + body IMU -> /odometry/local."
                )
            ),

            Node(
                package="a2_system",
                executable="sensor_covariance_injector.py",
                name="jt128_dlio_covariance_injector",
                condition=IfCondition(LaunchConfiguration("enable_ekf_local")),
                parameters=[
                    {
                        "message_type": "odometry",
                        "input_topic": "/jt128/dlio/odom",
                        "output_topic": "/jt128/dlio/odom_cov",
                        "pose_covariance_diagonal": [0.05, 0.05, 5.0, 2.0, 2.0, 0.1],
                        "twist_covariance_diagonal": [0.04, 0.04, 5.0, 2.0, 2.0, 0.2],
                        "replace_existing": True,
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),

            Node(
                package="a2_system",
                executable="sensor_covariance_injector.py",
                name="body_imu_covariance_injector",
                condition=IfCondition(LaunchConfiguration("enable_ekf_local")),
                parameters=[
                    {
                        "message_type": "imu",
                        "input_topic": "/imu/data",
                        "output_topic": "/imu/data_cov",
                        "orientation_covariance_diagonal": [10.0, 10.0, 10.0],
                        "angular_velocity_covariance_diagonal": [0.1, 0.1, 0.03],
                        "linear_acceleration_covariance_diagonal": [5.0, 5.0, 5.0],
                        "replace_existing": True,
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),

            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_local_filter_node",
                condition=IfCondition(LaunchConfiguration("enable_ekf_local")),
                parameters=[
                    LaunchConfiguration("ekf_local_config"),
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("odometry/filtered", LaunchConfiguration("output_topic")),
                ],
                output="screen",
            ),
        ]
    )
