from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from a2_bringup.runtime_mode import normalize_runtime_mode, as_bool


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    runtime_mode = normalize_runtime_mode(LaunchConfiguration("runtime_mode").perform(context))
    enable_nav2_bringup = as_bool(LaunchConfiguration("enable_nav2_bringup").perform(context))
    real_localization_mode = (
        LaunchConfiguration("real_localization_mode").perform(context).strip() or "amcl"
    )
    use_sim_time = as_bool(LaunchConfiguration("use_sim_time").perform(context))
    a2_system_share = get_package_share_directory("a2_system")
    actions = []
    use_manual_real_localization = (
        runtime_mode == "real"
        and enable_nav2_bringup
        and real_localization_mode == "manual_odom"
    )
    if use_manual_real_localization:
        actions.append(
            Node(
                package="localization_manager",
                executable="manual_localization_publisher",
                name="manual_localization_publisher",
                parameters=[{
                    "odom_topic": "/odom",
                    "initial_pose_topic": "/initialpose",
                    "pose_topic": "/amcl_pose",
                    "map_frame": "map",
                    "odom_frame": "odom",
                    "base_frame": "base_link",
                    "xy_variance_growth_per_meter": 0.03,
                    "yaw_variance_growth_per_rad": 0.02,
                    "max_xy_variance": 0.20,
                    "max_yaw_variance": 0.15,
                    "max_odom_age_sec": 1.0,
                    "use_sim_time": use_sim_time,
                }],
            )
        )
    if runtime_mode == "real" and not enable_nav2_bringup:
        gate_pose_topic = "/odom"
        gate_pose_msg_type = "nav_msgs/msg/Odometry"
        gate_max_pose_age_sec = 5.0
    else:
        gate_pose_topic = "/amcl_pose"
        gate_pose_msg_type = "geometry_msgs/msg/PoseWithCovarianceStamped"
        gate_max_pose_age_sec = 1.5
    actions.append(
        Node(
            package="localization_manager",
            executable="localization_gate",
            name="localization_gate",
            parameters=[f"{a2_system_share}/config/localization.yaml", {
                "runtime_mode": runtime_mode,
                "input_pose_topic": gate_pose_topic,
                "input_pose_msg_type": gate_pose_msg_type,
                "max_pose_age_sec": gate_max_pose_age_sec,
                "latch_valid_pose": enable_nav2_bringup,
                "use_sim_time": use_sim_time,
            }],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("runtime_mode", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_nav2_bringup", default_value="false"),
        DeclareLaunchArgument("real_localization_mode", default_value="amcl"),
        OpaqueFunction(function=_launch_setup),
    ])
