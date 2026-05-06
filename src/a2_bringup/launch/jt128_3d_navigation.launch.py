import os
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _unitree_ddsc_env():
    candidates = [
        "/opt/unitree_robotics/lib/x86_64/libddsc.so.0",
        "/unitree/opt/lib/libddsc.so.0",
    ]
    for candidate in candidates:
        if not os.path.exists(candidate):
            continue
        current = os.environ.get("LD_PRELOAD", "").strip()
        preload = candidate if not current else f"{candidate}:{current}"
        return {"LD_PRELOAD": preload}
    return {}


def _pointcloud_guard_action():
    try:
        get_package_share_directory("sensor_sync")
    except PackageNotFoundError:
        return LogInfo(
            msg=(
                "sensor_sync package is not installed; skipping pointcloud_guard. "
                "JT128 lidar readiness will be covered by safety/readiness monitors."
            )
        )
    return Node(
        package="sensor_sync",
        executable="pointcloud_guard",
        name="pointcloud_guard",
        condition=IfCondition(LaunchConfiguration("start_safety")),
        parameters=[
            {
                "pointcloud_topic": "/jt128/front/points",
                "stale_timeout_sec": 1.5,
                "connected_topic": "/a2/lidar/connected",
                "status_topic": "/a2/lidar/status",
                "status_label": "jt128",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )


def generate_launch_description():
    a2_system_share = get_package_share_directory("a2_system")
    unitree_ddsc_env = _unitree_ddsc_env()
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_id", default_value=""),
            DeclareLaunchArgument("pcd_path", default_value=""),
            DeclareLaunchArgument(
                "map_root", default_value=str(Path.home() / "a2_system_ws" / "runtime" / "maps")
            ),
            DeclareLaunchArgument("start_static_tf", default_value="false"),
            DeclareLaunchArgument("start_robot_state", default_value="true"),
            DeclareLaunchArgument("start_safety", default_value="true"),
            DeclareLaunchArgument("enable_motion", default_value="false"),
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("sdk_interface", default_value="eth0"),
            DeclareLaunchArgument("control_interface", default_value="eth0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            LogInfo(
                msg=(
                    "Starting JT128 3D navigation: loading PCD, running the first-pass "
                    "3D ICP relocalizer, localization gate, and pose-topic goal bridge."
                )
            ),
            Node(
                package="a2_sdk_bridge",
                executable="a2_sdk_bridge_node",
                name="a2_sdk_bridge",
                condition=IfCondition(LaunchConfiguration("start_robot_state")),
                additional_env=unitree_ddsc_env,
                parameters=[
                    f"{a2_system_share}/config/a2_sdk.yaml",
                    {
                        "use_mock": False,
                        "allow_loopback": False,
                        "network_interface": LaunchConfiguration("sdk_interface"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
            Node(
                package="a2_state_publisher",
                executable="a2_state_publisher_node",
                name="a2_state_publisher",
                condition=IfCondition(LaunchConfiguration("start_robot_state")),
                parameters=[
                    f"{a2_system_share}/config/state_bridge.yaml",
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "publish_tf": False,
                    },
                ],
            ),
            Node(
                package="tf_manager",
                executable="static_tf_manager",
                name="jt128_navigation_static_tf_manager",
                condition=IfCondition(LaunchConfiguration("start_static_tf")),
                parameters=[
                    {
                        "extrinsics_file": f"{a2_system_share}/config/jt128_extrinsics.yaml",
                        "tf_file": f"{a2_system_share}/config/tf.yaml",
                        "base_height": 0.28,
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
            Node(
                package="map_manager",
                executable="pointcloud_map_loader",
                name="pointcloud_map_loader",
                parameters=[
                    {
                        "map_root": LaunchConfiguration("map_root"),
                        "map_id": LaunchConfiguration("map_id"),
                        "pcd_path": LaunchConfiguration("pcd_path"),
                        "output_topic": "/a2/map/pointcloud_3d",
                        "frame_id": "map",
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("a2_ndt_adapter"),
                        "launch",
                        "ndt_adapter.launch.py",
                    )
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
            _pointcloud_guard_action(),
            Node(
                package="localization_manager",
                executable="localization_gate",
                name="localization_gate",
                parameters=[
                    f"{a2_system_share}/config/localization.yaml",
                    {
                        "runtime_mode": "real",
                        "input_pose_topic": "/a2/relocalization/pose",
                        "input_pose_msg_type": "geometry_msgs/msg/PoseWithCovarianceStamped",
                        "max_pose_age_sec": 1.5,
                        "max_xy_variance": 0.2,
                        "max_yaw_variance": 0.2,
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
            Node(
                package="safety_manager",
                executable="safety_supervisor",
                name="safety_supervisor",
                condition=IfCondition(LaunchConfiguration("start_safety")),
                parameters=[
                    f"{a2_system_share}/config/safety.yaml",
                    {
                        "runtime_mode": "real",
                        "lidar_topic": "/jt128/front/points",
                        "map_representation": "pointcloud_map_3d",
                        "require_map": False,
                        "require_localization": True,
                        "lidar_timeout_sec": 1.5,
                        "state_timeout_sec": 1.0,
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
            Node(
                package="safety_manager",
                executable="real_readiness_monitor",
                name="real_readiness_monitor",
                condition=IfCondition(LaunchConfiguration("start_safety")),
                parameters=[
                    {
                        "runtime_mode": "real",
                        "lidar_connected_topic": "/a2/lidar/connected",
                        "lidar_label": "jt128",
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
            Node(
                package="nav2_integration",
                executable="goal_bridge",
                name="goal_bridge",
                parameters=[
                    f"{a2_system_share}/config/nav2.yaml",
                    {
                        "runtime_mode": "real",
                        "navigation_backend": "pose_topic_3d",
                        "pose_goal_topic": "/a2/nav3/goal_pose",
                        "legacy_pose_goal_topic": "/goal_pose_",
                        "map_frame": "map",
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
            Node(
                package="nav2_integration",
                executable="pose_goal_controller_3d",
                name="pose_goal_controller_3d",
                output="screen",
                parameters=[
                    f"{a2_system_share}/config/pose_goal_controller_3d.yaml",
                    {
                        "dry_run": LaunchConfiguration("dry_run"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
            Node(
                package="a2_control_bridge",
                executable="a2_control_bridge_node",
                name="a2_control_bridge",
                condition=IfCondition(LaunchConfiguration("enable_motion")),
                additional_env=unitree_ddsc_env,
                parameters=[
                    f"{a2_system_share}/config/motion_limits.yaml",
                    {
                        "use_mock": False,
                        "allow_loopback": False,
                        "runtime_mode": "real",
                        "network_interface": LaunchConfiguration("control_interface"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
        ]
    )
