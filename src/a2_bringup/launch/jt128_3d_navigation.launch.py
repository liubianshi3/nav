import os
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _ros_bridge_env():
    return {
        "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"),
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
    }


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


def _resolve_nav2_map_arguments(context, *args, **kwargs):
    enable_nav2_3d = LaunchConfiguration("enable_nav2_3d").perform(context).strip().lower()
    if enable_nav2_3d not in ("1", "true", "t", "yes", "y", "on"):
        return []

    explicit_map = LaunchConfiguration("nav2_3d_map").perform(context).strip()
    if explicit_map:
        return [LogInfo(msg=f"Nav2 3D map YAML: {explicit_map}")]

    map_root = Path(LaunchConfiguration("map_root").perform(context))
    map_id = LaunchConfiguration("map_id").perform(context).strip()
    pcd_path = LaunchConfiguration("pcd_path").perform(context).strip()

    candidates: list[Path] = []
    if map_id:
        candidates.append(map_root / map_id / "map.yaml")
    if pcd_path:
        candidates.append(Path(pcd_path).expanduser().resolve().parent / "map.yaml")

    for candidate in candidates:
        if candidate.exists():
            resolved = str(candidate)
            return [
                SetLaunchConfiguration("nav2_3d_map", resolved),
                LogInfo(msg=f"Resolved Nav2 3D map YAML from navigation assets: {resolved}"),
            ]

    detail = f"map_id={map_id or 'none'};pcd_path={pcd_path or 'none'}"
    return [
        LogInfo(
            msg=(
                "Nav2 3D is enabled but no map YAML could be resolved automatically. "
                f"Pass nav2_3d_map explicitly or add map.yaml beside the selected 3D map. ({detail})"
            )
        )
    ]


def generate_launch_description():
    a2_system_share = get_package_share_directory("a2_system")
    ros_bridge_env = _ros_bridge_env()
    unitree_agent_socket = os.environ.get("A2_UNITREE_AGENT_SOCKET", "/run/a2/unitree_agent.sock")
    is_ndt_localization = PythonExpression(["'", LaunchConfiguration("localization_mode"), "' == 'ndt'"])
    is_odom_only_localization = PythonExpression(["'", LaunchConfiguration("localization_mode"), "' == 'odom_only'"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_id", default_value=""),
            DeclareLaunchArgument("pcd_path", default_value=""),
            DeclareLaunchArgument("map_root", default_value=os.environ.get("A2_WORKSPACE", str(Path.home() / "ws/device-navigation")) + "/runtime/maps"),
            DeclareLaunchArgument("start_static_tf", default_value="true"),
            DeclareLaunchArgument("start_robot_state", default_value="true"),
            DeclareLaunchArgument("start_task_manager", default_value="false"),
            DeclareLaunchArgument("start_scan_mission", default_value="false"),
            DeclareLaunchArgument("start_ekf_local", default_value="true"),
            DeclareLaunchArgument(
                "localization_mode",
                default_value="ndt",
                description="Localization mode for navigation: ndt for map localization, odom_only for short-range control-chain validation.",
            ),

            DeclareLaunchArgument("start_safety", default_value="false"),
            DeclareLaunchArgument("enable_motion", default_value="true"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("sdk_interface", default_value="eth0"),
            DeclareLaunchArgument("control_interface", default_value="eth0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "ndt_odom_topic",
                default_value="/jt128/dlio/odom",
                description="Odometry topic used by ndt_adapter for NDT initial guesses.",
            ),
            DeclareLaunchArgument("enable_nav2_3d", default_value="true",
                                  description="Launch Nav2 3D planning stack instead of pose_goal_controller_3d"),
            DeclareLaunchArgument("enable_global_traversability_layer", default_value="false",
                                  description="Feed stable 2.5D traversability obstacles into global_costmap. Default false; set true to enable explicitly."),
            DeclareLaunchArgument("global_traversability_config", default_value=f"{a2_system_share}/config/global_traversability_integrator.yaml",
                                  description="Path to global_traversability_integrator YAML"),
            DeclareLaunchArgument("nav2_3d_map", default_value="",
                                  description="Path to 2D projected map YAML for Nav2 3D mode"),
            DeclareLaunchArgument("ndt_max_map_to_odom_translation_step", default_value="5.0",
                                  description="Maximum accepted NDT map->odom correction step in meters for real A2 startup and map alignment"),
            DeclareLaunchArgument("ndt_max_map_to_odom_rotation_step_deg", default_value="90.0",
                                  description="Maximum accepted NDT map->odom correction step in degrees for real A2 startup and map alignment"),
            DeclareLaunchArgument(
                "collision_monitor_config",
                default_value=f"{a2_system_share}/config/collision_monitor.yaml",
                description=(
                    "Collision monitor YAML. Use collision_monitor_live_validation.yaml "
                    "only for supervised open-space live-motion validation."
                ),
            ),
            OpaqueFunction(function=_resolve_nav2_map_arguments),
            LogInfo(
                msg=(
                    "Starting JT128 3D navigation: loading map assets, odometry, safety gates, "
                    "and Nav2/control-chain components."
                )
            ),
            LogInfo(
                msg=(
                    "localization_mode=odom_only is for short-range live-motion "
                    "validation only; it is not a formal inspection localization mode."
                ),
                condition=IfCondition(is_odom_only_localization),
            ),
            Node(
                package="a2_sdk_bridge",
                executable="a2_sdk_bridge_node",
                name="a2_sdk_bridge",
                condition=IfCondition(LaunchConfiguration("start_robot_state")),
                additional_env=ros_bridge_env,
                parameters=[
                    f"{a2_system_share}/config/a2_sdk.yaml",
                    {
                        "use_mock": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
                        "ipc_socket_path": unitree_agent_socket,
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
                        get_package_share_directory("a2_bringup"),
                        "launch",
                        "ekf_local.launch.py",
                    )
                ),
                condition=IfCondition(LaunchConfiguration("start_ekf_local")),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "enable_ekf_local": "true",
                    "output_topic": "/odometry/local",
                }.items(),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odom_only_map_to_odom_static_tf",
                condition=IfCondition(is_odom_only_localization),
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                output="screen",
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
                    "odom_topic": LaunchConfiguration("ndt_odom_topic"),
                    "max_map_to_odom_translation_step": LaunchConfiguration("ndt_max_map_to_odom_translation_step"),
                    "max_map_to_odom_rotation_step_deg": LaunchConfiguration("ndt_max_map_to_odom_rotation_step_deg"),
                }.items(),
                condition=IfCondition(is_ndt_localization),
            ),
            _pointcloud_guard_action(),
            # ── Ground segmentation → /a2/obstacle/points + /a2/traversability ──
            Node(
                package="a2_ground_segmentation_cpp",
                executable="ground_segmentation_cpp_node",
                name="ground_segmentation",
                parameters=[
                    f"{get_package_share_directory('a2_ground_segmentation_cpp')}/config/ground_segmentation_cpp.yaml",
                    {
                        # Nav2 local_costmap/collision_monitor need robot-local obstacle observations.
                        "target_frame": "base_link",
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
                output="screen",
            ),
            # ── Collision monitor: /cmd_vel → stop/slowdown filter → /cmd_vel_safe ──
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                parameters=[
                    LaunchConfiguration("collision_monitor_config"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                output="screen",
            ),
            # collision_monitor lifecycle: managed by lifecycle_manager_navigation when Nav2 3D
            # is enabled; manually activated via TimerAction when enable_nav2_3d:=false.
            TimerAction(
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "bash", "-lc",
                            "ros2 lifecycle set /collision_monitor configure || true && "
                            "ros2 lifecycle set /collision_monitor activate || true",
                        ],
                        output="screen",
                    ),
                ],
                condition=UnlessCondition(LaunchConfiguration("enable_nav2_3d")),
            ),
            Node(
                package="a2_system",
                executable="auto_scan_mission.py",
                name="auto_scan_mission",
                condition=IfCondition(LaunchConfiguration("start_scan_mission")),
                parameters=[
                    f"{a2_system_share}/config/scan_mission_3d.yaml",
                    {
                        "enable_action_server": True,
                        "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
                        "waypoints_file": f"{a2_system_share}/config/scan_waypoints.example.yaml",
                        "reports_root": os.path.join(
                            os.environ.get("A2_WORKSPACE", str(Path.home() / "ws/device-navigation")),
                            "runtime",
                            "reports",
                            "scan_mission",
                        ),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "odom_topic": "/odometry/local",
                    },
                ],
                output="screen",
            ),
            Node(
                package="a2_system",
                executable="task_manager.py",
                name="task_manager",
                condition=IfCondition(LaunchConfiguration("start_task_manager")),
                parameters=[
                    f"{a2_system_share}/config/task_manager.yaml",
                    {
                        "runtime_mode": "",
                        "navigation_backend": "nav2",
                        "navigate_action_name": "/navigate_to_pose",
                        "run_mission_action_name": "/run_mission",
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
                output="screen",
            ),
            Node(
                package="localization_manager",
                executable="localization_gate",
                name="localization_gate",
                condition=IfCondition(is_ndt_localization),
                parameters=[
                    f"{a2_system_share}/config/localization_3d.yaml",
                    {
                        "runtime_mode": "",
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
            Node(
                package="localization_manager",
                executable="ndt_health_monitor",
                name="ndt_health_monitor",
                condition=IfCondition(is_ndt_localization),
                parameters=[
                    {
                        "ndt_status_topic": "/a2/relocalization/status",
                        "health_pub_topic": "/a2/ndt/healthy",
                        "health_status_topic": "/a2/ndt/health_status",
                        "min_score": 3.0,
                        "consecutive_failures_threshold": 5,
                        "eval_frequency": 5.0,
                        "initial_guess_timeout_sec": 1.0,
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
                        "runtime_mode": "",
                        "lidar_topic": "/jt128/front/points",
                        "map_topic": "/map",
                        "map_representation": "occupancy_grid_2d",
                        "latch_map_ready": True,
                        "map_transient_local": True,
                        "localization_mode": LaunchConfiguration("localization_mode"),
                        "require_map": ParameterValue(is_ndt_localization, value_type=bool),
                        "require_localization": ParameterValue(is_ndt_localization, value_type=bool),
                        "ndt_health_topic": "/a2/ndt/healthy",
                        "require_ndt_health": ParameterValue(is_ndt_localization, value_type=bool),
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
                        "runtime_mode": "",
                        "lidar_connected_topic": "/a2/lidar/connected",
                        "lidar_label": "jt128",
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
            # Nav2 3D planning stack (replaces pose_goal_controller_3d + goal_bridge)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("a2_bringup"),
                        "launch",
                        "nav2_3d.launch.py",
                    )
                ),
                condition=IfCondition(LaunchConfiguration("enable_nav2_3d")),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "map": LaunchConfiguration("nav2_3d_map"),
                    "enable_global_ekf_debug": "false",
                    "enable_global_traversability_layer": LaunchConfiguration("enable_global_traversability_layer"),
                    "global_traversability_config": LaunchConfiguration("global_traversability_config"),
                }.items(),
            ),
            # Fallback when Nav2 3D is disabled: obstacle-aware DWA-Lite planner
            # (C++ port of pose_goal_controller_3d with active obstacle avoidance).
            Node(
                package="nav2_integration_cpp",
                executable="obstacle_aware_local_planner_3d",
                name="obstacle_aware_local_planner_3d",
                condition=UnlessCondition(LaunchConfiguration("enable_nav2_3d")),
                parameters=[
                    f"{get_package_share_directory('nav2_integration_cpp')}/config/obstacle_aware_local_planner_3d.yaml",
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                output="screen",
            ),
            Node(
                package="a2_control_bridge",
                executable="a2_control_bridge_node",
                name="a2_control_bridge",
                additional_env=ros_bridge_env,
                parameters=[
                    f"{a2_system_share}/config/motion_limits.yaml",
                    {
                        "use_mock": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
                        "allow_loopback": False,
                        "runtime_mode": PythonExpression([
                            "'mock' if '",
                            LaunchConfiguration("dry_run"),
                            "'.lower() in ('1', 'true', 't', 'yes', 'y', 'on') else 'real'",
                        ]),
                        "network_interface": LaunchConfiguration("control_interface"),
                        "allow_motion_without_localization": ParameterValue(
                            PythonExpression(["'", LaunchConfiguration("localization_mode"), "' == 'odom_only' or '", LaunchConfiguration("start_safety"), "' != 'true'"]),
                            value_type=bool,
                        ),
                        "allow_motion_without_map": ParameterValue(
                            PythonExpression(["'", LaunchConfiguration("start_safety"), "' != 'true'"]),
                            value_type=bool,
                        ),
                        "linear_x_sign": float(os.environ.get("A2_CONTROL_LINEAR_X_SIGN", "1.0")),
                        "linear_y_sign": float(os.environ.get("A2_CONTROL_LINEAR_Y_SIGN", "1.0")),
                        "yaw_sign": float(os.environ.get("A2_CONTROL_YAW_SIGN", "1.0")),
                        "ipc_socket_path": unitree_agent_socket,
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
            # When safety_supervisor is off, run simplified gate that activates
            # collision_monitor lifecycle and publishes allow_motion=true.
            Node(
                package="a2_system",
                executable="simplified_safety_gate.py",
                name="simplified_safety_gate",
                condition=UnlessCondition(LaunchConfiguration("start_safety")),
                output="screen",
            ),
        ]
    )
