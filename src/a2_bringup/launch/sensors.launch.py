import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from a2_bringup.runtime_mode import normalize_runtime_mode
from pathlib import Path


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _load_real_lidar_config(path):
    cfg = _load_yaml(path)
    return cfg.get("real_lidar", {}).get("ros__parameters", {})


def _load_real_camera_config(path):
    cfg = _load_yaml(path)
    return cfg.get("real_camera", {}).get("ros__parameters", {})


def _load_robot_config(path):
    if not path:
        return {}
    try:
        return _load_yaml(path)
    except OSError:
        return {}


def _resolve_optional_config_path(a2_system_share, value):
    value = (value or "").strip()
    if not value:
        return ""
    if os.path.isabs(value):
        return value
    candidate = os.path.join(a2_system_share, "config", value)
    if os.path.exists(candidate):
        return candidate
    return value


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    runtime_mode = normalize_runtime_mode(LaunchConfiguration("runtime_mode").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    robot_config_path = LaunchConfiguration("robot_config").perform(context).strip()
    a2_system_share = get_package_share_directory("a2_system")
    bringup_share = get_package_share_directory("a2_bringup")
    real_lidar_config_path = LaunchConfiguration("real_lidar_config").perform(context).strip()
    if not real_lidar_config_path:
        real_lidar_config_path = os.path.join(a2_system_share, "config", "real_lidar.yaml")
    real_camera_config_path = LaunchConfiguration("real_camera_config").perform(context).strip()
    if not real_camera_config_path:
        real_camera_config_path = os.path.join(a2_system_share, "config", "real_camera.yaml")
    diagnostic_only = os.environ.get("A2_REAL_DIAGNOSTIC_ONLY", "0") == "1"
    real_lidar_cfg = _load_real_lidar_config(real_lidar_config_path)
    ground_segmentation_impl = (
        LaunchConfiguration("ground_segmentation_impl").perform(context).strip().lower() or "cpp"
    )
    if ground_segmentation_impl not in ("cpp", "python"):
        ground_segmentation_impl = "cpp"
    real_lidar_profile = real_lidar_cfg.get("profile", "hesai_jt128_front")
    real_lidar_driver_mode = real_lidar_cfg.get("driver_mode", "")
    real_lidar_imu_topic = real_lidar_cfg.get("imu_topic", "/jt128/front/imu")
    real_lidar_input_topic = real_lidar_cfg.get("input_topic", "/jt128/front/points")
    real_lidar_output_topic = real_lidar_cfg.get("output_topic", "/jt128/front/points")
    real_lidar_output_frame = real_lidar_cfg.get("output_frame_id", "jt128_front_link")
    requested_extrinsics_file = _resolve_optional_config_path(
        a2_system_share, real_lidar_cfg.get("extrinsics_file", "")
    )
    direct_pointcloud_mode = (
        real_lidar_driver_mode == "external_pointcloud"
        or real_lidar_profile == "unitree_native_fused"
    )
    guard_pointcloud_topic = (
        real_lidar_input_topic if direct_pointcloud_mode else real_lidar_output_topic
    )
    guard_stale_timeout = float(real_lidar_cfg.get("stale_timeout_sec", 1.0))
    use_jt128_extrinsics = real_lidar_output_frame.startswith("jt128_") or "jt128" in real_lidar_profile
    requested_robot_extrinsics_file = _resolve_optional_config_path(
        a2_system_share,
        _load_robot_config(robot_config_path)
        .get("static_tf_manager", {})
        .get("ros__parameters", {})
        .get("extrinsics_file", ""),
    )
    extrinsics_file = requested_robot_extrinsics_file or requested_extrinsics_file or (
        f"{a2_system_share}/config/jt128_extrinsics.yaml"
        if use_jt128_extrinsics
        else f"{a2_system_share}/config/extrinsics.yaml"
    )
    robot_cfg = _load_robot_config(robot_config_path)
    base_height = float(
        robot_cfg.get("static_tf_manager", {}).get("ros__parameters", {}).get("base_height", 0.28)
    )
    real_camera_cfg = _load_real_camera_config(real_camera_config_path)
    camera_enabled = bool(real_camera_cfg.get("enabled", False))
    camera_profile = str(real_camera_cfg.get("profile", "") or "").strip() or "disabled"
    camera_driver_mode = str(real_camera_cfg.get("driver_mode", "") or "").strip()
    camera_input_points = str(real_camera_cfg.get("input_pointcloud_topic", "") or "").strip()
    camera_output_points = str(real_camera_cfg.get("output_pointcloud_topic", "") or "").strip()
    camera_output_frame = str(real_camera_cfg.get("output_frame_id", "") or "").strip()
    camera_restamp = bool(real_camera_cfg.get("restamp_on_receive", False))
    camera_timeout = float(real_camera_cfg.get("stale_timeout_sec", 1.0))
    camera_use_for_costmap = bool(real_camera_cfg.get("use_for_costmap", False))

    actions = [
        Node(
            package="tf_manager",
            executable="static_tf_manager",
            name="static_tf_manager",
            parameters=[{
                "extrinsics_file": extrinsics_file,
                "tf_file": f"{a2_system_share}/config/tf.yaml",
                "base_height": base_height,
                "use_sim_time": use_sim_time,
            }],
        ),
        Node(
            package="sensor_sync",
            executable="sync_monitor",
            name="sync_monitor",
            parameters=[f"{a2_system_share}/config/sensor_sync.yaml", {
                "imu_topic": real_lidar_imu_topic,
                "pointcloud_topic": guard_pointcloud_topic,
                "runtime_mode": runtime_mode,
                "use_sim_time": use_sim_time,
            }],
        ),
        Node(
            package="sensor_sync",
            executable="pointcloud_guard",
            name="pointcloud_guard",
            parameters=[{
                "runtime_mode": runtime_mode,
                "pointcloud_topic": guard_pointcloud_topic,
                "stale_timeout_sec": guard_stale_timeout,
                "connected_topic": "/a2/lidar/connected",
                "status_topic": "/a2/lidar/status",
                "status_label": "lidar",
                "sensor_profile": real_lidar_profile,
                "sensor_model": real_lidar_profile,
                "sensor_config": Path(real_lidar_config_path).name,
                "use_sim_time": use_sim_time,
            }],
        ),
    ]

    # Diagnostic aggregator — always on, collects all status sources
    # and publishes standard DiagnosticArray + /a2/health
    actions.append(
        Node(
            package="a2_diagnostics",
            executable="diagnostic_aggregator",
            name="diagnostic_aggregator",
            parameters=[f"{get_package_share_directory('a2_diagnostics')}/config/diagnostic_aggregator.yaml"],
            output="screen",
        )
    )

    # Nav health monitor — consumes /diagnostics_agg, drives degradation levels
    actions.append(
        Node(
            package="nav_health_monitor",
            executable="nav_health_monitor",
            name="nav_health_monitor",
            parameters=[f"{get_package_share_directory('nav_health_monitor')}/config/nav_health_monitor.yaml"],
            output="screen",
        )
    )

    if diagnostic_only:
        actions.append(
            LogInfo(
                msg="Real diagnostic mode enabled. JT128 driver launch is deferred until the wired data path is validated."
            )
        )
        return actions

    if direct_pointcloud_mode:
        restamp_on_receive = bool(real_lidar_cfg.get("restamp_on_receive", False))
        if (
            real_lidar_input_topic != real_lidar_output_topic
            or real_lidar_output_frame
            or restamp_on_receive
        ):
            actions.append(
                LogInfo(
                    msg=(
                        f"Using external pointcloud input_topic={real_lidar_input_topic} "
                        f"consumer_topic={guard_pointcloud_topic} compatibility_output={real_lidar_output_topic} "
                        f"frame_id={real_lidar_output_frame} restamp_on_receive={restamp_on_receive}"
                    )
                )
            )
            actions.append(
                Node(
                    package="sensor_sync",
                    executable="pointcloud_relay",
                    name="pointcloud_relay",
                    parameters=[{
                        "input_topic": real_lidar_input_topic,
                        "output_topic": real_lidar_output_topic,
                        "frame_id": real_lidar_output_frame,
                        "restamp_on_receive": restamp_on_receive,
                    }],
                )
            )
    if real_lidar_profile == "hesai_jt128_front" or real_lidar_driver_mode == "dedicated_hesai_ros_driver":

        # Ground segmentation: separate ground from obstacles,
        # feeds /a2/obstacle/points -> collision_monitor + occupancy_mapper
        # and   /a2/ground/points   -> future traversability mapping
        enable_ground_seg = bool(real_lidar_cfg.get("enable_ground_segmentation", True))
        if enable_ground_seg:
            actions.append(
                LogInfo(
                    msg=(
                        "Ground segmentation enabled. "
                        f"input={real_lidar_output_topic} -> "
                        "obstacle=/a2/obstacle/points ground=/a2/ground/points"
                    )
                )
            )
            if ground_segmentation_impl == "cpp":
                gs_share = get_package_share_directory("a2_ground_segmentation_cpp")
                actions.append(
                    Node(
                        package="a2_ground_segmentation_cpp",
                        executable="ground_segmentation_cpp_node",
                        name="ground_segmentation_cpp",
                        parameters=[
                            f"{gs_share}/config/ground_segmentation_cpp.yaml",
                            {
                                "input_topic": real_lidar_output_topic,
                                "use_sim_time": use_sim_time,
                            },
                        ],
                    )
                )
            else:
                actions.append(
                    Node(
                        package="a2_ground_segmentation",
                        executable="ground_segmentation_node",
                        name="ground_segmentation",
                        parameters=[{
                            "input_topic": real_lidar_output_topic,
                            "use_sim_time": use_sim_time,
                        }],
                    )
                )

            # Collision monitor: spatial safety paired with ground_seg.
            # Takes /cmd_vel (from Nav2 velocity_smoother) + /a2/obstacle/points,
            # publishes safe commands to /cmd_vel_safe -> a2_control_bridge.
            enable_collision = bool(real_lidar_cfg.get("enable_collision_monitor", True))
            if enable_collision:
                actions.append(
                    LogInfo(
                        msg=(
                            "Collision monitor enabled. "
                            "/cmd_vel -> [stop/slowdown filter] -> /cmd_vel_safe"
                        )
                    )
                )
                actions.append(
                    Node(
                        package="nav2_collision_monitor",
                        executable="collision_monitor",
                        name="collision_monitor",
                        parameters=[f"{a2_system_share}/config/collision_monitor.yaml", {
                            "use_sim_time": use_sim_time,
                        }],
                        output="screen",
                    )
                )
        actions.append(
            LogInfo(
                msg=(
                    "Starting JT128 driver from sensors.launch.py "
                    f"profile={real_lidar_profile} output_topic={real_lidar_output_topic}"
                )
            )
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(f"{bringup_share}/launch/jt128_driver.launch.py"),
                launch_arguments={
                    "config_path": f"{a2_system_share}/config/jt128_front_hesai.yaml",
                    "use_sim_time": str(use_sim_time).lower(),
                }.items(),
            )
        )
    else:
        actions.append(
            LogInfo(
                msg=(
                    "Unsupported real_lidar profile for the cleaned real-only stack: "
                    f"profile={real_lidar_profile} driver_mode={real_lidar_driver_mode}"
                )
            )
        )

    if camera_enabled:
        if not camera_input_points:
            actions.append(
                LogInfo(
                    msg=(
                        "Camera enabled but input_pointcloud_topic is empty. "
                        f"profile={camera_profile} driver_mode={camera_driver_mode}"
                    )
                )
            )
            return actions

        camera_consumer_points = camera_input_points
        if camera_output_points:
            camera_consumer_points = camera_output_points

        if camera_output_points and (
            camera_input_points != camera_output_points or camera_output_frame or camera_restamp
        ):
            actions.append(
                LogInfo(
                    msg=(
                        f"Using external depth pointcloud input={camera_input_points} "
                        f"output={camera_output_points} frame_id={camera_output_frame} "
                        f"restamp_on_receive={camera_restamp} use_for_costmap={camera_use_for_costmap}"
                    )
                )
            )
            actions.append(
                Node(
                    package="sensor_sync",
                    executable="pointcloud_relay",
                    name="camera_pointcloud_relay",
                    parameters=[{
                        "input_topic": camera_input_points,
                        "output_topic": camera_output_points,
                        "frame_id": camera_output_frame,
                        "restamp_on_receive": camera_restamp,
                    }],
                )
            )

        actions.append(
            Node(
                package="sensor_sync",
                executable="pointcloud_guard",
                name="camera_pointcloud_guard",
                parameters=[{
                    "runtime_mode": runtime_mode,
                    "pointcloud_topic": camera_consumer_points,
                    "stale_timeout_sec": camera_timeout,
                    "connected_topic": "/a2/camera/depth/connected",
                    "status_topic": "/a2/camera/depth/status",
                    "status_label": "camera_depth",
                    "sensor_profile": camera_profile,
                    "sensor_model": camera_profile,
                    "sensor_config": Path(real_camera_config_path).name,
                    "use_sim_time": use_sim_time,
                }],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("runtime_mode", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("network_interface", default_value=""),
        DeclareLaunchArgument("robot_config", default_value=""),
        DeclareLaunchArgument("real_lidar_config", default_value=""),
        DeclareLaunchArgument("real_camera_config", default_value=""),
        DeclareLaunchArgument(
            "ground_segmentation_impl",
            default_value="cpp",
            description="Ground segmentation implementation: 'cpp' (default) or 'python' fallback.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
