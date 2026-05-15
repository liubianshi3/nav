import math
import os
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from a2_bringup.runtime_mode import normalize_runtime_mode


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _rotation_from_rpy(rpy):
    roll, pitch, yaw = [float(value) for value in rpy]
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _transpose(matrix):
    return [[matrix[col][row] for col in range(3)] for row in range(3)]


def _matmul(lhs, rhs):
    return [
        [
            sum(lhs[row][inner] * rhs[inner][col] for inner in range(3))
            for col in range(3)
        ]
        for row in range(3)
    ]


def _matvec(matrix, vector):
    return [
        sum(matrix[row][col] * vector[col] for col in range(3))
        for row in range(3)
    ]


def _relative_transform(base_xyz, base_rpy, target_xyz, target_rpy):
    base_rotation = _rotation_from_rpy(base_rpy)
    target_rotation = _rotation_from_rpy(target_rpy)
    base_rotation_inv = _transpose(base_rotation)
    relative_translation = [float(target_xyz[index]) - float(base_xyz[index]) for index in range(3)]
    relative_xyz = _matvec(base_rotation_inv, relative_translation)
    relative_rotation = _matmul(base_rotation_inv, target_rotation)
    relative_rpy = [
        math.atan2(relative_rotation[2][1], relative_rotation[2][2]),
        math.atan2(-relative_rotation[2][0], math.sqrt(relative_rotation[2][1] ** 2 + relative_rotation[2][2] ** 2)),
        math.atan2(relative_rotation[1][0], relative_rotation[0][0]),
    ]
    return relative_xyz, relative_rotation, relative_rpy


def _flatten_row_major(matrix):
    return [float(matrix[row][col]) for row in range(3) for col in range(3)]


def _render_fastlio_config(a2_system_share, runtime_dir):
    slam_cfg = _load_yaml(os.path.join(a2_system_share, "config", "slam.yaml"))
    extrinsics_cfg = _load_yaml(os.path.join(a2_system_share, "config", "extrinsics.yaml"))
    slam_params = slam_cfg.get("slam_manager", {}).get("ros__parameters", {})
    extrinsics = extrinsics_cfg.get("extrinsics", {})
    lidar = extrinsics.get("lidar", {})
    imu = extrinsics.get("imu", {})

    rel_xyz, rel_rotation, rel_rpy = _relative_transform(
        imu.get("xyz", [0.0, 0.0, 0.0]),
        imu.get("rpy", [0.0, 0.0, 0.0]),
        lidar.get("xyz", [0.0, 0.0, 0.0]),
        lidar.get("rpy", [0.0, 0.0, 0.0]),
    )

    runtime_dir.mkdir(parents=True, exist_ok=True)
    map_dir = Path.home() / "a2_system_ws" / "runtime" / "maps" / "fastlio"
    map_dir.mkdir(parents=True, exist_ok=True)
    config_path = runtime_dir / "fastlio_front_lidar.yaml"

    rendered = {
        "common": {
            "lid_topic": slam_params.get("pointcloud_topic", "/jt128/front/points"),
            "imu_topic": slam_params.get("imu_topic", "/jt128/front/imu"),
            "time_sync_en": False,
            "time_offset_lidar_to_imu": 0.0,
        },
        "preprocess": {
            "lidar_type": 1,
            "scan_line": 4,
            "timestamp_unit": 3,
            "blind": 0.2,
        },
        "mapping": {
            "acc_cov": 0.1,
            "gyr_cov": 0.1,
            "b_acc_cov": 0.0001,
            "b_gyr_cov": 0.0001,
            "fov_degree": 360.0,
            "det_range": 100.0,
            "extrinsic_est_en": False,
            "extrinsic_T": [float(value) for value in rel_xyz],
            "extrinsic_R": _flatten_row_major(rel_rotation),
        },
        "publish": {
            "path_en": True,
            "scan_publish_en": True,
            "dense_publish_en": False,
            "scan_bodyframe_pub_en": False,
        },
        "pcd_save": {
            "pcd_save_en": True,
            "interval": -1,
            "map_file_path": str(map_dir),
        },
        "a2_frames": {
            "map_frame": slam_params.get("map_frame", "map"),
            "odom_frame": slam_params.get("odom_frame", "odom"),
            "base_frame": slam_params.get("base_frame", "base_link"),
            "lidar_frame": lidar.get("child", "lidar_link"),
            "imu_frame": imu.get("child", "imu_link"),
            "lidar_rpy_in_imu": [float(value) for value in rel_rpy],
        },
    }
    config_path.write_text(yaml.safe_dump(rendered, sort_keys=False), encoding="utf-8")
    return str(config_path), slam_params


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    runtime_mode = normalize_runtime_mode(
        LaunchConfiguration("runtime_mode").perform(context),
    )
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    a2_system_share = get_package_share_directory("a2_system")
    runtime_dir = Path.home() / "a2_system_ws" / "runtime" / "generated"
    diagnostic_only = os.environ.get("A2_REAL_DIAGNOSTIC_ONLY", "0") == "1"
    slam_params = _load_yaml(os.path.join(a2_system_share, "config", "slam.yaml")).get(
        "slam_manager", {}
    ).get("ros__parameters", {})
    slam_manager_params = _load_yaml(
        os.path.join(a2_system_share, "config", "slam_manager.yaml")
    ).get("slam_orchestrator", {}).get("ros__parameters", {})

    stack_profile = slam_params.get("external_stack_profile", slam_manager_params.get("stack_profile", "fast_lio"))
    external_odom_topics = slam_params.get(
        "odom_topic_candidates",
        slam_manager_params.get("external_odom_topics", ["/Odometry"]),
    )

    actions = []
    stack_available = False
    stack_blocked_reason = ""

    if runtime_mode == "real" and diagnostic_only:
        stack_blocked_reason = "diagnostic_network_not_ready"
        actions.append(LogInfo(msg="Real stack is in diagnostic mode. External SLAM launch is deferred until wired data links are online."))
    elif runtime_mode == "real" and stack_profile == "external_odom":
        stack_available = True
        actions.append(
            LogInfo(
                msg=(
                    "Using robot-native odometry as the real SLAM profile. "
                    f"Waiting for odom on {external_odom_topics}."
                )
            )
        )
    elif runtime_mode == "real" and stack_profile == "fast_lio":
        try:
            get_package_share_directory(slam_params.get("fast_lio_package", "fast_lio"))
            generated_config, slam_params = _render_fastlio_config(a2_system_share, runtime_dir)
            stack_available = True
            actions.extend([
                LogInfo(msg=f"FAST_LIO config generated at {generated_config}"),
                Node(
                    package=slam_params.get("fast_lio_package", "fast_lio"),
                    executable=slam_params.get("fast_lio_executable", "fastlio_mapping"),
                    name="fastlio_mapping",
                    output="screen",
                    parameters=[generated_config, {"use_sim_time": use_sim_time}],
                ),
            ])
        except PackageNotFoundError:
            actions.append(LogInfo(msg="fast_lio package not found. SLAM will remain in waiting/offline mode."))
    elif runtime_mode == "real":
        actions.append(LogInfo(msg=f"SLAM stack profile `{stack_profile}` is not implemented by this bringup yet."))

    actions.append(
        Node(
            package="slam_manager",
            executable="slam_orchestrator",
            name="slam_orchestrator",
            parameters=[
                f"{a2_system_share}/config/slam_manager.yaml",
                {
                    "runtime_mode": runtime_mode,
                    "stack_profile": stack_profile,
                    "stack_available": stack_available,
                    "stack_blocked_reason": stack_blocked_reason,
                    "external_odom_topics": external_odom_topics,
                    "use_sim_time": use_sim_time,
                },
            ],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("runtime_mode", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
