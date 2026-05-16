from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass
class ServerConfig:
    host: str = "0.0.0.0"
    port: int = 8080
    allow_non_lan_access: bool = False
    cors_origins: list[str] = field(default_factory=list)
    static_dir: str = "backend/static"
    websocket_path: str = "/ws"


@dataclass
class GrpcConfig:
    enabled: bool = False
    host: str = "0.0.0.0"
    port: int = 50051


@dataclass
class RosTopicConfig:
    map_topic: str = "/map"
    pointcloud_topic: str = "/jt128/front/points"
    pointcloud_fallback_topic: str = "/jt128/front/points"
    pointcloud_primary_stale_sec: float = 2.0
    pointcloud_preview_max_points: int = 20000
    manage_map_service: str = "/map_manager/manage_map"
    task_manager_service: str = "/a2/task_manager/command"
    localization_pose_topic: str = "/amcl_pose"
    localization_pose_msg_type: str = "geometry_msgs/msg/PoseWithCovarianceStamped"
    odom_topic: str = "/odom"
    tf_topic: str = "/tf"
    tf_static_topic: str = "/tf_static"
    real_report_topic: str = "/a2/real/report"
    lidar_status_topic: str = "/a2/lidar/status"
    camera_status_topic: str = "/a2/camera/depth/status"
    localization_ok_topic: str = "/a2/localization_ok"
    localization_status_topic: str = "/a2/localization/status"
    relocalization_status_topic: str = "/a2/relocalization/status"
    safety_status_topic: str = "/a2/safety/status"
    map_manager_status_topic: str = "/a2/map_manager/status"
    map_manager_active_map_topic: str = "/a2/map_manager/active_map"
    task_manager_status_topic: str = "/a2/task_manager/status"
    pose_goal_status_topic: str = "/a2/nav2/status"
    sdk_status_topic: str = "/a2/sdk/status"
    raw_state_topic: str = "/a2/raw_state"
    camera_image_topic: str = "/camera/image_raw"
    camera_compressed_topic: str = "/camera/image_raw/compressed"
    battery_topic: str = "/a2/battery"
    scan_mission_status_topic: str = "/a2/scan_mission/status"
    light_command_topic: str = "/a2/light/command"


@dataclass
class CameraConfig:
    enabled: bool = True
    prefer_compressed: bool = True
    max_broadcast_hz: float = 2.0
    jpeg_quality: int = 70


@dataclass
class NavigationConfig:
    backend: str = "nav2"
    action_name: str = "/navigate_to_pose"
    goal_topic: str = "/goal_pose_"
    goal_frame: str = "map"
    cancel_stop_topic: str = "/cmd_vel"
    cancel_stop_burst_count: int = 5
    cancel_stop_burst_interval_sec: float = 0.05
    cancel_retarget_current_pose: bool = True
    initial_pose_topic: str = "/initialpose"
    action_wait_timeout_sec: float = 3.0
    goal_response_timeout_sec: float = 5.0
    goal_timeout_sec: float = 180.0
    cancel_timeout_sec: float = 3.0
    initial_pose_wait_timeout_sec: float = 8.0
    initial_pose_publish_interval_sec: float = 0.4
    initial_pose_covariance_xy: float = 0.05
    initial_pose_covariance_yaw: float = 0.03
    allow_send_goal: bool = True
    require_map_for_goal: bool = True
    require_localization_ready: bool = True
    pose_goal_tolerance_m: float = 0.35
    pose_goal_yaw_tolerance_rad: float = 0.35
    occupancy_block_threshold: int = 65
    goal_snap_radius_m: float = 1.5
    goal_clearance_m: float = 0.18
    initial_pose_snap_radius_m: float = 1.0
    initial_pose_clearance_m: float = 0.18


@dataclass
class HealthConfig:
    pose_stale_sec: float = 2.0
    battery_stale_sec: float = 5.0
    health_broadcast_hz: float = 1.0


@dataclass
class NativeSlamConfig:
    enabled: bool = True
    request_topic: str = "/api/slam_operate/request"
    response_topic: str = "/api/slam_operate/response"
    response_timeout_sec: float = 5.0
    mapping_type: str = "indoor"
    save_root: str = "/home/unitree/dist/maps"


@dataclass
class StackConfig:
    workspace: str = "~/a2_system_ws"
    network_interface: str = "net1"
    map_root: str = "~/a2_system_ws/runtime/maps"
    start_script: str = "~/a2_system_ws/src/a2_system/tools/start_real_stack.sh"
    stop_script: str = "~/a2_system_ws/src/a2_system/tools/stop_stack.sh"
    command_timeout_sec: float = 15.0


@dataclass
class AppConfig:
    server: ServerConfig = field(default_factory=ServerConfig)
    grpc: GrpcConfig = field(default_factory=GrpcConfig)
    ros: RosTopicConfig = field(default_factory=RosTopicConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    health: HealthConfig = field(default_factory=HealthConfig)
    native_slam: NativeSlamConfig = field(default_factory=NativeSlamConfig)
    stack: StackConfig = field(default_factory=StackConfig)
    config_path: Path | None = None
    project_root: Path = field(default_factory=lambda: Path(__file__).resolve().parents[1])

    @property
    def static_dir(self) -> Path:
        return (self.project_root / self.server.static_dir).resolve()


def _update_dataclass(instance: Any, data: dict[str, Any]) -> None:
    for key, value in data.items():
        if not hasattr(instance, key):
            continue
        current = getattr(instance, key)
        if hasattr(current, "__dataclass_fields__") and isinstance(value, dict):
            _update_dataclass(current, value)
        else:
            setattr(instance, key, value)


def load_config(config_path: str | Path | None = None) -> AppConfig:
    config = AppConfig()
    if config_path is None:
        default_path = config.project_root / "backend" / "config.example.yaml"
        config_path = default_path if default_path.exists() else None
    if config_path is not None:
        path = Path(config_path).expanduser().resolve()
        if path.exists():
            loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            _update_dataclass(config, loaded)
            config.config_path = path
    return config
