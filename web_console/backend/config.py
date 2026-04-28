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
class RosTopicConfig:
    map_topic: str = "/map"
    pointcloud_topic: str = "/unitree/slam_lidar/points1"
    manage_map_service: str = "/map_manager/manage_map"
    localization_pose_topic: str = "/uslam/localization/odom"
    localization_pose_msg_type: str = "nav_msgs/msg/Odometry"
    odom_topic: str = "/odom"
    tf_topic: str = "/tf"
    tf_static_topic: str = "/tf_static"
    real_report_topic: str = "/a2/real/report"
    lidar_status_topic: str = "/a2/lidar/status"
    localization_ok_topic: str = "/a2/localization_ok"
    localization_status_topic: str = "/a2/localization/status"
    map_manager_status_topic: str = "/a2/map_manager/status"
    map_manager_active_map_topic: str = "/a2/map_manager/active_map"
    task_manager_status_topic: str = "/a2/task_manager/status"
    sdk_status_topic: str = "/a2/sdk/status"
    raw_state_topic: str = "/a2/raw_state"
    camera_image_topic: str = "/camera/image_raw"
    camera_compressed_topic: str = "/camera/image_raw/compressed"


@dataclass
class CameraConfig:
    enabled: bool = True
    prefer_compressed: bool = True
    max_broadcast_hz: float = 2.0
    jpeg_quality: int = 70


@dataclass
class NavigationConfig:
    action_name: str = "/navigate_to_pose"
    initial_pose_topic: str = "/initialpose"
    action_wait_timeout_sec: float = 3.0
    goal_response_timeout_sec: float = 5.0
    cancel_timeout_sec: float = 3.0
    initial_pose_wait_timeout_sec: float = 8.0
    initial_pose_publish_interval_sec: float = 0.4
    allow_send_goal: bool = True
    occupancy_block_threshold: int = 65
    goal_snap_radius_m: float = 1.5
    goal_clearance_m: float = 0.18
    initial_pose_snap_radius_m: float = 1.0
    initial_pose_clearance_m: float = 0.18


@dataclass
class HealthConfig:
    pose_stale_sec: float = 2.0
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
    network_interface: str = "eth0"
    map_root: str = "~/a2_system_ws/runtime/maps"
    start_script: str = "~/a2_system_ws/install/a2_system/share/a2_system/start_real_stack.sh"
    stop_script: str = "~/a2_system_ws/install/a2_system/share/a2_system/stop_stack.sh"
    command_timeout_sec: float = 15.0


@dataclass
class AppConfig:
    server: ServerConfig = field(default_factory=ServerConfig)
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
