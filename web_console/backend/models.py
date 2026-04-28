from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field


class TextStatus(BaseModel):
    raw: str | None = None
    mode: str | None = None
    state: str | None = None
    ready: bool | None = None
    reason: str | None = None
    fields: dict[str, str] = Field(default_factory=dict)


class Pose2D(BaseModel):
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class MapSnapshot(BaseModel):
    loaded: bool = False
    representation: str = "occupancy_grid_2d"
    frame_id: str | None = None
    width: int = 0
    height: int = 0
    resolution: float = 0.0
    origin: Pose2D = Field(default_factory=Pose2D)
    stamp: str | None = None
    data: list[int] = Field(default_factory=list)


class PointCloudSnapshot(BaseModel):
    loaded: bool = False
    representation: str = "pointcloud_map_3d"
    frame_id: str | None = None
    stamp: str | None = None
    source_topic: str | None = None
    points: list[list[float]] = Field(default_factory=list)
    points_total: int = 0
    points_sampled: int = 0
    sample_stride: int = 1


class RobotPose(BaseModel):
    available: bool = False
    source: str = "localization_pose"
    frame_id: str | None = None
    stamp: str | None = None
    x: float | None = None
    y: float | None = None
    yaw: float | None = None
    stale: bool = True


class RawStateSummary(BaseModel):
    source_mode: str | None = None
    frame_id: str | None = None
    connected: bool | None = None
    imu_valid: bool | None = None
    odom_valid: bool | None = None
    position: list[float] = Field(default_factory=list)
    velocity: list[float] = Field(default_factory=list)
    rpy: list[float] = Field(default_factory=list)
    linear_acceleration: list[float] = Field(default_factory=list)
    angular_velocity: list[float] = Field(default_factory=list)
    body_height: float | None = None
    yaw_speed: float | None = None
    motion_mode: int | None = None
    gait_type: int | None = None
    progress: float | None = None


class RobotStatus(BaseModel):
    system_ready: bool | None = None
    localization_ok: bool | None = None
    real_report: TextStatus = Field(default_factory=TextStatus)
    lidar_status: TextStatus = Field(default_factory=TextStatus)
    localization_status: TextStatus = Field(default_factory=TextStatus)
    map_manager_status: TextStatus = Field(default_factory=TextStatus)
    task_manager_status: TextStatus = Field(default_factory=TextStatus)
    sdk_status: TextStatus = Field(default_factory=TextStatus)
    active_map: str | None = None
    velocity_linear_x: float | None = None
    velocity_angular_z: float | None = None
    raw_state: RawStateSummary | None = None


class NavigationGoal(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"


class NavigationGoalRequest(BaseModel):
    goal: NavigationGoal


class InitialPoseRequest(BaseModel):
    pose: NavigationGoal


class StartNavigationRequest(BaseModel):
    map_id: str


class SaveMapRequest(BaseModel):
    map_id: str


class NavigationTaskState(BaseModel):
    state: str = "idle"
    message: str | None = None
    action_server_ready: bool = False
    goal: NavigationGoal | None = None
    feedback: dict[str, Any] = Field(default_factory=dict)
    updated_at: str | None = None


class CameraFrame(BaseModel):
    available: bool = False
    topic: str | None = None
    frame_id: str | None = None
    stamp: str | None = None
    width: int | None = None
    height: int | None = None
    encoding: str | None = None
    format: str | None = None
    data_url: str | None = None
    stale: bool = True


class SystemHealth(BaseModel):
    backend_ok: bool = True
    ros_connected: bool = False
    ros_thread_alive: bool = False
    websocket_clients: int = 0
    action_server_ready: bool = False
    map_received: bool = False
    pose_received: bool = False
    camera_received: bool = False
    last_map_update: str | None = None
    last_pose_update: str | None = None
    last_camera_update: str | None = None
    last_error: str | None = None


class NodeCheck(BaseModel):
    key: str
    label: str
    state: str = "missing"
    running: bool = False
    required: bool = True
    detail: str | None = None


class MapArtifactInfo(BaseModel):
    kind: str
    path: str
    topic: str | None = None
    frame_id: str | None = None
    resolution: float | None = None
    stamp_sec: int | None = None
    stamp_nanosec: int | None = None
    points_total: int | None = None
    points_saved: int | None = None
    sample_stride: int | None = None


class SavedMapInfo(BaseModel):
    map_id: str
    map_yaml: str
    created_at: str | None = None
    representation: str | None = None
    source_topic: str | None = None
    pointcloud_topic_3d: str | None = None
    has_pointcloud_3d: bool = False
    width: int | None = None
    height: int | None = None
    resolution: float | None = None
    artifacts: list[MapArtifactInfo] = Field(default_factory=list)


class StackStatus(BaseModel):
    mode: str = "stopped"
    pid: int | None = None
    log_file: str | None = None
    selected_map_id: str | None = None
    selected_map_yaml: str | None = None
    nodes: list[NodeCheck] = Field(default_factory=list)
    maps: list[SavedMapInfo] = Field(default_factory=list)
    message: str | None = None


class DashboardSnapshot(BaseModel):
    map: MapSnapshot = Field(default_factory=MapSnapshot)
    pointcloud: PointCloudSnapshot = Field(default_factory=PointCloudSnapshot)
    pose: RobotPose = Field(default_factory=RobotPose)
    status: RobotStatus = Field(default_factory=RobotStatus)
    navigation: NavigationTaskState = Field(default_factory=NavigationTaskState)
    camera: CameraFrame = Field(default_factory=CameraFrame)
    health: SystemHealth = Field(default_factory=SystemHealth)
