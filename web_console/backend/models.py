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
    camera_status: TextStatus = Field(default_factory=TextStatus)
    localization_status: TextStatus = Field(default_factory=TextStatus)
    map_manager_status: TextStatus = Field(default_factory=TextStatus)
    task_manager_status: TextStatus = Field(default_factory=TextStatus)
    sdk_status: TextStatus = Field(default_factory=TextStatus)
    active_map: str | None = None
    velocity_linear_x: float | None = None
    velocity_angular_z: float | None = None
    raw_state: RawStateSummary | None = None
    ndt_score: float | None = None
    ndt_healthy: bool | None = None
    planner_type: str | None = None
    bt_filename: str | None = None


class NavigationGoal(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"


class NavigationGoalRequest(BaseModel):
    goal: NavigationGoal
    map_id: str | None = None


class InitialPoseRequest(BaseModel):
    pose: NavigationGoal
    map_id: str | None = None
    map_path: str | None = None


class StartNavigationRequest(BaseModel):
    map_id: str


class SaveMapRequest(BaseModel):
    map_id: str


class TaskRouteSummary(BaseModel):
    route_id: str
    route_path: str | None = None
    mission_name: str | None = None
    waypoint_count: int = 0
    updated_at: str | None = None


class TaskRouteDetail(TaskRouteSummary):
    route_yaml: str = ""


class SaveTaskRouteRequest(BaseModel):
    route_id: str
    route_yaml: str
    map_id: str | None = None


class RunTaskRouteRequest(BaseModel):
    route_id: str
    map_id: str | None = None
    mission_name: str | None = None
    dry_run: bool = False
    stop_on_failure: bool = True
    save_map_on_finish: bool = False
    save_map_on_failure: bool = False


class TaskRouteStatus(BaseModel):
    raw: str | None = None
    ready: bool | None = None
    state: str | None = None
    reason: str | None = None
    current_mode: str | None = None
    active_map: str | None = None
    route_state: str | None = None
    route_id: str | None = None
    route_path: str | None = None
    report_path: str | None = None
    fields: dict[str, str] = Field(default_factory=dict)


class VirtualObstacleZone(BaseModel):
    obstacle_id: str
    label: str | None = None
    kind: str = "circle_keepout"
    x: float
    y: float
    radius: float
    created_at: str | None = None
    updated_at: str | None = None


class VirtualObstacleUpsertRequest(BaseModel):
    obstacle_id: str | None = None
    label: str | None = None
    x: float
    y: float
    radius: float = 0.6


class VirtualObstacleListing(BaseModel):
    map_id: str
    obstacles: list[VirtualObstacleZone] = Field(default_factory=list)


class NavigationTaskState(BaseModel):
    state: str = "idle"
    message: str | None = None
    backend: str = "pose_topic_3d"
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


class LightColorPayload(BaseModel):
    r: int = Field(default=0, ge=0, le=255)
    g: int = Field(default=0, ge=0, le=255)
    b: int = Field(default=0, ge=0, le=255)


class SetLightRequestPayload(BaseModel):
    device_id: str = "a2"
    on: bool = False
    intensity: int = Field(default=0, ge=0, le=255)
    color_mode: int = Field(default=0, ge=0, le=255)
    rgb: LightColorPayload = Field(default_factory=LightColorPayload)
    color_temperature_kelvin: int = Field(default=0, ge=0, le=65535)


class LightStatusPayload(BaseModel):
    device_id: str = "a2"
    on: bool = False
    intensity: int = Field(default=0, ge=0, le=255)
    color_mode: int = Field(default=0, ge=0, le=255)
    rgb: LightColorPayload = Field(default_factory=LightColorPayload)
    color_temperature_kelvin: int = Field(default=0, ge=0, le=65535)
    timestamp: int = 0


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


class MapMediaEntry(BaseModel):
    kind: str
    path: str
    name: str
    group: str | None = None
    size_bytes: int | None = None
    artifact_kind: str | None = None
    linked_pointcloud_path: str | None = None
    linked_image_path: str | None = None
    link_source: str | None = None


class MapMediaListing(BaseModel):
    map_id: str
    entries: list[MapMediaEntry] = Field(default_factory=list)


class SavedMapInfo(BaseModel):
    map_id: str
    map_yaml: str | None = None
    created_at: str | None = None
    representation: str | None = None
    source_topic: str | None = None
    pointcloud_topic_3d: str | None = None
    has_pointcloud_3d: bool = False
    width: int | None = None
    height: int | None = None
    resolution: float | None = None
    navigation_compatible: bool = True
    navigation_compatibility_reason: str | None = None
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


class RecoveryStatus(BaseModel):
    active: bool = False
    step: str | None = None
    sequence: list[str] = Field(default_factory=list)
    recovered: bool | None = None
    duration_sec: float | None = None
    attempts: int = 0
    raw: str | None = None


class BatterySnapshot(BaseModel):
    available: bool = False
    percentage: float | None = None
    voltage: float | None = None
    charging: bool | None = None
    stamp: str | None = None


class DashboardSnapshot(BaseModel):
    map: MapSnapshot = Field(default_factory=MapSnapshot)
    pointcloud: PointCloudSnapshot = Field(default_factory=PointCloudSnapshot)
    pose: RobotPose = Field(default_factory=RobotPose)
    status: RobotStatus = Field(default_factory=RobotStatus)
    navigation: NavigationTaskState = Field(default_factory=NavigationTaskState)
    camera: CameraFrame = Field(default_factory=CameraFrame)
    health: SystemHealth = Field(default_factory=SystemHealth)
    battery: BatterySnapshot = Field(default_factory=BatterySnapshot)
    recovery: RecoveryStatus = Field(default_factory=RecoveryStatus)
