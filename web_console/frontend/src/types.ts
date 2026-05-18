export type StatusFields = Record<string, string>;

export interface TextStatus {
  raw: string | null;
  mode: string | null;
  state: string | null;
  ready: boolean | null;
  reason: string | null;
  fields: StatusFields;
}

export interface Pose2D {
  x: number;
  y: number;
  yaw: number;
}

export interface MapSnapshot {
  loaded: boolean;
  representation: string;
  frame_id: string | null;
  width: number;
  height: number;
  resolution: number;
  origin: Pose2D;
  stamp: string | null;
  data: number[];
}

export interface PointCloudSnapshot {
  loaded: boolean;
  representation: string;
  frame_id: string | null;
  stamp: string | null;
  source_topic: string | null;
  points: number[][];
  points_total: number;
  points_sampled: number;
  sample_stride: number;
}

export interface RobotPose {
  available: boolean;
  source: string;
  frame_id: string | null;
  stamp: string | null;
  x: number | null;
  y: number | null;
  yaw: number | null;
  stale: boolean;
}

export interface RawStateSummary {
  source_mode: string | null;
  frame_id: string | null;
  connected: boolean | null;
  imu_valid: boolean | null;
  odom_valid: boolean | null;
  position: number[];
  velocity: number[];
  rpy: number[];
  linear_acceleration: number[];
  angular_velocity: number[];
  body_height: number | null;
  yaw_speed: number | null;
  motion_mode: number | null;
  gait_type: number | null;
  progress: number | null;
}

export interface RobotStatus {
  system_ready: boolean | null;
  localization_ok: boolean | null;
  real_report: TextStatus;
  lidar_status: TextStatus;
  camera_status: TextStatus;
  localization_status: TextStatus;
  relocalization_status: TextStatus;
  safety_status: TextStatus;
  map_manager_status: TextStatus;
  task_manager_status: TextStatus;
  sdk_status: TextStatus;
  active_map: string | null;
  velocity_linear_x: number | null;
  velocity_angular_z: number | null;
  raw_state: RawStateSummary | null;
  ndt_score: number | null;
  ndt_healthy: boolean | null;
  planner_type: string | null;
  bt_filename: string | null;
}

export interface NavigationGoal {
  x: number;
  y: number;
  yaw: number;
  frame_id: string;
}

export interface NavigationTaskState {
  state: string;
  message: string | null;
  backend: string;
  action_server_ready: boolean;
  goal: NavigationGoal | null;
  feedback: Record<string, unknown>;
  updated_at: string | null;
}

export interface CameraFrame {
  available: boolean;
  topic: string | null;
  frame_id: string | null;
  stamp: string | null;
  width: number | null;
  height: number | null;
  encoding: string | null;
  format: string | null;
  data_url: string | null;
  stale: boolean;
}

export interface InitialPoseResult {
  pose: NavigationGoal;
  snapped: boolean;
  attempts?: number;
  message: string;
}

export interface NodeCheck {
  key: string;
  label: string;
  state: string;
  running: boolean;
  required: boolean;
  detail: string | null;
}

export interface SavedMapInfo {
  map_id: string;
  map_yaml: string | null;
  created_at: string | null;
  representation: string | null;
  source_topic: string | null;
  pointcloud_topic_3d: string | null;
  has_pointcloud_3d: boolean;
  width: number | null;
  height: number | null;
  resolution: number | null;
  navigation_compatible: boolean;
  navigation_compatibility_reason: string | null;
  artifacts: MapArtifactInfo[];
}

export interface MapArtifactInfo {
  kind: string;
  path: string;
  topic: string | null;
  frame_id: string | null;
  resolution: number | null;
  stamp_sec: number | null;
  stamp_nanosec: number | null;
  points_total: number | null;
  points_saved: number | null;
  sample_stride: number | null;
}

export interface MapMediaEntry {
  kind: string;
  path: string;
  name: string;
  group: string | null;
  size_bytes: number | null;
  artifact_kind: string | null;
  linked_pointcloud_path: string | null;
  linked_image_path: string | null;
  link_source: string | null;
}

export interface MapMediaListing {
  map_id: string;
  entries: MapMediaEntry[];
}

export interface TaskRouteSummary {
  route_id: string;
  route_path: string | null;
  mission_name: string | null;
  waypoint_count: number;
  updated_at: string | null;
}

export interface TaskRouteDetail extends TaskRouteSummary {
  route_yaml: string;
}

export interface TaskRouteStatus {
  raw: string | null;
  ready: boolean | null;
  state: string | null;
  reason: string | null;
  current_mode: string | null;
  active_map: string | null;
  route_state: string | null;
  route_id: string | null;
  route_path: string | null;
  report_path: string | null;
  fields: StatusFields;
}

export interface TaskRouteListing {
  routes: TaskRouteSummary[];
  status: TaskRouteStatus;
}

export interface TaskRouteRunRequestPayload {
  route_id: string;
  map_id?: string | null;
  mission_name?: string | null;
  dry_run?: boolean;
  stop_on_failure?: boolean;
  save_map_on_finish?: boolean;
  save_map_on_failure?: boolean;
}

export interface VirtualObstacleZone {
  obstacle_id: string;
  label: string | null;
  kind: string;
  x: number;
  y: number;
  radius: number;
  created_at: string | null;
  updated_at: string | null;
}

export interface VirtualObstacleListing {
  map_id: string;
  obstacles: VirtualObstacleZone[];
}

export interface VirtualObstacleUpsertPayload {
  obstacle_id?: string | null;
  label?: string | null;
  x: number;
  y: number;
  radius: number;
}

export interface LightColorPayload {
  r: number;
  g: number;
  b: number;
}

export interface LightStatusPayload {
  device_id: string;
  on: boolean;
  intensity: number;
  color_mode: number;
  rgb: LightColorPayload;
  color_temperature_kelvin: number;
  timestamp: number;
}

export interface SetLightRequestPayload {
  device_id: string;
  on: boolean;
  intensity: number;
  color_mode: number;
  rgb: LightColorPayload;
  color_temperature_kelvin: number;
}

export interface SetLightDebugResponse {
  ok: boolean;
  success: boolean;
  message: string;
  status: LightStatusPayload;
}

export interface StackStatus {
  mode: "stopped" | "starting" | "mapping" | "navigation" | string;
  pid: number | null;
  log_file: string | null;
  selected_map_id: string | null;
  selected_map_yaml: string | null;
  localization_mode: string | null;
  motion_mode: string | null;
  enable_motion: boolean | null;
  live_motion: boolean | null;
  dry_run: boolean | null;
  enable_nav2_3d: boolean | null;
  collision_monitor_profile: string | null;
  collision_monitor_config: string | null;
  nodes: NodeCheck[];
  maps: SavedMapInfo[];
  message: string | null;
}

export interface RecoveryStatus {
  active: boolean;
  step: string | null;
  sequence: string[];
  recovered: boolean | null;
  duration_sec: number | null;
  attempts: number;
  raw: string | null;
}

export interface BatterySnapshot {
  available: boolean;
  percentage: number | null;
  voltage: number | null;
  charging: boolean | null;
  health: number | null;
  stamp: string | null;
  stale: boolean;
}

export interface SystemHealth {
  backend_ok: boolean;
  ros_connected: boolean;
  ros_thread_alive: boolean;
  websocket_clients: number;
  action_server_ready: boolean;
  map_received: boolean;
  pose_received: boolean;
  camera_received: boolean;
  last_map_update: string | null;
  last_pose_update: string | null;
  last_camera_update: string | null;
  last_error: string | null;
}

export interface InitialPoseRequestPayload {
  pose: NavigationGoal;
  map_id?: string | null;
}

export interface DashboardSnapshot {
  map: MapSnapshot;
  pointcloud: PointCloudSnapshot;
  pose: RobotPose;
  status: RobotStatus;
  navigation: NavigationTaskState;
  camera: CameraFrame;
  health: SystemHealth;
  battery: BatterySnapshot;
  recovery: RecoveryStatus;
}

export interface BackendEvent<T> {
  type: string;
  payload: T;
}
