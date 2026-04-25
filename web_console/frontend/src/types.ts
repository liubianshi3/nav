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
  frame_id: string | null;
  width: number;
  height: number;
  resolution: number;
  origin: Pose2D;
  stamp: string | null;
  data: number[];
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
  localization_status: TextStatus;
  map_manager_status: TextStatus;
  sdk_status: TextStatus;
  active_map: string | null;
  velocity_linear_x: number | null;
  velocity_angular_z: number | null;
  raw_state: RawStateSummary | null;
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
  action_server_ready: boolean;
  goal: NavigationGoal | null;
  feedback: Record<string, unknown>;
  updated_at: string | null;
}

export interface InitialPoseResult {
  pose: NavigationGoal;
  snapped: boolean;
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
  map_yaml: string;
  created_at: string | null;
  width: number | null;
  height: number | null;
  resolution: number | null;
}

export interface StackStatus {
  mode: "stopped" | "starting" | "mapping" | "navigation" | string;
  pid: number | null;
  log_file: string | null;
  selected_map_id: string | null;
  selected_map_yaml: string | null;
  nodes: NodeCheck[];
  maps: SavedMapInfo[];
  message: string | null;
}

export interface SystemHealth {
  backend_ok: boolean;
  ros_connected: boolean;
  ros_thread_alive: boolean;
  websocket_clients: number;
  action_server_ready: boolean;
  map_received: boolean;
  pose_received: boolean;
  last_map_update: string | null;
  last_pose_update: string | null;
  last_error: string | null;
}

export interface DashboardSnapshot {
  map: MapSnapshot;
  pose: RobotPose;
  status: RobotStatus;
  navigation: NavigationTaskState;
  health: SystemHealth;
}

export interface BackendEvent<T> {
  type: string;
  payload: T;
}
