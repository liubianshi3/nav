# Interface Contracts

This file locks the ROS 2 interfaces exposed by the host-side A2 stack.

## Topic Contracts

| Topic | Type | Producer | Consumer | Mode | Notes |
|---|---|---|---|---|---|
| `/a2/raw_state` | `a2_interfaces/msg/RobotState` | `a2_sdk_bridge` | `a2_state_publisher` | real | Stable internal robot state contract |
| `/robot_state` | `a2_interfaces/msg/RobotState` | `a2_state_publisher` | `safety_manager`, tools | real | User-facing normalized state |
| `/imu/data` | `sensor_msgs/msg/Imu` | `a2_state_publisher` | state diagnostics | real | Robot body IMU, frame `imu_link` |
| `/jt128/front/imu` | `sensor_msgs/msg/Imu` | JT128 driver path | `sensor_sync`, DLIO | real | Front-lidar IMU, frame `jt128_front_imu_link` |
| `/odom` | `nav_msgs/msg/Odometry` | `a2_state_publisher` | legacy 2D consumers | real | Producer owns `odom->base_link` |
| `/jt128/dlio/odom` | `nav_msgs/msg/Odometry` | DLIO | localization, diagnostics | real | 3D odometry contract |
| `/jt128/front/points` | `sensor_msgs/msg/PointCloud2` | front lidar source | SLAM, safety, projection | real | Canonical front-lidar point cloud, frame `jt128_front_link` |
| `/scan` | `sensor_msgs/msg/LaserScan` | `pointcloud_to_laserscan` | `slam_toolbox`, AMCL, Nav2 | real | 2D projection from `/jt128/front/points` |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `slam_toolbox` or `native_map_relay` | web, `map_manager`, Nav2 | real | Canonical 2D navigation map |
| `/jt128/dlio/map_points` | `sensor_msgs/msg/PointCloud2` | DLIO map node | `map_manager`, PCD tools | real | Canonical 3D pointcloud map |
| `/a2/map/pointcloud_3d` | `sensor_msgs/msg/PointCloud2` | `pointcloud_map_loader` | 3D relocalization | real | Loaded PCD map for relocalization |
| `/a2/relocalization/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `a2_ndt_adapter` or `pcd_relocalizer_3d` | `localization_gate`, 3D control | real | 3D localization output in `map` frame |
| `/a2/relocalization/status` | `std_msgs/msg/String` | `a2_ndt_adapter` or `pcd_relocalizer_3d` | diagnostics, Web, dry-run checks | real | Parseable key-value NDT readiness, score, iteration, and map-cell status |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | A2 camera driver or image transport | web console | real | Preferred low-bandwidth camera stream |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 or `pose_goal_controller_3d` | `a2_control_bridge` | real | Canonical velocity command |
| `/a2/command_limited` | `geometry_msgs/msg/TwistStamped` | `a2_control_bridge` | diagnostics | real | Saturated and gated command |
| `/a2/control/gait_type` | `std_msgs/msg/Int32` | tools/UI | `a2_control_bridge` | real | Unitree A2 `SportClient::SwitchGait` request applied before `Move()` |
| `/a2/control/speed_level` | `std_msgs/msg/Int32` | tools/UI | `a2_control_bridge` | real | Unitree A2 `SportClient::SpeedLevel` request applied with gait control |
| `/a2/control/body_height` | `std_msgs/msg/Float32` | tools/UI | `a2_control_bridge` | real | Optional Unitree A2 `SportClient::BodyHeight` request when enabled |
| `/a2/localization_ok` | `std_msgs/msg/Bool` | `localization_gate` | safety, control | real | Motion gate input |
| `/a2/localization/status` | `std_msgs/msg/String` | `localization_gate` | tools/UI | real | `mode=...;state=...;ready=...;reason=...` |
| `/a2/allow_motion` | `std_msgs/msg/Bool` | `safety_supervisor` | `a2_control_bridge` | real | Final motion allow bit |
| `/a2/estop` | `std_msgs/msg/Bool` | `safety_supervisor` | `a2_control_bridge` | real | Emergency stop channel |
| `/a2/safety/status` | `std_msgs/msg/String` | `safety_supervisor` | tools/UI | real | Unified safety readiness report |
| `/a2/lidar/connected` | `std_msgs/msg/Bool` | `pointcloud_guard` | readiness monitors | real | Front-lidar freshness gate |
| `/a2/lidar/status` | `std_msgs/msg/String` | `pointcloud_guard` | tools/UI | real | Unified lidar readiness report |
| `/a2/sensor_sync/ok` | `std_msgs/msg/Bool` | `sync_monitor` | tools/UI | real | Sensor freshness gate |
| `/a2/sensor_sync/status` | `std_msgs/msg/String` | `sync_monitor` | tools/UI | real | Unified sensor sync report |
| `/a2/exploration/goal` | `geometry_msgs/msg/PoseStamped` | `exploration_manager` | `goal_bridge` | real | Frontier goal |
| `/a2/exploration/state` | `std_msgs/msg/String` | `exploration_manager` | tools/UI | real | High-level exploration state |
| `/a2/exploration/coverage` | `std_msgs/msg/Float32` | `exploration_manager` | tools/UI | real | Coverage ratio in `[0,1]` |
| `/a2/exploration/reason` | `std_msgs/msg/String` | `exploration_manager` | tools/UI | real | Reason codes like `frontier_goal_published` |
| `/a2/slam/status` | `std_msgs/msg/String` | `slam_orchestrator` | tools/UI | real | Unified SLAM readiness report |
| `/a2/slam/mode` | `std_msgs/msg/String` | `slam_orchestrator` | tools/UI | real | `mapping`, `localization`, `navigation`, `idle` |
| `/a2/map_manager/active_map` | `std_msgs/msg/String` | `map_manager` | tools/UI | real | Active map id |
| `/a2/map_manager/status` | `std_msgs/msg/String` | `map_manager` | tools/UI | real | Save, load, list, promote state and active map |
| `/a2/system_mode` | `std_msgs/msg/String` | `map_manager` | tools/UI | real | `mapping`, `navigation`, etc. |
| `/a2/nav2/status` | `std_msgs/msg/String` | `goal_bridge` | tools/UI | real | Unified navigation execution report |
| `/a2/nav3/status` | `std_msgs/msg/String` | `obstacle_aware_local_planner_3d` | tools/UI | real | 3D local navigation execution report, including planner blocked/recovery hints |
| `/a2/nav3/goal_pose` | `geometry_msgs/msg/PoseStamped` | `goal_bridge`, `task_manager` | `obstacle_aware_local_planner_3d` | real | Canonical 3D local-goal contract |
| `/goal_pose_` | `geometry_msgs/msg/PoseStamped` | compatibility publisher | legacy 3D consumers | real | Deprecated compatibility alias for `/a2/nav3/goal_pose` |
| `/a2/task_manager/status` | `std_msgs/msg/String` | `task_manager` | tools/UI | real | Unified task orchestration state |
| `/a2/task_manager/report` | `std_msgs/msg/String` | `task_manager` | tools/UI | real | Latest route mission report path mirrored from `/a2/scan_mission/report` |
| `/a2/scan_mission/status` | `std_msgs/msg/String` | `auto_scan_mission` | tools/UI | real | Waypoint scan mission readiness and execution status |
| `/a2/scan_mission/report` | `std_msgs/msg/String` | `auto_scan_mission` | tools/UI | real | Absolute path to generated Markdown mission report |
| `/a2/scan_mission/progress` | `std_msgs/msg/Float32` | `auto_scan_mission` | tools/UI | real | Mission progress ratio in `[0,1]` |
| `/a2/scan_mission/goal` | `geometry_msgs/msg/PoseStamped` | `auto_scan_mission` | Nav2 diagnostics/UI | real | Current mission goal, frame `map` |
| `/a2/sdk/status` | `std_msgs/msg/String` | `a2_sdk_bridge` | tools/UI | real | Unified SDK readiness report |
| `/a2/control/status` | `std_msgs/msg/String` | `a2_control_bridge` | tools/UI | real | Unified control bridge readiness and motion gate report |
| `/a2/control/state` | `a2_interfaces/msg/ControlState` | `a2_control_bridge` | platform, tools/UI | real/gazebo/mock | Structured control status, last high-level command, SDK return code, and normalized error code |
| `/a2/real/report` | `std_msgs/msg/String` | `real_readiness_monitor` | tools/UI | real | Aggregate stack readiness report with flattened `slam_state/slam_ready/slam_reason` |
| `/a2/ndt/healthy` | `std_msgs/msg/Bool` | `ndt_health_monitor` | `safety_supervisor` | real | NDT health gate (healthy=true) |
| `/a2/ndt/health_status` | `std_msgs/msg/String` | `ndt_health_monitor` | tools/UI | real | `state={healthy|degrading|failed|ndt_not_ready};score=...;ndt_ready=...` |
| `/a2/recovery/cmd_vel` | `geometry_msgs/msg/Twist` | `auto_scan_mission` | `obstacle_aware_local_planner_3d`/`collision_monitor` | real | Recovery FSM velocity hints (safety chain enforced) |
| `/a2/battery` | `sensor_msgs/msg/BatteryState` | `a2_battery_publisher` | web backend/tools | real | Battery snapshot (available/percentage/voltage/charging) |

## Action Contracts

| Action | Type | Client | Server | Purpose |
|---|---|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `goal_bridge`, `auto_scan_mission`, web backend | Nav2 BT navigator | Canonical single-goal navigation action |
| `/run_mission` | `a2_interfaces/action/RunMission` | `task_manager` | `auto_scan_mission` | Mission execution (goal/feedback/result), mirrors `/a2/scan_mission/*` topics |

## Service Contracts

| Service | Type | Provider | Purpose |
|---|---|---|---|
| `/map_manager/manage_map` | `a2_interfaces/srv/ManageMap` | `map_manager` | `save`, `load`, `list`, `promote` |
| `/map_manager/set_mode` | `a2_interfaces/srv/SetMode` | `map_manager` | Switch mapping, localization, navigation mode |
| `/slam_manager/set_mode` | `a2_interfaces/srv/SetMode` | `slam_orchestrator` | Switch SLAM runtime mode |
| `/a2/task_manager/command` | `a2_interfaces/srv/NavCommand` | `task_manager` | Unified command layer for map management, single-goal navigation, initial pose, route asset CRUD, and route mission lifecycle |
| `/a2/control/command` | `a2_interfaces/srv/MotionCommand` | `a2_control_bridge` | Platform-facing motion primitive entrypoint: `stop`, `stand_up`, `stand_down`, `balance_stand`, `recovery_stand`, `damp`, `switch_gait`, `speed_level`, `body_height`, `set_auto_recovery` |

## TF Ownership

| Transform | Owner |
|---|---|
| `map -> odom` | exactly one active source: `slam_toolbox`, `amcl`, or `pcd_relocalizer_3d` |
| `odom -> base_link` | `a2_state_publisher` |
| `base_footprint -> base_link` | `tf_manager` |
| `base_link -> trunk` | `tf_manager` |
| `base_link -> jt128_front_link` | `tf_manager` |
| `base_link -> jt128_front_imu_link` | `tf_manager` |
| `base_link -> camera_link` | `tf_manager` |

## QoS Guidance

- State and control topics: keep last, depth 10 or 20, reliable.
- Point cloud: keep last, depth 5 to 10, reliable first.
- TF static: transient local.
- Exploration and status topics: keep last, depth 10.

## Contract Rules

- Readiness and status topics should prefer the shared text shape `mode=...;state=...;ready=...;reason=...`.
- Driver replacement is only allowed below the front-lidar source boundary.
- Any future package that wants to publish `odom->base_link` or `map->odom` must first remove the current publisher to avoid TF duplication.
