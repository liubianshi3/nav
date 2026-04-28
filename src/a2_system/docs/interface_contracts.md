# Interface Contracts

This file locks the ROS 2 interfaces exposed by the host-side A2 stack.

## Topic Contracts

| Topic | Type | Producer | Consumer | Mode | Notes |
|---|---|---|---|---|---|
| `/a2/raw_state` | `a2_interfaces/msg/RobotState` | `a2_sdk_bridge` | `a2_state_publisher` | mock/real | Stable internal robot state contract |
| `/robot_state` | `a2_interfaces/msg/RobotState` | `a2_state_publisher` | `safety_manager`, tools | mock/real | User-facing normalized state |
| `/imu/data` | `sensor_msgs/msg/Imu` | `a2_state_publisher` | `sensor_sync`, SLAM | mock/real | Frame `imu_link` |
| `/odom` | `nav_msgs/msg/Odometry` | `a2_state_publisher` | localization, Nav2, mock map | mock/real | Producer owns `odom->base_link` |
| `/mid360/points` | `sensor_msgs/msg/PointCloud2` | `mid360_wrapper` or real driver | SLAM, safety | mock/real | Frame `lidar_link` |
| `/scan` | `sensor_msgs/msg/LaserScan` | `pointcloud_to_laserscan` | `slam_toolbox`, AMCL, Nav2 | real/gazebo | 2D projection from the front LiDAR point cloud |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `slam_toolbox`, `native_map_relay`, or mock mapper | web, `map_manager`, Nav2 | mock/real/gazebo | Canonical 2D navigation map exposed to the upper stack |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Gazebo camera plugin | future vision modules | gazebo | Optional simulated RGB stream |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo camera plugin | future vision modules | gazebo | Camera intrinsics for simulated RGB stream |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | A2 camera driver or image transport | web console | real/gazebo | Preferred low-bandwidth camera stream for browser display |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | `livox_ros_driver2` | `FAST_LIO`, `livox_custom_to_pointcloud` | real | Internal Livox custom stream for real 3D SLAM |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 or mock nav | `a2_control_bridge`, mock A2 state | mock/real | Canonical velocity command |
| `/a2/command_limited` | `geometry_msgs/msg/TwistStamped` | `a2_control_bridge` | exploration diagnostics | mock/real | Saturated and gated command |
| `/a2/localization_ok` | `std_msgs/msg/Bool` | `localization_gate` | safety, control | mock/real | Motion gate input |
| `/a2/localization/status` | `std_msgs/msg/String` | `localization_gate` | tools/UI | mock/real | `mode=...;state=...;ready=...;reason=...` |
| `/a2/allow_motion` | `std_msgs/msg/Bool` | `safety_supervisor` | `a2_control_bridge` | mock/real | Final motion allow bit |
| `/a2/estop` | `std_msgs/msg/Bool` | `safety_supervisor` | `a2_control_bridge` | mock/real | Emergency stop channel |
| `/a2/safety/status` | `std_msgs/msg/String` | `safety_supervisor` | tools/UI | mock/real | Unified safety readiness report |
| `/a2/sensor_sync/ok` | `std_msgs/msg/Bool` | `sync_monitor` | tools/UI | mock/real | Sensor freshness gate |
| `/a2/sensor_sync/status` | `std_msgs/msg/String` | `sync_monitor` | tools/UI | mock/real | Unified sensor sync report |
| `/a2/exploration/goal` | `geometry_msgs/msg/PoseStamped` | `exploration_manager` | Nav2 bridge or mock nav | mock/real | Frontier goal |
| `/a2/exploration/state` | `std_msgs/msg/String` | `exploration_manager` | tools/UI | mock/real | High-level exploration state |
| `/a2/exploration/coverage` | `std_msgs/msg/Float32` | `exploration_manager` | tools/UI | mock/real | Coverage ratio in `[0,1]` |
| `/a2/exploration/reason` | `std_msgs/msg/String` | `exploration_manager` | tools/UI | mock/real | Reason codes like `frontier_goal_published` |
| `/a2/slam/status` | `std_msgs/msg/String` | `slam_orchestrator` | tools/UI | mock/real | Unified SLAM readiness report |
| `/a2/slam/mode` | `std_msgs/msg/String` | `slam_orchestrator` | tools/UI | mock/real | `mapping`, `localization`, `navigation`, `idle` |
| `/a2/map_manager/active_map` | `std_msgs/msg/String` | `map_manager` | tools/UI | mock/real | Active map id |
| `/a2/map_manager/status` | `std_msgs/msg/String` | `map_manager` | tools/UI | mock/real | Save/load/list/promote state and active map |
| `/a2/system_mode` | `std_msgs/msg/String` | `map_manager` | tools/UI | mock/real | `mapping`, `navigation`, etc. |
| `/a2/nav2/status` | `std_msgs/msg/String` | `goal_bridge` or `mock_nav_controller` | tools/UI | mock/real | Unified navigation execution report |
| `/a2/task_manager/status` | `std_msgs/msg/String` | `task_manager` | tools/UI | mock/real | Unified task orchestration state for map commands, single-goal nav, and route missions |
| `/a2/task_manager/report` | `std_msgs/msg/String` | `task_manager` | tools/UI | real | Latest route mission report path mirrored from `/a2/scan_mission/report` |
| `/a2/scan_mission/status` | `std_msgs/msg/String` | `auto_scan_mission` | tools/UI | real | Waypoint scan mission readiness and execution status |
| `/a2/scan_mission/report` | `std_msgs/msg/String` | `auto_scan_mission` | tools/UI | real | Absolute path to generated Markdown mission report |
| `/a2/scan_mission/progress` | `std_msgs/msg/Float32` | `auto_scan_mission` | tools/UI | real | Mission progress ratio in `[0,1]` |
| `/a2/scan_mission/goal` | `geometry_msgs/msg/PoseStamped` | `auto_scan_mission` | Nav2 diagnostics/UI | real | Current mission goal, frame `map` |
| `/a2/mid360/status` | `std_msgs/msg/String` | `mid360_driver_guard` | tools/UI | mock/real | Unified lidar readiness report |
| `/a2/sdk/status` | `std_msgs/msg/String` | `a2_sdk_bridge` | tools/UI | mock/real | Unified SDK readiness report |
| `/a2/control/status` | `std_msgs/msg/String` | `a2_control_bridge` | tools/UI | mock/real | Unified control bridge readiness and motion gate report |
| `/a2/real/report` | `std_msgs/msg/String` | `real_readiness_monitor` | tools/UI | mock/real | Aggregate stack readiness report with flattened `slam_state/slam_ready/slam_reason` |

## Action Contracts

| Action | Type | Client | Server | Purpose |
|---|---|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `goal_bridge`, `auto_scan_mission`, web backend | Nav2 BT navigator | Canonical single-goal navigation action |

## Service Contracts

| Service | Type | Provider | Purpose |
|---|---|---|---|
| `/map_manager/manage_map` | `a2_interfaces/srv/ManageMap` | `map_manager` | `save`, `load`, `list`, `promote` |
| `/map_manager/set_mode` | `a2_interfaces/srv/SetMode` | `map_manager` | Switch mapping/localization/navigation mode |
| `/slam_manager/set_mode` | `a2_interfaces/srv/SetMode` | `slam_orchestrator` | Switch SLAM runtime mode |
| `/a2/task_manager/command` | `a2_interfaces/srv/NavCommand` | `task_manager` | Unified command layer for map management, single-goal navigation, initial pose, route asset CRUD, and route mission lifecycle |

## TF Ownership

| Transform | Owner |
|---|---|
| `map -> odom` | `slam_orchestrator` in mock mode, real SLAM/localization in real mode |
| `odom -> base_link` | `a2_state_publisher` |
| `base_footprint -> base_link` | `tf_manager` |
| `base_link -> trunk` | `tf_manager` |
| `base_link -> lidar_link` | `tf_manager` |
| `base_link -> imu_link` | `tf_manager` |

## QoS Guidance

- State and control topics:
  keep last, depth 10 or 20, reliable.
- Point cloud:
  keep last, depth 5 to 10, reliable first, best effort only if network pressure forces it.
- TF static:
  transient local.
- Exploration and status topics:
  keep last, depth 10.

## Contract Rule

- `mock`、`gazebo` and `real` mode must keep the same topic names and message types whenever the upper stack consumes them.
- Readiness/status topics should prefer the shared text shape `mode=...;state=...;ready=...;reason=...`.
- Driver replacement is only allowed below the wrapper boundary.
- Any future package that wants to publish `odom->base_link` or `map->odom` must first remove the current publisher to avoid TF duplication.
