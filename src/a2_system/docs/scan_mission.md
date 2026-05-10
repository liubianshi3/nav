# Scan Mission Runbook

This document defines the first industrial validation layer for automatic scanning: a waypoint-based mission runner that validates readiness, checks route safety against the active map, executes `NavigateToPose` goals in sequence, and writes a Markdown evidence report.

## Scope

The scan mission is the current repeatable closed-loop validation path for the real A2 body stack:

- front-LiDAR-only input path
- `/map`
- `/amcl_pose`
- `/odom`
- `/a2/localization_ok`
- `/a2/real/report`
- `/navigate_to_pose`
- `/map_manager/manage_map`

Rear LiDAR `.21` must not be assumed available for this mission.

## ROS Contract

Inputs:

- `/map`: `nav_msgs/msg/OccupancyGrid`, expected frame `map`
- `/amcl_pose`: `geometry_msgs/msg/PoseWithCovarianceStamped`
- `/odom`: `nav_msgs/msg/Odometry`
- `/a2/localization_ok`: `std_msgs/msg/Bool`
- `/a2/localization/status`: `std_msgs/msg/String`
- `/a2/real/report`: `std_msgs/msg/String`
- `/a2/map_manager/status`: `std_msgs/msg/String`
- `/a2/map_manager/active_map`: `std_msgs/msg/String`
- `/a2/nav2/status`: `std_msgs/msg/String`

Action and service dependencies:

- `/navigate_to_pose`: `nav2_msgs/action/NavigateToPose`
- `/map_manager/set_mode`: `a2_interfaces/srv/SetMode`
- `/map_manager/manage_map`: `a2_interfaces/srv/ManageMap`

Outputs:

- `/a2/scan_mission/status`: text status in `key=value;...` shape
- `/a2/scan_mission/report`: path to generated Markdown report
- `/a2/scan_mission/progress`: `std_msgs/msg/Float32`, range `[0,1]`
- `/a2/scan_mission/goal`: current `geometry_msgs/msg/PoseStamped` target

## Dry Run

Use dry-run first on the robot before moving:

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/a2_system_ws/install/setup.bash
ros2 launch a2_bringup scan_mission.launch.py dry_run:=true \
  waypoints_file:=/home/unitree/a2_system_ws/src/a2_system/config/scan_waypoints.example.yaml
```

## Map Validation

Dry-run behavior:

- loads and validates the route file
- waits for map, pose, localization readiness, and real readiness
- validates route cells against `/map`
- does not send `NavigateToPose` goals
- does not save a map
- writes the same Markdown report format

## Map Validation

The mission validates each waypoint against the latest `/map` occupancy grid before executing navigation goals:

- rejects waypoints landing on occupied cells (`occupied_threshold`)
- rejects waypoints that violate the configured clearance radius (`min_clearance_cells`)
- rejects unknown cells when `allow_unknown_cells` is false

This validation runs in both dry-run and real runs when `validate_waypoints_against_map` is enabled.

## Real Run

After dry-run passes and the physical area is safe:

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/a2_system_ws/install/setup.bash
ros2 launch a2_bringup scan_mission.launch.py \
  waypoints_file:=/home/unitree/a2_system_ws/src/a2_system/config/scan_waypoints.example.yaml
```

The real run:

- switches map manager mode to `mapping`
- executes waypoints in order
- stops on first failed waypoint by default
- measures final pose error and yaw error
- records localization and readiness drop events
- saves a map on successful completion by default
- writes Markdown, JSON, and CSV reports under `runtime/reports/scan_mission`

## Offline Check

Before deployment:

```bash
source /opt/ros/humble/setup.bash
source /home/dell/a2_system_ws/install/setup.bash
ros2 run a2_system nav_contract_check.py
```

Expected result:

```text
PASS: A2 navigation contract checks passed.
```
