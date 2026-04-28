# A2 Operations Runbook

This runbook is the standard operating flow for the current front-LiDAR-first A2
navigation and scan mission stack.

## Preconditions

- Use the robot body stack, not the gimbal/cloud-platform Docker path.
- Treat rear LiDAR `.21` as offline unless verified live.
- Use AMCL as the real localization mode.
- Keep `a2_control_bridge` disabled if the current task is mapping, localization,
  web monitoring, or dry-run validation only.

## Local Pre-Deployment Checks

Run on the development machine before copying to the robot:

```bash
cd /home/dell/a2_system_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select a2_system a2_bringup localization_manager nav2_integration tf_manager map_manager slam_manager
source install/setup.bash
ros2 run a2_system config_schema_check.py
ros2 run a2_system nav_contract_check.py
src/a2_system/tools/run_unit_tests.sh
```

Current real mapping default:

- `slam_toolbox` on `/scan + /odom`
- `native_map_relay` remains available only as an explicit fallback profile

## Deploy To Robot

From the development machine:

```bash
cd /home/dell/a2_system_ws
./scripts/deploy_to_a2.sh a2
```

Optional environment variables:

- `REMOTE_WS=/home/unitree/a2_system_ws`
- `BUILD_WEB=1`
- `START_SERVICE=1`

## Robot Dry-Run Scan Mission

Run this before moving the robot:

```bash
ssh a2
cd /home/unitree/a2_system_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch a2_bringup scan_mission.launch.py dry_run:=true \
  waypoints_file:=/home/unitree/a2_system_ws/src/a2_system/config/scan_waypoints.example.yaml
```

Pass condition:

- `/map` received
- `/amcl_pose` received
- `/a2/localization_ok=true`
- `/a2/real/report ready=true`
- every waypoint passes map-cell validation
- report outcome is `dry_run_succeeded`

## Real Scan Mission

Only run after dry-run passes and the physical area is safe:

```bash
ros2 launch a2_bringup scan_mission.launch.py \
  waypoints_file:=/home/unitree/a2_system_ws/src/a2_system/config/scan_waypoints.example.yaml
```

Outputs:

- Markdown report
- JSON report
- CSV report
- optional saved map

Report directory:

```text
/home/unitree/a2_system_ws/runtime/reports/scan_mission
```

## Local Mock Mission Test

Run without robot hardware:

```bash
source /opt/ros/humble/setup.bash
source /home/dell/a2_system_ws/install/setup.bash
ros2 launch a2_bringup scan_mission_mock.launch.py result_mode:=succeeded
```

Supported mock result modes:

- `succeeded`
- `aborted`
- `reject`
- `timeout`

Run the automated mock regression set:

```bash
source /opt/ros/humble/setup.bash
source /home/dell/a2_system_ws/install/setup.bash
/home/dell/a2_system_ws/install/a2_system/share/a2_system/run_mock_scan_mission_tests.sh
```

## Web Console

The web console is served by FastAPI after the frontend is built:

```bash
cd /home/unitree/a2_system_ws/web_console
./scripts/build_frontend.sh
./scripts/run_backend.sh
```

Browser:

```text
http://<robot-ip>:8080
```

Recommended one-click standby entry on the robot:

```bash
/home/unitree/a2_system_ws/install/a2_system/share/a2_system/start_web_console_suite.sh --iface eth0
```

What this standby script does:

- stops residual mapping/navigation bringup processes
- stops known ROS1/native interference helpers
- starts the native front-LiDAR source and validates `/unitree/slam_lidar/points1`
- prepares or rebuilds the web frontend/backend when needed
- restarts `a2-web-console.service`
- leaves the system in web-controlled standby so mapping/navigation can be started from the UI

Camera display is configurable through:

- `ros.camera_image_topic`
- `ros.camera_compressed_topic`
- `camera.enabled`
- `camera.prefer_compressed`

Default camera assumption is only that the A2 has an onboard HD camera. The
actual ROS topic must be verified on the robot with `ros2 topic list`.

## Stop And Recover

Stop current stack:

```bash
ros2 topic echo /a2/scan_mission/status
ros2 action list | grep navigate_to_pose
```

Prefer using the web Stop Navigation button or:

```bash
ros2 action info /navigate_to_pose
```

If the stack is inconsistent:

```bash
/home/unitree/a2_system_ws/install/a2_system/share/a2_system/stop_stack.sh
```

Then restart mapping or navigation mode from the web console or launch files.
