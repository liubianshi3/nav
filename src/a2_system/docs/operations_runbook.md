# A2 Operations Runbook

This runbook is the standard operating flow for the current front-LiDAR-first A2 navigation and scan mission stack.

## Preconditions

- Use the robot body stack, not the gimbal or cloud-platform path.
- Treat rear LiDAR `.21` as offline unless verified live.
- Use AMCL as the default real localization mode for Nav2.
- Keep `a2_control_bridge` disabled if the current task is mapping, localization, web monitoring, or dry-run validation only.

## Local Pre-Deployment Checks

Run on the development machine before copying to the robot:

```bash
cd /home/dell/a2_system_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select a2_system a2_bringup localization_manager nav2_integration tf_manager map_manager slam_manager sensor_sync safety_manager
source install/setup.bash
ros2 run a2_system config_schema_check.py
ros2 run a2_system nav_contract_check.py
src/a2_system/tools/run_unit_tests.sh
```

## Deploy To Robot

From the development machine:

```bash
cd /home/dell/a2_system_ws
./scripts/deploy_to_a2.sh a2
```

## JT128 DLIO Mapping

```bash
ssh a2
cd /home/unitree/a2_system_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch a2_bringup dlio_mapping.launch.py
```

Pass condition:

- `/jt128/front/points` is live
- `/jt128/front/imu` is live
- `/jt128/dlio/odom` is live
- `/jt128/dlio/map_points` is live
- `/a2/lidar/connected=true`
- `/a2/sensor_sync/ok=true`

## Nav2 2D Navigation

```bash
ros2 launch a2_bringup bringup.launch.py \
  network_interface:=eth0 \
  enable_nav2_bringup:=true \
  real_localization_mode:=amcl \
  map:=/home/unitree/a2_system_ws/runtime/maps/<map_id>/map.yaml
```

Pass condition:

- `/scan` received
- `/map` received
- `/amcl_pose` received
- `/a2/localization_ok=true`
- `/a2/real/report ready=true`

## JT128 3D Navigation

```bash
ros2 launch a2_bringup jt128_3d_navigation.launch.py map_id:=<saved_map_id> dry_run:=true enable_motion:=false
```

Dry-run readiness check:

```bash
ros2 run a2_system ndt_adapter_dry_run_check.py
```

Pass condition:

- confirm `/a2/map/pointcloud_3d` received
- confirm `/jt128/dlio/odom` received
- publish `/initialpose`
- watch `/a2/relocalization/status` for matcher=autoware_ndt and ready=true
- confirm status includes `score`, `iteration_num`, `map_ready`, and `last_map_returned_points`
- confirm `/a2/relocalization/pose` freshness
- confirm `/a2/localization_ok=true`
- confirm `/a2/lidar/connected=true`
- confirm `/a2/real/report` says ready=true
- confirm `/jt128/front/points` is fresh before accepting a 3D goal
- confirm `/a2/nav3/status` transitions out of `waiting_goal`
- run dry-run goal first
- only enable motion after NDT ready and a clear area is verified

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

## Stop And Recover

Prefer using the web Stop Navigation button or:

```bash
ros2 action info /navigate_to_pose
```

If the stack is inconsistent:

```bash
/home/unitree/a2_system_ws/install/a2_system/share/a2_system/stop_stack.sh
```

Then restart mapping or navigation mode from the web console or launch files.
