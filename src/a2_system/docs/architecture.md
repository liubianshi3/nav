# A2 System Architecture

This workspace is organized around a host-side ROS 2 deployment for the Unitree A2 real robot. The active deployment target is the robot body chain with front-LiDAR-first operation.

## Main Data Path

1. `a2_sdk_bridge` resolves the network interface, reads A2 state through SDK2, and publishes `/a2/raw_state`.
2. `a2_state_publisher` normalizes `/a2/raw_state` into `/imu/data`, `/odom`, `/robot_state`, and body TF.
3. The active front-LiDAR source publishes `/jt128/front/points`, with JT128 IMU on `/jt128/front/imu` when the JT128 driver path is used.
4. `sensor_sync` and `pointcloud_guard` validate IMU and pointcloud freshness.
5. `slam_manager` and `localization_manager` provide the active `map -> odom` source.
6. `safety_supervisor` evaluates lidar, state, map, and localization readiness.
7. `goal_bridge` forwards high-level goals into Nav2 or the local 3D pose-goal controller path.
8. `a2_control_bridge` converts `/cmd_vel` into Unitree A2 motion commands with saturation, timeout stop, and motion gating.

## Navigation Split

- 2D path: `/jt128/front/points -> /scan -> slam_toolbox or AMCL -> Nav2`
- 3D path: `JT128 -> DLIO -> /jt128/dlio/map_points and /jt128/dlio/odom -> PCD relocalization -> pose_goal_controller_3d`
- Shared artifact boundary: `map_manager` save, load, list, and promote services

See also:

- [autoware_ndt_adapter_plan.md](/home/dell/a2_system_ws/src/a2_system/docs/autoware_ndt_adapter_plan.md) for the recommended migration path from the current host-side relocalizer toward an Autoware-style NDT adapter while preserving A2 contracts.

## TF Design

- `odom -> base_link` is owned by `a2_state_publisher`.
- `map -> odom` must be owned by exactly one active localization or SLAM source.
- Static body and sensor transforms are published by `tf_manager`.
- JT128 front lidar static frame is `jt128_front_link`.
- JT128 front IMU static frame is `jt128_front_imu_link`.
