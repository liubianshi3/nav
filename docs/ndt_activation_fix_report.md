# NDT Activation Fix Report

Date: 2026-05-09
Robot: A2 (unitree@192.168.31.49)
Workspace: `/home/unitree/a2_system_ws`

## 1. Original Problem

NDT scan matcher always showed `is_activated=false`, `score=-1.000`, and never published
`ndt_pose_with_covariance`, `transform_probability`, or `iteration_num` topics.
The `/a2/relocalization/status` was stuck at `waiting_seed` or `waiting_score`.

## 2. Root Causes (4 issues found and fixed)

### Cause 1: NDT not activated via `/trigger_node_srv`

**NDT scan matcher is NOT a LifecycleNode** (despite "Node is not activated" diagnostics).
Internal `is_activated_` flag must be set to `true` by calling `/trigger_node_srv`
(`std_srvs/srv/SetBool`) with `data: true`. No component in the stack did this.

**Fix**: Must call `ros2 service call /trigger_node_srv std_srvs/srv/SetBool "{data: true}"`
after NDT starts.

### Cause 2: Score thresholds incompatible with pclomp NDT implementation

The config used `converged_param_type: 1` (NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD)
with threshold `2.3`. Actual scores were ~0.5 for this metric.
Transform probability threshold was `3.0` (impossible — metric ranges ~0-1).

**Fix**: Changed to `converged_param_type: 0` (TRANSFORM_PROBABILITY) with threshold `0.5`.
Also updated `ndt_adapter` score_topic to `transform_probability` and threshold to `0.5`.

### Cause 3: Max iterations limit prevented convergence

With `max_iterations: 30`, NDT often hit the iteration limit without converging.
The convergence check requires `iteration_num < max_iterations`.

**Fix**: Increased `max_iterations` to `60`, increased `trans_epsilon` to `0.05`
for easier convergence.

### Cause 4: Static TF missing for LiDAR frame

NDT requires `jt128_front_link → base_link` transform to convert sensor points.
When nodes were restarted individually, this TF was often lost.

**Fix**: Ensure `static_tf_manager` is running with proper extrinsics config.

## 3. Modified Files

### NDT config (`/opt/ros/humble/share/autoware_ndt_scan_matcher/config/ndt_scan_matcher.param.yaml`)
```yaml
# Changed from: converged_param_type: 1 (NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD)
# Changed from: converged_param_nearest_voxel_transformation_likelihood: 2.3
converged_param_type: 0
converged_param_transform_probability: 0.5

# Changed from: max_iterations: 30
# Changed from: trans_epsilon: 0.01
max_iterations: 60
trans_epsilon: 0.05
```

### NDT adapter launch (`install/a2_ndt_adapter/share/a2_ndt_adapter/launch/ndt_adapter.launch.py`)
```python
# Changed from: 'score_topic': 'nearest_voxel_transformation_likelihood'
# Changed from: 'score_threshold': 2.3
'score_topic': 'transform_probability',
'score_threshold': 0.5,
```

### DLIO config (`src/a2_system/config/dlio_jt128.yaml`)
Restored original IMU calibration parameters after debugging.

## 4. Why These Changes

The Autoware NDT default parameters were designed for a different sensor/setup.
For the A2 robot with JT128 LiDAR:
- Transform probability (range 0-1) is the practical convergence metric
- Score 0.5+ indicates good alignment (actual scores: 0.65-0.76)
- 60 iterations with 0.05 epsilon gives NDT enough room to converge
- The static TF must be explicitly managed when restarting individual nodes

## 5. Verification Commands & Results

### Prerequisites (startup sequence)
```bash
# 1. Start JT128 driver
ros2 launch a2_bringup jt128_driver.launch.py

# 2. Start DLIO odometry
ros2 run direct_lidar_inertial_odometry dlio_odom_node \
  --ros-args -r __node:=jt128_dlio_odom \
  --params-file install/a2_system/share/a2_system/config/dlio_jt128.yaml \
  -r pointcloud:=/jt128/front/points -r imu:=/jt128/front/imu \
  -r odom:=/jt128/dlio/odom

# 3. Start static TF
ros2 run tf_manager static_tf_manager --ros-args -r __node:=jt128_static_tf_manager \
  -p extrinsics_file:=.../jt128_extrinsics.yaml \
  -p tf_file:=.../tf.yaml -p base_height:=0.28

# 4. Start 3D navigation (NDT + adapter + safety + localization gate)
ros2 launch a2_bringup jt128_3d_navigation.launch.py \
  map_id:=dlio_21534_map start_static_tf:=false start_robot_state:=true \
  start_safety:=true enable_motion:=false dry_run:=true enable_nav2_3d:=false

# 5. Activate NDT
ros2 service call /trigger_node_srv std_srvs/srv/SetBool "{data: true}"

# 6. Send initial pose with current DLIO position
python3 scripts/send_initial_pose.py <x> <y>
```

### Verification results (live)
```
/transform_probability       → data: 0.761
/iteration_num               → data: 7
/ndt_pose_with_covariance    → position: (2.66, -1.14, 0.21)
/a2/relocalization/pose      → position received
/a2/relocalization/status    → score=0.761, score_threshold=0.500
/a2/ndt/health_status        → score=0.761, healthy=true
/a2/localization/status      → state=ready, ready=true, reason=pose_ok
TF map→base_link             → translation: (1.77, -0.56, 0.13)
```

## 6. Still Present Risks

1. **DLIO IMU calibration**: Starting DLIO while robot is moving produces NaN/nonsense
   odometry (position drift to 10^32m). Always keep robot STATIONARY during the 3-second
   IMU calibration period.
2. **Static TF fragility**: The `jt128_front_link → base_link` TF is lost whenever
   processes are restarted without the static TF manager. Needs proactive management.
3. **Health monitor convergence**: Requires 5 consecutive good scores to clear.
   After a restart, it takes ~5 seconds to stabilize.
4. **Adapter score freshness**: `score_timeout_sec: 1.0` means if NDT outputs are
   slower than 1Hz, adapter rejects them as stale. Fine-tuning may be needed for
   different map sizes.
