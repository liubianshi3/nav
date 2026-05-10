# 10 — EKF Sensor Fusion (robot_localization)

## Why This Exists

The Autoware NDT scan matcher needs an initial pose guess (`ekf_pose_with_covariance`)
to start each scan match. Without an EKF, this guess came from **open-loop dead reckoning**:
the NDT adapter multiplied the last successful NDT fix (`map→odom`) by the current DLIO
odometry (`odom→base_link`) and published the result.

This has two problems:
1. **No filtering** — odometry noise feeds directly into the initial guess, especially
   the 6-DOF pose from a walking quadruped (vibration, foot slip, leg compliance)
2. **No covariance** — the NDT matcher gets no uncertainty estimate, so it cannot
   distinguish a tight prior from a loose one

An EKF solves both: it fuses DLIO odometry (continuous prediction) with NDT pose
measurements (discrete corrections) into a filtered, smooth `map→base_link` estimate
with proper covariance — which is exactly what the NDT scan matcher expects as its
`input_initial_pose_topic`.

## Architecture

```
                         ┌─────────────┐
  /jt128/dlio/odom ─────→│             │
  (twist, 50 Hz)         │             │
                         │    EKF      │──→ /odometry/filtered
  ndt_pose_with_cov ────→│  (map frame)│    (nav_msgs/Odometry,
  (pose, ~10 Hz)         │             │     map→base_link)
                         │             │
  /imu/data ────────────→│             │
  (angular vel, 50 Hz)   └─────────────┘
                                │
                                ▼
                         ┌─────────────┐
                         │  odom→pose  │──→ ekf_pose_with_covariance
                         │   bridge    │    (PoseWithCovarianceStamped)
                         └─────────────┘
                                │
                                ▼
                         ┌─────────────┐
                         │ NDT scan    │
                         │ matcher     │  (initial guess for scan matching)
                         └─────────────┘
```

### Sensor Configuration

| Input | Topic | Message Type | What's Used | Why |
|---|---|---|---|---|
| `odom0` | `/jt128/dlio/odom` | `Odometry` | Twist (vx,vy,vz, vroll,vpitch,vyaw) | DLIO already fuses LiDAR+IMU internally; we use only its velocity for continuous prediction |
| `pose0` | `ndt_pose_with_covariance` | `PoseWithCovarianceStamped` | Pose (x,y,z, roll,pitch,yaw) | NDT provides absolute map-frame pose corrections |
| `imu0` | `/imu/data` | `Imu` | Angular velocity (ωx,ωy,ωz) | Body IMU from Unitree SDK; independent of LiDAR IMU DLIO uses internally |

### Why Not Feed Raw IMU for Orientation?

DLIO already fuses the LiDAR's internal IMU (`/jt128/front/imu`) with pointcloud data via
its GEO filter. The body IMU (`/imu/data`, from the Unitree SDK) is an independent source
on the robot chassis. Using only angular velocity from the body IMU gives the EKF an
orientation signal that is independent of DLIO's internal filter, avoiding double-counting
while still benefiting from complementary sensing.

### Process Noise Tuning

Quadruped walking gaits produce more prediction noise than wheeled robots:

```yaml
process_noise_covariance:
  # Position: 0.05 m²/s (x,y), 0.1 m²/s (z — vertical oscillation)
  # Orientation: 0.03 rad²/s (r,p), 0.06 rad²/s (yaw drift)
  # Velocity: 0.2 (x,y), 0.4 (z), 0.1 (r,p), 0.15 (yaw)
  # Acceleration: 0.3 (x,y), 0.6 (z)
```

Compared to the default `robot_localization` values (0.05 for everything), this quadruped
tuning is higher on z-axis (bouncing) and yaw (rotational slip), which prevents the EKF
from becoming overconfident in odometry predictions.

## NDT Adapter Change

Previously the NDT adapter published open-loop predictions to `ekf_pose_with_covariance`
on every odometry callback (~50 Hz). With the EKF in place, this is redundant and
would conflict (same topic, two publishers).

**Change**: The NDT adapter's `ndt_initial_pose_topic` parameter defaults to
`/a2/ndt/open_loop_pose` instead of `ekf_pose_with_covariance`. The open-loop
prediction is still available for fallback/debugging but is no longer consumed
by the NDT scan matcher.

## Topic Ownership

| Topic | Published By | Subscribed By |
|---|---|---|
| `ekf_pose_with_covariance` | EKF bridge (this work) | NDT scan matcher |
| `/odometry/filtered` | EKF node | EKF bridge |
| `/a2/ndt/open_loop_pose` | NDT adapter | (unused — fallback only) |
| `/a2/relocalization/pose` | NDT adapter → localization_gate | — |

## Files

| File | Role |
|---|---|
| `a2_system/config/ekf_3d.yaml` | EKF node config (frequency, noise, sensor mappings) |
| `a2_system/scripts/odometry_to_pose_covariance.py` | `Odometry` → `PoseWithCovarianceStamped` bridge |
| `a2_bringup/launch/ekf.launch.py` | EKF + bridge nodes launch |
| `a2_bringup/launch/nav2_3d.launch.py` | Includes `ekf.launch.py` before Nav2 bringup |
| `a2_ndt_adapter/launch/ndt_adapter.launch.py` | `ndt_initial_pose_topic` → `/a2/ndt/open_loop_pose` |

## Dependencies

- `ros-humble-robot-localization` (APT package)
- `ros-humble-tf2-ros`
- Autoware NDT scan matcher (already in workspace)

## Build Verification

```bash
sudo apt-get install -y ros-humble-robot-localization
colcon build --packages-select a2_system a2_bringup a2_ndt_adapter --symlink-install
```

## Runtime Verification

After launching `nav2_3d.launch.py`:

```bash
# EKF node should be running
ros2 node list | grep ekf

# Should see smooth, filtered pose (not raw odometry)
ros2 topic echo /odometry/filtered --once

# Bridge should forward as PoseWithCovarianceStamped
ros2 topic echo /ekf_pose_with_covariance --once

# NDT adapter publishes to fallback topic (for debugging)
ros2 topic echo /a2/ndt/open_loop_pose --once

# Compare: EKF output should be smoother than DLIO odom + NDT raw
ros2 topic echo /jt128/dlio/odom --once
ros2 topic echo ndt_pose_with_covariance --once
```

## What This Unlocks

- **Better NDT convergence**: Filtered initial guess → fewer iterations, more stable matching
- **Smooth localization output**: The `localization_gate` variance checks get meaningful covariance
- **Future**: The `ekf_pose_with_covariance` topic can feed additional consumers (e.g., object tracking, behavior trees that need pose uncertainty)
- **Future**: `publish_tf: true` can be enabled to let the EKF publish `map→odom` (displacing the NDT adapter's raw transform), giving the whole stack a uniformly filtered transform tree
