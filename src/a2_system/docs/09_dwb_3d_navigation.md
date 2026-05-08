# 09 — DWB + 3D Costmap Navigation

## Why This Exists

The A2 3D navigation pipeline originally used `pose_goal_controller_3d`, a 252-line P-controller
with a 1.5m dead-zone that stops the robot at the goal but cannot plan around obstacles. Replacing
it with Nav2's full planning stack gives the A2 real local planning with obstacle avoidance,
lateral movement, and velocity smoothing — all mature, tested components from the Nav2 ecosystem.

## What Changed

### Before (pose_goal_controller_3d)

```
Goal pose → P-controller → /cmd_vel → a2_control_bridge → robot
                                  ↑
                    No obstacle avoidance
                    No costmap awareness
                    Max range: 1.5m
                    No lateral movement
```

### After (Nav2 3D DWB stack)

```
Goal pose → BT navigator → DWB controller → /cmd_vel → velocity_smoother → /cmd_vel
                                                                              ↓
                    ┌───────────────────────────────────────── collision_monitor
                    ↓                                                     ↓
    global_costmap (static + obstacle + inflation)          /cmd_vel_safe → a2_control_bridge
    local_costmap  (obstacle + traversability + inflation)
                    ↑
    /jt128/front/points (live scan)
    /a2/traversability/obstacle_points (traversability→cloud bridge)
```

## Configuration

### DWB Local Planner (`nav2_3d.yaml`)

Key settings that enable 3D-capable, omnidirectional motion:

```yaml
controller_server:
  ros__parameters:
    controller_plugin_ids: ["FollowPath"]
    FollowPath.plugin: "dwb_core::DWBLocalPlanner"
    
    # Lateral (y-axis) motion — essential for quadruped omnidirectional capability
    max_vel_y: 0.12
    min_vel_y: -0.12
    vy_samples: 5
    
    # Linear limits
    max_vel_x: 0.8
    min_vel_x: -0.3       # slight reverse allowed
    vx_samples: 10
    
    # Angular
    max_vel_theta: 0.6
    vtheta_samples: 10
```

DWB critics: `PathAlign`, `GoalAlign`, `PathDist`, `GoalDist`, `ObstacleFootprint`,
`BaseObstacle`, `PreferForward`, `Twirling`, `Oscillation`.

### Velocity Smoother

```yaml
velocity_smoother:
  ros__parameters:
    odom_topic: "/jt128/dlio/odom"   # full 3D odometry for closed-loop feedback
    velocity_feedback: "CLOSED_LOOP"  # corrects for slippage
    max_velocity: [0.8, 0.12, 0.6]
    acceleration: [0.4, 0.1, 0.3]
    deadband_velocity: [0.02, 0.02, 0.02]
```

### 3D Local Costmap

```yaml
local_costmap:
  ros__parameters:
    width: 80     # 8m × 8m rolling window at 0.1m resolution
    height: 80
    resolution: 0.1
    rolling_window: true
    
    plugins: ["obstacle_layer", "traversability_obstacle_layer", "inflation_layer"]
    
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      observation_sources: "jt128_scan"
      jt128_scan:
        topic: "/jt128/front/points"
        sensor_frame: "jt128_front_link"
        min_height: 0.05
        max_height: 0.85
        obstacle_max_range: 6.0
    
    traversability_obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      observation_sources: "traversability_cloud"
      traversability_cloud:
        topic: "/a2/traversability/obstacle_points"
        sensor_frame: "map"
        min_height: 0.0
        max_height: 0.5
    
    inflation_layer:
      inflation_radius: 0.35
      cost_scaling_factor: 3.0
```

### Traversability Grid Bridge (`traversability_to_obstacle_cloud.py`)

The ground segmentation node publishes `/a2/traversability` as an `OccupancyGrid` in the map
frame. Nav2's `ObstacleLayer` cannot consume `OccupancyGrid` directly — it requires
`PointCloud2`. The bridge converts cells with value ≥ 90 (steep/non-traversable terrain) and
unknown cells (−1, conservative) into 3D obstacle points at z=0.15.

Publishes at 2 Hz to `/a2/traversability/obstacle_points`.

### Topic Chain for Collision Safety

```
Nav2 /cmd_vel → collision_monitor → /cmd_vel_safe → a2_control_bridge
                ↑
    /a2/obstacle/points (ground_seg live obstacles)
    /a2/traversability/obstacle_points (traversability map)
```

The `collision_monitor` (from `nav2_collision_monitor` package) applies two safety polygons:

| Polygon | Dimensions (forward/side/rear) | Action |
|---|---|---|
| `PolygonStop` | 0.5m / 0.4m / 0.3m | Full stop (≥3 obstacle points inside) |
| `PolygonSlow` | 0.9m / 0.7m / 0.5m | Slow to 30% (≥2 obstacle points inside) |

Dual safety channels: `collision_monitor` for spatial safety + `nav_health_monitor` for
system-level diagnostics (sets `max_speed_scale` on degraded state).

### Lifecycle Management

All nodes managed by Nav2's `lifecycle_manager_navigation` with 12s bond timeout, including
`collision_monitor`. This ensures the collision monitor activates and deactivates together
with the rest of the navigation stack.

## Files

| File | Role |
|---|---|
| `a2_system/config/nav2_3d.yaml` | Full 3D Nav2 config (DWB, costmap, smoother, lifecycle) |
| `a2_system/scripts/traversability_to_obstacle_cloud.py` | OccupancyGrid→PointCloud2 bridge |
| `a2_system/config/collision_monitor.yaml` | Stop/Slowdown polygon config |
| `a2_bringup/launch/nav2_3d.launch.py` | Nav2 3D stack launcher |
| `a2_bringup/launch/jt128_3d_navigation.launch.py` | Top-level 3D nav launcher |
| `a2_system/config/motion_limits.yaml` | `cmd_topic: /cmd_vel_safe` |

## Build Verification

```bash
colcon build --packages-select a2_system a2_bringup nav2_integration
```

All packages build cleanly. Runtime verification requires the full sensor pipeline
(DLIO, NDT, ground_seg) to be running.

## Dependencies

- `nav2_bringup` (Humble)
- `nav2_collision_monitor` (Humble)
- `sensor_msgs_py` (Python PointCloud2 helpers)
