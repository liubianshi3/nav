# 3D Migration Plan

## Purpose

This document freezes the first engineering contracts for migrating the A2 stack
from the current 2D-first navigation/mapping path to a 3D-first path using the
robot's front native LiDAR topic `/unitree/slam_lidar/points1`.

It does not claim that the full 3D stack is already implemented. It defines:

- current 2D hard dependencies
- representation metadata that must become explicit
- the migration phases and rollback boundaries

## Current hard 2D chain

Real mapping:

`/unitree/slam_lidar/points1 -> pointcloud_frame_relay -> /mid360/points -> pointcloud_to_laserscan -> /scan -> slam_toolbox -> /map`

Real navigation:

`/map + /scan + /odom -> AMCL -> /amcl_pose -> Nav2 -> /navigate_to_pose`

Web:

`/map (OccupancyGrid) -> backend MapSnapshot -> frontend MapCanvas (2D canvas)`

Automatic scan:

`/map + /amcl_pose + /navigate_to_pose -> auto_scan_mission / exploration_manager`

## Explicit representation metadata

The first implementation step is to stop treating the representation as implicit.

The repository now carries explicit representation metadata in:

- `src/a2_system/config/slam.yaml`
- `src/a2_system/config/map_manager.yaml`
- `src/map_manager/map_manager/map_manager_node.py`
- `web_console/backend/models.py`
- `web_console/backend/stack_control.py`

Current default values remain:

- `primary_map_representation: occupancy_grid_2d`
- `localization_representation: occupancy_grid_2d`
- `navigation_representation: occupancy_grid_2d`
- `web_map_representation: occupancy_grid_2d`

This keeps today's runtime stable while making future 3D contracts observable.

## Target 3D-first direction

The target architecture is:

`/unitree/slam_lidar/points1 -> 3D SLAM / 3D localization / 3D map store -> 3D navigation core -> Web 3D viewer`

Temporary 2D compatibility artifacts are allowed only as derived products:

- 2D occupancy slice for compatibility
- 2D route overlay for legacy consumers
- 2D exported map for rollback

They must not remain the source of truth.

## Modules expected to change

Hard replacement candidates:

- `mapping.launch.py`
- `nav2.launch.py`
- `localization.launch.py`
- `pointcloud_to_laserscan` usage
- `slam_toolbox` usage
- `AMCL` usage
- `exploration_manager`
- `auto_scan_mission`
- `web_console/frontend/src/components/MapCanvas.tsx`

Modules that may be preserved with contract changes:

- `a2_sdk_bridge`
- `a2_control_bridge`
- `sensor_sync`
- `tf_manager`
- `safety_manager`
- `task_manager`
- `stack_control`

## Phase boundaries

### Phase 0

- Freeze current interfaces and metadata
- Add migration audit tooling
- Do not change runtime defaults

### Phase 1

- Introduce a parallel 3D mapping pipeline for `/unitree/slam_lidar/points1`
- Keep 2D map export as compatibility only

### Phase 2

- Replace AMCL-based localization with 3D localization
- Keep 2D `/amcl_pose` compatibility only if required by rollback path

### Phase 3

- Replace 2D Nav2-first navigation with 3D-first navigation
- Keep 2D goal bridge only as compatibility adapter if necessary

### Phase 4

- Replace `MapCanvas` 2D viewer with a real 3D viewer
- Preserve 2D layer only as optional overlay

### Phase 5

- Rebuild `auto_scan_mission` and exploration against 3D world state

## Validation policy

Every phase must define:

- source topics
- output topics
- frame tree
- timeout behavior
- rollback switch
- log/report path

No phase should delete the previous stage before the new stage has:

- repeatable startup
- diagnosable failures
- real-robot validation path
- rollback
