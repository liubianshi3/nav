# Legacy 2D Launch Files

These launch files are the **old 2D SLAM/AMCL/pointcloud_to_laserscan** primary path.
They have been moved from `launch/` to `launch/legacy/` as part of the 3D-first
architecture convergence (2026-05-16).

## Files

| File | Original Role | 3D-First Replacement |
|---|---|---|
| `slam.launch.py` | slam_toolbox / Fast-LIO mapping | `dlio_mapping.launch.py` (DLIO + OctoMap) |
| `mapping.launch.py` | 2D occupancy grid mapping | OctoMap + pointcloud_map_3d |
| `localization.launch.py` | AMCL localization | `jt128_3d_navigation.launch.py` (NDT relocalization) |
| `nav2.launch.py` | Nav2 2D navigation bringup | Nav2 3D via `nav2_3d.launch.py` |

## Usage

These files are still installed and can be launched manually for fallback/testing:

```bash
ros2 launch a2_bringup legacy/slam.launch.py
ros2 launch a2_bringup legacy/mapping.launch.py
```

They remain referenced by `bringup.launch.py` when `enable_nav2_bringup:=true`
(legacy 2D path only; the default 3D path uses `start_jt128_3d_stack.sh` instead).

## Do NOT

- Add new dependencies to these files
- Use them as default in new scripts or UI
- Reference them from 3D-first launch files
