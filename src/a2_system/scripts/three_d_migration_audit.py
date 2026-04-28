#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[3]
SRC_ROOT = REPO_ROOT / "src"


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def main() -> int:
    slam_cfg = load_yaml(SRC_ROOT / "a2_system" / "config" / "slam.yaml")
    nav2_cfg = load_yaml(SRC_ROOT / "a2_system" / "config" / "nav2_stack.yaml")
    localization_cfg = load_yaml(SRC_ROOT / "a2_system" / "config" / "localization.yaml")
    map_manager_cfg = load_yaml(SRC_ROOT / "a2_system" / "config" / "map_manager.yaml")
    scan_mission_cfg = load_yaml(SRC_ROOT / "a2_system" / "config" / "scan_mission.yaml")

    slam_params = slam_cfg.get("slam_manager", {}).get("ros__parameters", {})
    map_manager_params = map_manager_cfg.get("map_manager", {}).get("ros__parameters", {})
    localization_params = localization_cfg.get("localization_gate", {}).get("ros__parameters", {})
    scan_mission_params = scan_mission_cfg.get("auto_scan_mission", {}).get("ros__parameters", {})
    loc_lifecycle = nav2_cfg.get("lifecycle_manager_localization", {}).get("ros__parameters", {}).get("node_names", [])

    print("=== A2 2D->3D Migration Audit ===")
    print(f"repo_root={REPO_ROOT}")
    print("")
    print("[Current primary representations]")
    print(f"- primary_map_representation={slam_params.get('primary_map_representation', '<unset>')}")
    print(f"- localization_representation={slam_params.get('localization_representation', '<unset>')}")
    print(f"- navigation_representation={slam_params.get('navigation_representation', '<unset>')}")
    print(f"- web_map_representation={slam_params.get('web_map_representation', '<unset>')}")
    print("")
    print("[Hard 2D dependencies detected]")
    mapping_profile = slam_params.get("mapping_stack_profile", "<unset>")
    projection_enabled = slam_params.get("pointcloud_projection_enabled", "<unset>")
    print(f"- mapping_stack_profile={mapping_profile}")
    print(f"- pointcloud_projection_enabled={projection_enabled}")
    print(f"- localization_gate.input_pose_topic={localization_params.get('input_pose_topic', '<unset>')}")
    print(f"- lifecycle_manager_localization={loc_lifecycle}")
    print(f"- map_manager.occupancy_topic={map_manager_params.get('occupancy_topic', '<unset>')}")
    print(f"- map_manager.map_representation={map_manager_params.get('map_representation', '<unset>')}")
    print(f"- scan_mission.map_topic={scan_mission_params.get('map_topic', '<unset>')}")
    print(f"- scan_mission.pose_topic={scan_mission_params.get('pose_topic', '<unset>')}")
    print(f"- scan_mission.navigate_action_name={scan_mission_params.get('navigate_action_name', '<unset>')}")
    print("")
    print("[Files that must change for 3D-first migration]")
    for path in (
        "src/a2_bringup/launch/mapping.launch.py",
        "src/a2_bringup/launch/nav2.launch.py",
        "src/a2_bringup/launch/localization.launch.py",
        "src/map_manager/map_manager/map_manager_node.py",
        "src/exploration_manager/exploration_manager/exploration_manager_node.py",
        "src/a2_system/scripts/auto_scan_mission.py",
        "web_console/backend/ros_bridge.py",
        "web_console/frontend/src/components/MapCanvas.tsx",
    ):
        print(f"- {path}")
    print("")
    print("[Recommended immediate next step]")
    print("- front lidar pointcloud should remain the primary map truth")
    print("- eliminate pointcloud_to_laserscan / slam_toolbox / AMCL as primary truth path")
    print("- finish 3D localization, 3D navigation, and Web 3D viewer contracts in parallel")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
