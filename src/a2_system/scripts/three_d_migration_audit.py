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


def nested(data: dict[str, Any], *keys: str, default: Any = None) -> Any:
    current: Any = data
    for key in keys:
        if not isinstance(current, dict):
            return default
        current = current.get(key, default)
    return current


def main() -> int:
    errors: list[str] = []
    try:
        slam_cfg = load_yaml(SRC_ROOT / "a2_system" / "config" / "slam_3d.yaml")
        localization_cfg = load_yaml(SRC_ROOT / "a2_system" / "config" / "localization_3d.yaml")
        scan_mission_cfg = load_yaml(SRC_ROOT / "a2_system" / "config" / "scan_mission_3d.yaml")
        using_3d_profiles = True
    except FileNotFoundError as exc:
        using_3d_profiles = False
        errors.append(f"missing 3D profile file: {exc.filename}")
        slam_cfg = load_yaml(SRC_ROOT / "a2_system" / "config" / "slam.yaml")
        localization_cfg = load_yaml(SRC_ROOT / "a2_system" / "config" / "localization.yaml")
        scan_mission_cfg = load_yaml(SRC_ROOT / "a2_system" / "config" / "scan_mission.yaml")

    nav2_cfg = load_yaml(SRC_ROOT / "a2_system" / "config" / "nav2_3d.yaml")
    map_manager_cfg = load_yaml(SRC_ROOT / "a2_system" / "config" / "map_manager.yaml")
    web_3d_cfg = load_yaml(REPO_ROOT / "web_console" / "backend" / "config.3d.yaml")
    jt128_launch = (SRC_ROOT / "a2_bringup" / "launch" / "jt128_3d_navigation.launch.py").read_text(
        encoding="utf-8"
    )
    start_script = (SRC_ROOT / "a2_system" / "tools" / "start_jt128_3d_stack.sh").read_text(
        encoding="utf-8"
    )

    slam_params = slam_cfg.get("slam_manager", {}).get("ros__parameters", {})
    map_manager_params = map_manager_cfg.get("map_manager", {}).get("ros__parameters", {})
    localization_params = localization_cfg.get("localization_gate", {}).get("ros__parameters", {})
    scan_mission_params = scan_mission_cfg.get("auto_scan_mission", {}).get("ros__parameters", {})
    loc_lifecycle = nav2_cfg.get("lifecycle_manager_localization", {}).get("ros__parameters", {}).get("node_names", [])

    print("=== A2 2D->3D Migration Audit ===")
    print(f"repo_root={REPO_ROOT}")
    print(f"using_3d_profile_files={using_3d_profiles}")
    print("")

    if slam_params.get("primary_map_representation") != "pointcloud_map_3d":
        errors.append("primary_map_representation is not pointcloud_map_3d")
    if slam_params.get("mapping_stack_profile") != "front_lidar_pointcloud_3d":
        errors.append("slam_3d.mapping_stack_profile is not front_lidar_pointcloud_3d")
    if localization_params.get("input_pose_topic") != "/a2/relocalization/pose":
        errors.append("localization_gate input_pose_topic is not /a2/relocalization/pose")
    if scan_mission_params.get("pose_goal_topic") != "/a2/nav3/goal_pose":
        errors.append("scan_mission pose_goal_topic is not /a2/nav3/goal_pose")
    if scan_mission_params.get("navigate_action_name") != "/navigate_to_pose":
        errors.append("scan_mission_3d navigate_action_name must be /navigate_to_pose")
    if nested(web_3d_cfg, "ros", "localization_pose_topic") != "/a2/relocalization/pose":
        errors.append("web config.3d localization_pose_topic is not /a2/relocalization/pose")
    if nested(web_3d_cfg, "stack", "start_script", default="").endswith("start_jt128_3d_stack.sh") is False:
        errors.append("web config.3d stack.start_script does not point to start_jt128_3d_stack.sh")
    if "localization_3d.yaml" not in jt128_launch:
        errors.append("jt128_3d_navigation.launch.py does not load localization_3d.yaml")
    if "start_static_tf:=true" not in start_script:
        errors.append("start_jt128_3d_stack.sh must pass start_static_tf:=true")

    if errors:
        print("[RESULT] FAIL: 3D industrial primary path is NOT fully configured.")
        for err in errors:
            print(f"  - ERROR: {err}")
    else:
        print("[RESULT] PASS: 3D industrial primary path is ACTIVE.")

    print("")
    print("[Current primary representations]")
    print(f"- primary_map_representation={slam_params.get('primary_map_representation', '<unset>')}")
    print(f"- localization_representation={slam_params.get('localization_representation', '<unset>')}")
    print(f"- navigation_representation={slam_params.get('navigation_representation', '<unset>')}")
    print(f"- web_map_representation={slam_params.get('web_map_representation', '<unset>')}")
    print("")
    print("[3D primary contract]")
    mapping_profile = slam_params.get("mapping_stack_profile", "<unset>")
    projection_enabled = slam_params.get("pointcloud_projection_enabled", "<unset>")
    print(f"- mapping_stack_profile={mapping_profile}")
    print(f"- pointcloud_projection_enabled={projection_enabled}")
    print(f"- localization_gate.input_pose_topic={localization_params.get('input_pose_topic', '<unset>')}")
    print(f"- map_manager.occupancy_topic={map_manager_params.get('occupancy_topic', '<unset>')}")
    print(f"- map_manager.map_representation={map_manager_params.get('map_representation', '<unset>')}")
    print(f"- scan_mission.map_topic={scan_mission_params.get('map_topic', '<unset>')}")
    print(f"- scan_mission.pose_topic={scan_mission_params.get('pose_topic', '<unset>')}")
    print(f"- scan_mission.pose_msg_type={scan_mission_params.get('pose_msg_type', '<unset>')}")
    print(f"- scan_mission.navigation_backend={scan_mission_params.get('navigation_backend', '<unset>')}")
    print(f"- scan_mission.pose_goal_topic={scan_mission_params.get('pose_goal_topic', '<unset>')}")
    print(f"- scan_mission.navigate_action_name={scan_mission_params.get('navigate_action_name', '<unset>')}")
    print("")
    print("[Legacy 2D compatibility still present]")
    print(f"- lifecycle_manager_localization={loc_lifecycle}")
    print("- nav2_stack.yaml remains available only as a fallback configuration")
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
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
