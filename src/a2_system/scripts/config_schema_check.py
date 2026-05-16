#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import yaml


class UniqueKeyLoader(yaml.SafeLoader):
    pass


def construct_mapping(loader: UniqueKeyLoader, node: yaml.nodes.MappingNode, deep: bool = False) -> dict[str, Any]:
    mapping: dict[str, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise ValueError(f"duplicate YAML key `{key}`")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, construct_mapping)


def default_config_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("a2_system")) / "config"
    except Exception:
        return Path(__file__).resolve().parents[1] / "config"


def load_yaml_unique(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        payload = yaml.load(handle, Loader=UniqueKeyLoader)
    return payload or {}


class AuditResult:
    def __init__(self) -> None:
        self.errors: list[str] = []
        self.warnings: list[str] = []

    def require(self, condition: bool, message: str) -> None:
        if not condition:
            self.errors.append(message)

    def warn_if(self, condition: bool, message: str) -> None:
        if condition:
            self.warnings.append(message)


def get(data: dict[str, Any], *keys: str, default: Any = None) -> Any:
    current: Any = data
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def require_keys(result: AuditResult, data: dict[str, Any], path: str, keys: list[str]) -> None:
    target = data
    if path:
        for token in path.split("."):
            target = target.get(token, {}) if isinstance(target, dict) else {}
    for key in keys:
        result.require(isinstance(target, dict) and key in target, f"{path or '<root>'} missing required key `{key}`")


def audit_scan_mission(result: AuditResult, config_dir: Path) -> None:
    data = load_yaml_unique(config_dir / "scan_mission.yaml")
    params = get(data, "auto_scan_mission", "ros__parameters", default={})
    require_keys(
        result,
        data,
        "auto_scan_mission.ros__parameters",
        [
            "dry_run",
            "waypoints_file",
            "map_topic",
            "pose_topic",
            "odom_topic",
            "localization_ok_topic",
            "real_report_topic",
            "mission_status_topic",
            "mission_report_topic",
            "mission_progress_topic",
            "mission_goal_topic",
            "goal_frame",
            "navigate_action_name",
            "preflight_timeout_sec",
            "goal_result_timeout_sec",
            "position_pass_threshold_m",
            "yaw_pass_threshold_rad",
        ],
    )
    result.require(params.get("goal_frame") == "map", "scan_mission goal_frame must be map")
    result.require(str(params.get("map_topic", "")).startswith("/"), "scan_mission map_topic must be absolute")
    result.require(str(params.get("navigate_action_name", "")).startswith("/"), "scan_mission action name must be absolute")
    result.require(float(params.get("preflight_timeout_sec", 0.0)) >= 10.0, "scan_mission preflight timeout too short")
    result.require(float(params.get("position_pass_threshold_m", 999.0)) <= 0.15, "scan_mission position pass threshold too loose")
    result.require(float(params.get("yaw_pass_threshold_rad", 999.0)) <= 0.20, "scan_mission yaw pass threshold too loose")
    result.require(int(params.get("occupied_threshold", 100)) <= 70, "scan_mission occupied_threshold must be conservative")
    result.require(not bool(params.get("allow_unknown_cells", True)), "scan_mission must not allow unknown cells by default")


def audit_nav2_stack(result: AuditResult, config_dir: Path) -> None:
    data = load_yaml_unique(config_dir / "nav2_stack.yaml")
    controller = get(data, "controller_server", "ros__parameters", default={})
    planner = get(data, "planner_server", "ros__parameters", "GridBased", default={})
    goal_checker = get(controller, "general_goal_checker", default={})
    global_costmap = get(data, "global_costmap", "global_costmap", "ros__parameters", default={})
    local_costmap = get(data, "local_costmap", "local_costmap", "ros__parameters", default={})

    require_keys(result, data, "controller_server.ros__parameters", ["controller_frequency", "general_goal_checker"])
    require_keys(result, data, "planner_server.ros__parameters", ["GridBased"])
    result.require(float(controller.get("controller_frequency", 0.0)) >= 15.0, "Nav2 controller_frequency must be >= 15")
    result.require(float(goal_checker.get("xy_goal_tolerance", 999.0)) <= 0.10, "Nav2 xy tolerance too loose")
    result.require(float(goal_checker.get("yaw_goal_tolerance", 999.0)) <= 0.12, "Nav2 yaw tolerance too loose")
    result.require(float(planner.get("tolerance", 999.0)) <= 0.15, "Nav2 planner tolerance too loose")
    result.require(global_costmap.get("global_frame") == "map", "global_costmap global_frame must be map")
    result.require(local_costmap.get("global_frame") == "odom", "local_costmap global_frame must be odom")
    result.require(global_costmap.get("robot_base_frame") == "base_link", "global_costmap robot_base_frame must be base_link")
    result.require(local_costmap.get("robot_base_frame") == "base_link", "local_costmap robot_base_frame must be base_link")


def audit_localization(result: AuditResult, config_dir: Path) -> None:
    data = load_yaml_unique(config_dir / "localization.yaml")
    params = get(data, "localization_gate", "ros__parameters", default={})
    require_keys(
        result,
        data,
        "localization_gate.ros__parameters",
        [
            "input_pose_topic",
            "input_pose_msg_type",
            "status_topic",
            "status_report_topic",
            "max_pose_age_sec",
            "max_xy_variance",
            "max_yaw_variance",
            "pose_transient_local",
        ],
    )
    result.require(params.get("input_pose_topic") == "/a2/relocalization/pose", "localization_gate input must be /a2/relocalization/pose (3D NDT)")
    result.require(
        params.get("input_pose_msg_type") == "geometry_msgs/msg/PoseWithCovarianceStamped",
        "localization_gate input_pose_msg_type must be geometry_msgs/msg/PoseWithCovarianceStamped",
    )
    result.require(params.get("status_topic") == "/a2/localization_ok", "localization status topic mismatch")
    result.require(
        not bool(params.get("pose_transient_local", True)),
        "localization_gate must use volatile QoS",
    )
    result.require(float(params.get("max_pose_age_sec", 999.0)) <= 10.0, "localization max_pose_age_sec too loose")
    result.require(float(params.get("max_xy_variance", 999.0)) <= 0.20, "localization max_xy_variance too loose")
    result.require(float(params.get("max_yaw_variance", 999.0)) <= 0.15, "localization max_yaw_variance too loose")


def audit_native_map(result: AuditResult, config_dir: Path) -> None:
    data = load_yaml_unique(config_dir / "native_map_relay.yaml")
    params = get(data, "native_map_relay", "ros__parameters", default={})
    require_keys(
        result,
        data,
        "native_map_relay.ros__parameters",
        ["profile", "input_topic", "output_topic", "output_frame_id", "publish_rate_hz", "status_topic"],
    )
    result.require(
        str(params.get("input_topic", "")).startswith("/"),
        "native_map_relay input_topic must be absolute",
    )
    result.require(
        str(params.get("output_topic", "")) == "/map",
        "native_map_relay output_topic must stay /map",
    )
    result.require(
        str(params.get("status_topic", "")).startswith("/a2/"),
        "native_map_relay status_topic must be namespaced under /a2/",
    )
    result.require(
        float(params.get("publish_rate_hz", 0.0)) > 0.0,
        "native_map_relay publish_rate_hz must be > 0",
    )


def audit_real_mapping_stack(result: AuditResult, config_dir: Path) -> None:
    slam_cfg = load_yaml_unique(config_dir / "slam.yaml")
    slam_params = get(slam_cfg, "slam_manager", "ros__parameters", default={})
    mapping_profile = str(slam_params.get("mapping_stack_profile", "") or "").strip()
    result.require(
        mapping_profile in {"front_lidar_pointcloud_3d", "slam_toolbox", "native_global_map", "projected_occupancy"},
        "slam.yaml mapping_stack_profile must be a known 3D-first or legacy fallback profile",
    )
    # 3D-first: mapping_stack_profile defaults to front_lidar_pointcloud_3d.
    # Legacy "slam_toolbox" is still accepted as fallback but no longer the primary default.
    result.require(
        mapping_profile == "front_lidar_pointcloud_3d",
        "slam.yaml must default mapping_stack_profile to front_lidar_pointcloud_3d for 3D-first navigation",
    )

    toolbox_cfg = load_yaml_unique(config_dir / "slam_toolbox_mapping.yaml")
    params = get(toolbox_cfg, "slam_toolbox", "ros__parameters", default={})
    require_keys(
        result,
        toolbox_cfg,
        "slam_toolbox.ros__parameters",
        [
            "odom_frame",
            "map_frame",
            "base_frame",
            "scan_topic",
            "mode",
            "resolution",
            "minimum_travel_distance",
            "minimum_travel_heading",
            "do_loop_closing",
        ],
    )
    # Legacy slam_toolbox parameter checks — kept for config integrity when 2D fallback is used
    result.require(params.get("scan_topic") == "/scan", "slam_toolbox_mapping scan_topic must be /scan")
    result.require(params.get("map_frame") == "map", "slam_toolbox_mapping map_frame must be map")
    result.require(params.get("odom_frame") == "odom", "slam_toolbox_mapping odom_frame must be odom")
    result.require(params.get("base_frame") == "base_link", "slam_toolbox_mapping base_frame must be base_link")
    result.require(params.get("mode") == "mapping", "slam_toolbox_mapping mode must stay mapping")
    result.require(float(params.get("resolution", 999.0)) <= 0.05, "slam_toolbox_mapping resolution too coarse")
    result.require(float(params.get("minimum_travel_distance", 999.0)) <= 0.15, "slam_toolbox minimum_travel_distance too large")
    result.require(float(params.get("minimum_travel_heading", 999.0)) <= 0.15, "slam_toolbox minimum_travel_heading too large")
    result.require(bool(params.get("do_loop_closing", False)), "slam_toolbox must keep loop closing enabled")


def audit_all(config_dir: Path) -> AuditResult:
    result = AuditResult()
    for filename in (
        "scan_mission.yaml",
        "nav2_stack.yaml",
        "localization.yaml",
        "native_map_relay.yaml",
        "slam.yaml",
        "slam_toolbox_mapping.yaml",
    ):
        path = config_dir / filename
        result.require(path.exists(), f"missing config file: {path}")
        if path.exists():
            try:
                load_yaml_unique(path)
            except Exception as exc:
                result.errors.append(f"{filename}: {exc}")
    if not result.errors:
        audit_scan_mission(result, config_dir)
        audit_nav2_stack(result, config_dir)
        audit_localization(result, config_dir)
        audit_native_map(result, config_dir)
        audit_real_mapping_stack(result, config_dir)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="A2 production YAML schema and safety audit.")
    parser.add_argument(
        "--config-dir",
        default=str(default_config_dir()),
        help="Directory containing A2 YAML configs.",
    )
    args = parser.parse_args()
    result = audit_all(Path(args.config_dir).expanduser().resolve())
    for warning in result.warnings:
        print(f"WARN: {warning}")
    for error in result.errors:
        print(f"FAIL: {error}")
    if result.errors:
        return 1
    print("PASS: A2 config schema checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
