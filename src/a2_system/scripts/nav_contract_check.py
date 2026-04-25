#!/usr/bin/env python3

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import yaml


def first_existing(candidates: list[Path]) -> Path:
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


SCRIPT_PATH = Path(__file__).resolve()


def package_share(package_name: str) -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package_name))
    except Exception:
        return None


A2_SYSTEM_SHARE = package_share("a2_system")
A2_BRINGUP_SHARE = package_share("a2_bringup")
CONFIG_DIR = first_existing(
    [
        A2_SYSTEM_SHARE / "config" if A2_SYSTEM_SHARE else Path("__missing__"),
        SCRIPT_PATH.parents[3] / "src" / "a2_system" / "config",
        SCRIPT_PATH.parents[2] / "share" / "a2_system" / "config",
        Path.cwd() / "src" / "a2_system" / "config",
    ]
)
BRINGUP_DIR = first_existing(
    [
        A2_BRINGUP_SHARE / "launch" if A2_BRINGUP_SHARE else Path("__missing__"),
        SCRIPT_PATH.parents[3] / "src" / "a2_bringup" / "launch",
        SCRIPT_PATH.parents[2] / "share" / "a2_bringup" / "launch",
        Path.cwd() / "src" / "a2_bringup" / "launch",
    ]
)


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


class CheckResult:
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


def check_nav2_stack(result: CheckResult) -> None:
    nav2 = load_yaml(CONFIG_DIR / "nav2_stack.yaml")
    goal_checker = get(nav2, "controller_server", "ros__parameters", "general_goal_checker", default={})
    controller = get(nav2, "controller_server", "ros__parameters", default={})
    planner = get(nav2, "planner_server", "ros__parameters", "GridBased", default={})
    nav_lifecycle = get(nav2, "lifecycle_manager_navigation", "ros__parameters", "node_names", default=[])
    loc_lifecycle = get(nav2, "lifecycle_manager_localization", "ros__parameters", "node_names", default=[])

    result.require(float(goal_checker.get("xy_goal_tolerance", 999.0)) <= 0.10, "Nav2 xy_goal_tolerance must be <= 0.10m")
    result.require(float(goal_checker.get("yaw_goal_tolerance", 999.0)) <= 0.12, "Nav2 yaw_goal_tolerance must be <= 0.12rad")
    result.require(float(planner.get("tolerance", 999.0)) <= 0.15, "Planner tolerance must be <= 0.15m")
    result.require(float(controller.get("controller_frequency", 0.0)) >= 15.0, "Controller frequency must be >= 15Hz")
    result.require("velocity_smoother" in nav_lifecycle, "velocity_smoother must be managed by lifecycle_manager_navigation")
    result.require("map_server" not in nav_lifecycle, "map_server must not be managed by lifecycle_manager_navigation")
    result.require("amcl" not in nav_lifecycle, "amcl must not be managed by lifecycle_manager_navigation")
    result.require("map_server" in loc_lifecycle and "amcl" in loc_lifecycle, "map_server and amcl must be managed by lifecycle_manager_localization")
    result.require("velocity_smoother" in nav2, "velocity_smoother parameters must exist")


def check_localization(result: CheckResult) -> None:
    params = get(load_yaml(CONFIG_DIR / "localization.yaml"), "localization_gate", "ros__parameters", default={})
    result.require(params.get("input_pose_topic") == "/amcl_pose", "localization_gate must consume /amcl_pose")
    result.require(bool(params.get("pose_transient_local", False)), "localization_gate must subscribe to AMCL with transient local QoS")
    result.require(bool(params.get("latch_valid_pose", False)), "localization_gate must latch recently valid poses")
    result.require(float(params.get("max_pose_age_sec", 999.0)) <= 10.0, "max_pose_age_sec must be bounded for real readiness")
    result.require(float(params.get("latched_pose_timeout_sec", 999.0)) <= 60.0, "latched_pose_timeout_sec must be bounded")
    result.require(float(params.get("max_xy_variance", 999.0)) <= 0.20, "max_xy_variance must be <= 0.20")
    result.require(float(params.get("max_yaw_variance", 999.0)) <= 0.15, "max_yaw_variance must be <= 0.15")


def check_state_bridge(result: CheckResult) -> None:
    params = get(load_yaml(CONFIG_DIR / "state_bridge.yaml"), "a2_state_publisher", "ros__parameters", default={})
    result.require(bool(params.get("flatten_z_in_odom", False)), "odom must flatten Z for Nav2")
    result.require(bool(params.get("planarize_orientation_in_odom", False)), "odom TF must use planar yaw-only orientation for Nav2")
    result.require(len(params.get("pose_covariance_diagonal", [])) == 6, "pose_covariance_diagonal must contain 6 values")
    result.require(len(params.get("twist_covariance_diagonal", [])) == 6, "twist_covariance_diagonal must contain 6 values")


def check_real_lidar(result: CheckResult) -> None:
    params = get(load_yaml(CONFIG_DIR / "real_lidar.yaml"), "real_lidar", "ros__parameters", default={})
    result.require(params.get("driver_mode") == "external_pointcloud", "real lidar must consume robot-native pointcloud")
    result.require(params.get("output_topic") == "/mid360/points", "real lidar output must remain /mid360/points for upper layers")
    result.warn_if(params.get("input_topic") == params.get("output_topic"), "real lidar input and output topics are identical")


def check_goal_bridge(result: CheckResult) -> None:
    params = get(load_yaml(CONFIG_DIR / "nav2.yaml"), "goal_bridge", "ros__parameters", default={})
    result.require(params.get("map_frame") == "map", "goal_bridge map_frame must be map")
    result.require(bool(params.get("require_map_frame", False)), "goal_bridge must reject non-map goals by default")
    result.require(float(params.get("goal_timeout_sec", 0.0)) > 0.0, "goal_bridge must define goal_timeout_sec")
    result.require(float(params.get("action_wait_timeout_sec", 0.0)) > 0.0, "goal_bridge must define action_wait_timeout_sec")


def check_launch_defaults(result: CheckResult) -> None:
    for name in ("bringup.launch.py", "nav2.launch.py", "localization.launch.py"):
        text = (BRINGUP_DIR / name).read_text(encoding="utf-8")
        result.require(
            'DeclareLaunchArgument("real_localization_mode", default_value="amcl")' in text,
            f"{name} must default real_localization_mode to amcl",
        )


def main() -> int:
    result = CheckResult()
    check_nav2_stack(result)
    check_localization(result)
    check_state_bridge(result)
    check_real_lidar(result)
    check_goal_bridge(result)
    check_launch_defaults(result)

    for warning in result.warnings:
        print(f"WARN: {warning}")
    for error in result.errors:
        print(f"FAIL: {error}")
    if result.errors:
        return 1
    print("PASS: A2 navigation contract checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
