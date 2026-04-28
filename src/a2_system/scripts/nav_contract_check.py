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
DOCS_DIR = first_existing(
    [
        A2_SYSTEM_SHARE / "docs" if A2_SYSTEM_SHARE else Path("__missing__"),
        SCRIPT_PATH.parents[3] / "src" / "a2_system" / "docs",
        SCRIPT_PATH.parents[2] / "share" / "a2_system" / "docs",
        Path.cwd() / "src" / "a2_system" / "docs",
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
TOOLS_DIR = first_existing(
    [
        A2_SYSTEM_SHARE if A2_SYSTEM_SHARE else Path("__missing__"),
        SCRIPT_PATH.parents[3] / "src" / "a2_system" / "tools",
        SCRIPT_PATH.parents[2] / "share" / "a2_system",
        Path.cwd() / "src" / "a2_system" / "tools",
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


def check_scan_mission(result: CheckResult) -> None:
    params = get(load_yaml(CONFIG_DIR / "scan_mission.yaml"), "auto_scan_mission", "ros__parameters", default={})
    waypoint_yaml = load_yaml(CONFIG_DIR / "scan_waypoints.example.yaml")
    waypoints = waypoint_yaml.get("waypoints", [])
    scan_launch = (BRINGUP_DIR / "scan_mission.launch.py").read_text(encoding="utf-8")
    mock_launch = (BRINGUP_DIR / "scan_mission_mock.launch.py").read_text(encoding="utf-8")

    result.require(params.get("navigate_action_name") == "/navigate_to_pose", "scan mission must target /navigate_to_pose")
    result.require(params.get("mission_status_topic") == "/a2/scan_mission/status", "scan mission status topic must be stable")
    result.require(params.get("mission_report_topic") == "/a2/scan_mission/report", "scan mission report topic must be stable")
    result.require(params.get("goal_frame") == "map", "scan mission goals must be emitted in map frame")
    result.require(bool(params.get("require_map_frame", False)), "scan mission must require map frame by default")
    result.require(bool(params.get("validate_waypoints_against_map", False)), "scan mission must validate waypoints against /map")
    result.require(not bool(params.get("allow_unknown_cells", True)), "scan mission must block unknown cells by default")
    result.require(int(params.get("occupied_threshold", 100)) <= 70, "scan mission occupied threshold must be <= 70")
    result.require(int(params.get("min_clearance_cells", -1)) >= 0, "scan mission min_clearance_cells must be >= 0")
    result.require("dry_run" in params, "scan mission must expose dry_run")
    result.require(not bool(params.get("dry_run", True)), "scan mission must default dry_run to false for explicit launch behavior")
    result.require(
        not bool(params.get("dry_run_require_action_server", True)),
        "dry_run must not require action server by default",
    )
    result.require(bool(params.get("stop_on_failure", False)), "scan mission must stop on failure by default")
    result.require(bool(params.get("save_map_on_finish", False)), "scan mission must save a map by default")
    result.require(float(params.get("preflight_timeout_sec", 0.0)) >= 10.0, "scan mission preflight timeout must be >= 10s")
    result.require(float(params.get("goal_result_timeout_sec", 0.0)) >= 30.0, "scan mission goal result timeout must be >= 30s")
    result.require(float(params.get("position_pass_threshold_m", 999.0)) <= 0.15, "scan mission pass position threshold must be <= 0.15m")
    result.require(float(params.get("yaw_pass_threshold_rad", 999.0)) <= 0.20, "scan mission pass yaw threshold must be <= 0.20rad")
    result.require(isinstance(waypoints, list) and len(waypoints) >= 2, "scan waypoint example must contain at least 2 waypoints")
    for index, item in enumerate(waypoints, start=1):
        result.require(isinstance(item, dict), f"scan waypoint #{index} must be a mapping")
        if isinstance(item, dict):
            result.require("x" in item and "y" in item, f"scan waypoint #{index} must contain x and y")
            result.require("yaw" in item, f"scan waypoint #{index} must contain yaw")
    result.require("auto_scan_mission.py" in scan_launch, "scan mission launch must start auto_scan_mission.py")
    result.require("mock_scan_mission_harness.py" in mock_launch, "scan mission mock launch must start mock harness")
    result.require("result_mode" in mock_launch, "scan mission mock launch must expose result_mode")


def check_launch_defaults(result: CheckResult) -> None:
    for name in (
        "bringup.launch.py",
        "nav2.launch.py",
        "localization.launch.py",
        "scan_mission.launch.py",
        "scan_mission_mock.launch.py",
    ):
        text = (BRINGUP_DIR / name).read_text(encoding="utf-8")
        if name not in {"scan_mission.launch.py", "scan_mission_mock.launch.py"}:
            result.require(
                'DeclareLaunchArgument("real_localization_mode", default_value="amcl")' in text,
                f"{name} must default real_localization_mode to amcl",
            )
        elif name == "scan_mission.launch.py":
            result.require("waypoints_file" in text, "scan_mission.launch.py must expose a waypoints_file argument")
            result.require("dry_run" in text, "scan_mission.launch.py must expose a dry_run argument")
        else:
            result.require("result_mode" in text, "scan_mission_mock.launch.py must expose result_mode")
            result.require("dry_run" in text, "scan_mission_mock.launch.py must expose dry_run")


def check_real_entrypoints(result: CheckResult) -> None:
    start_real_stack = (TOOLS_DIR / "start_real_stack.sh").read_text(encoding="utf-8")
    result.require(
        'A2_REAL_LOCALIZATION_MODE:-amcl' in start_real_stack,
        "start_real_stack.sh must default A2_REAL_LOCALIZATION_MODE to amcl",
    )
    result.require(
        'A2_REAL_LOCALIZATION_MODE:-manual_odom' not in start_real_stack,
        "start_real_stack.sh must not default A2_REAL_LOCALIZATION_MODE to manual_odom",
    )


def check_real_mapping_source_contract(result: CheckResult) -> None:
    mapping_launch = (BRINGUP_DIR / "mapping.launch.py").read_text(encoding="utf-8")
    slam_cfg = load_yaml(CONFIG_DIR / "slam.yaml")
    slam_params = get(slam_cfg, "slam_manager", "ros__parameters", default={})
    slam_toolbox_cfg = load_yaml(CONFIG_DIR / "slam_toolbox_mapping.yaml")
    slam_toolbox_params = get(slam_toolbox_cfg, "slam_toolbox", "ros__parameters", default={})
    native_map_cfg = load_yaml(CONFIG_DIR / "native_map_relay.yaml")
    native_map_params = get(native_map_cfg, "native_map_relay", "ros__parameters", default={})

    result.require(
        "native_map_relay" in mapping_launch,
        "mapping.launch.py must support native_map_relay",
    )
    result.require(
        "slam_toolbox" in mapping_launch,
        "mapping.launch.py must support slam_toolbox",
    )
    result.require(
        slam_params.get("mapping_stack_profile") == "slam_toolbox",
        "slam.yaml must default mapping_stack_profile to slam_toolbox",
    )
    result.require(
        slam_toolbox_params.get("scan_topic") == "/scan",
        "slam_toolbox_mapping scan_topic must default to /scan",
    )
    result.require(
        slam_toolbox_params.get("map_frame") == "map",
        "slam_toolbox_mapping map_frame must default to map",
    )
    result.require(
        slam_toolbox_params.get("odom_frame") == "odom",
        "slam_toolbox_mapping odom_frame must default to odom",
    )
    result.require(
        slam_toolbox_params.get("base_frame") == "base_link",
        "slam_toolbox_mapping base_frame must default to base_link",
    )
    result.require(
        native_map_params.get("input_topic") == "/global_map",
        "native_map_relay input_topic must default to /global_map",
    )
    result.require(
        native_map_params.get("output_topic") == "/map",
        "native_map_relay output_topic must default to /map",
    )


def check_web_stack_contract(result: CheckResult) -> None:
    stack_control = first_existing(
        [
            SCRIPT_PATH.parents[3] / "web_console" / "backend" / "stack_control.py",
            Path.cwd() / "web_console" / "backend" / "stack_control.py",
        ]
    )
    if not stack_control.exists():
        result.warnings.append("web stack_control.py not found; skipped web navigation contract check")
        return
    text = stack_control.read_text(encoding="utf-8")
    result.require(
        '("localization", "AMCL localization", "amcl")' in text,
        "web navigation startup contract must wait for AMCL, not manual localization",
    )
    result.require(
        '"manual localization"' not in text,
        "web navigation startup contract must not require manual localization by default",
    )
    result.require(
        '"A2_REAL_LOCALIZATION_MODE": "amcl"' in text,
        "web navigation startup must explicitly set A2_REAL_LOCALIZATION_MODE=amcl",
    )


def check_docs(result: CheckResult) -> None:
    scan_doc = DOCS_DIR / "scan_mission.md"
    interface_doc = DOCS_DIR / "interface_contracts.md"
    operations_doc = DOCS_DIR / "operations_runbook.md"
    fault_doc = DOCS_DIR / "fault_tree.md"
    result.require(scan_doc.exists(), "scan mission runbook must exist")
    if scan_doc.exists():
        scan_text = scan_doc.read_text(encoding="utf-8")
        result.require("Dry Run" in scan_text, "scan mission runbook must document dry-run")
        result.require("Map Validation" in scan_text, "scan mission runbook must document map validation")
    if interface_doc.exists():
        interface_text = interface_doc.read_text(encoding="utf-8")
        result.require("/a2/scan_mission/status" in interface_text, "interface contracts must include scan mission status")
        result.require("/navigate_to_pose" in interface_text, "interface contracts must include NavigateToPose action")
        result.require("/camera/image_raw/compressed" in interface_text, "interface contracts must include camera compressed topic")
    else:
        result.require(False, "interface contracts document must exist")
    result.require(operations_doc.exists(), "operations runbook must exist")
    result.require(fault_doc.exists(), "fault tree must exist")


def main() -> int:
    result = CheckResult()
    check_nav2_stack(result)
    check_localization(result)
    check_state_bridge(result)
    check_real_lidar(result)
    check_goal_bridge(result)
    check_scan_mission(result)
    check_launch_defaults(result)
    check_real_entrypoints(result)
    check_real_mapping_source_contract(result)
    check_web_stack_contract(result)
    check_docs(result)

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
