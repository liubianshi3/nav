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
    result.require(params.get("input_pose_topic") == "/amcl_pose", "localization_gate must consume /amcl_pose in Nav2 AMCL mode")
    result.require(
        params.get("input_pose_msg_type") == "geometry_msgs/msg/PoseWithCovarianceStamped",
        "localization_gate must consume geometry_msgs/msg/PoseWithCovarianceStamped in AMCL mode",
    )
    result.require(
        not bool(params.get("pose_transient_local", True)),
        "localization_gate must use volatile QoS",
    )
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
    result.require(params.get("profile") == "hesai_jt128_front", "real lidar profile must be JT128 front")
    result.require(params.get("driver_mode") == "dedicated_hesai_ros_driver", "real lidar must use the dedicated Hesai JT128 driver")
    result.require(params.get("input_topic") == "/jt128/front/points", "real lidar input must be /jt128/front/points")
    result.require(params.get("output_topic") == "/jt128/front/points", "real lidar output must stay JT128-native")


def check_goal_bridge(result: CheckResult) -> None:
    params = get(load_yaml(CONFIG_DIR / "nav2.yaml"), "goal_bridge", "ros__parameters", default={})
    result.require(params.get("navigation_backend") == "nav2", "goal_bridge must default to nav2")
    result.require(
        params.get("pose_goal_topic") == "/a2/nav3/goal_pose",
        "goal_bridge pose_goal_topic must be /a2/nav3/goal_pose",
    )
    result.require(params.get("map_frame") == "map", "goal_bridge map_frame must be map")
    result.require(bool(params.get("require_map_frame", False)), "goal_bridge must reject non-map goals by default")
    result.require(float(params.get("goal_timeout_sec", 0.0)) > 0.0, "goal_bridge must define goal_timeout_sec")
    result.require(float(params.get("action_wait_timeout_sec", 0.0)) > 0.0, "goal_bridge must define action_wait_timeout_sec")


def check_pose_goal_controller_3d(result: CheckResult) -> None:
    params = get(load_yaml(CONFIG_DIR / "pose_goal_controller_3d.yaml"), "pose_goal_controller_3d", "ros__parameters", default={})
    result.require(params.get("goal_topic") == "/a2/nav3/goal_pose", "3D pose controller goal_topic must remain /a2/nav3/goal_pose")
    result.require(params.get("pose_topic") == "/a2/relocalization/pose", "3D pose controller pose_topic must remain /a2/relocalization/pose")
    result.require(params.get("cmd_topic") == "/cmd_vel", "3D pose controller cmd_topic must remain /cmd_vel")
    result.require(bool(params.get("dry_run", False)), "3D pose controller must default to dry_run")
    result.require(bool(params.get("require_localization_ok", False)), "3D pose controller must require localization_ok")
    result.require(bool(params.get("require_obstacle_cloud", False)), "3D pose controller must require obstacle pointcloud")
    result.require(params.get("obstacle_cloud_topic") == "/jt128/front/points", "3D pose controller obstacle_cloud_topic must remain /jt128/front/points")
    result.require(float(params.get("obstacle_cloud_timeout_sec", 999.0)) <= 1.5, "3D pose controller obstacle cloud timeout must be bounded")


def check_scan_mission(result: CheckResult) -> None:
    params = get(load_yaml(CONFIG_DIR / "scan_mission.yaml"), "auto_scan_mission", "ros__parameters", default={})
    waypoint_yaml = load_yaml(CONFIG_DIR / "scan_waypoints.example.yaml")
    waypoints = waypoint_yaml.get("waypoints", [])
    scan_launch = (BRINGUP_DIR / "scan_mission.launch.py").read_text(encoding="utf-8")

    result.require(params.get("navigation_backend") == "nav2", "scan mission must default to nav2")
    result.require(params.get("pose_topic") == "/amcl_pose", "scan mission must consume /amcl_pose in AMCL mode")
    result.require(
        params.get("pose_msg_type") == "geometry_msgs/msg/PoseWithCovarianceStamped",
        "scan mission pose_msg_type must be geometry_msgs/msg/PoseWithCovarianceStamped in AMCL mode",
    )
    result.require(params.get("pointcloud_topic") == "/jt128/front/points", "scan mission must use JT128 front pointcloud")
    result.require(params.get("navigate_action_name") == "/navigate_to_pose", "scan mission must keep Nav2 action as fallback")
    result.require(params.get("mission_status_topic") == "/a2/scan_mission/status", "scan mission status topic must be stable")
    result.require(params.get("mission_report_topic") == "/a2/scan_mission/report", "scan mission report topic must be stable")
    result.require(params.get("goal_frame") == "map", "scan mission goals must be emitted in map frame")
    result.require(bool(params.get("require_map_frame", False)), "scan mission must require map frame by default")
    result.require(
        bool(params.get("validate_waypoints_against_map", False)),
        "scan mission must validate waypoints against /map by default",
    )
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


def check_launch_defaults(result: CheckResult) -> None:
    for name in (
        "bringup.launch.py",
        "nav2.launch.py",
        "localization.launch.py",
        "scan_mission.launch.py",
    ):
        text = (BRINGUP_DIR / name).read_text(encoding="utf-8")
        if name != "scan_mission.launch.py":
            result.require(
                'DeclareLaunchArgument("real_localization_mode", default_value="amcl")' in text,
                f"{name} must default real_localization_mode to amcl",
            )
        else:
            result.require("waypoints_file" in text, "scan_mission.launch.py must expose a waypoints_file argument")
            result.require("dry_run" in text, "scan_mission.launch.py must expose a dry_run argument")


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
        "slam.yaml must default mapping_stack_profile to slam_toolbox for Nav2-first navigation",
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
        '("localization", "3D localization gate", "localization_gate")' in text,
        "web 3D navigation startup contract must wait for localization_gate",
    )
    result.require(
        '"manual localization"' not in text,
        "web navigation startup contract must not require manual localization by default",
    )
    result.require(
        '"A2_REAL_LOCALIZATION_MODE": "uslam_odom"' in text,
        "web 3D navigation startup must explicitly set A2_REAL_LOCALIZATION_MODE=uslam_odom",
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


def check_pcd_relocalization(result: CheckResult) -> None:
    params = get(load_yaml(CONFIG_DIR / "pcd_relocalization_3d.yaml"), "pcd_relocalizer_3d", "ros__parameters", default={})
    result.require(params.get("matcher_backend") == "ndt", "matcher_backend must be ndt")
    result.require(params.get("live_cloud_topic") == "/jt128/front/points", "live_cloud_topic must remain /jt128/front/points")
    result.require(params.get("odom_topic") == "/jt128/dlio/odom", "odom_topic must remain /jt128/dlio/odom")
    result.require(params.get("pose_topic") == "/a2/relocalization/pose", "pose_topic must remain /a2/relocalization/pose")
    result.require(params.get("status_topic") == "/a2/relocalization/status", "status_topic must remain /a2/relocalization/status")
    result.require(not bool(params.get("auto_seed_identity", True)), "auto_seed_identity must remain false")
    result.require(float(params.get("ndt_resolution", 0.0)) > 0.0, "ndt_resolution must be > 0")
    result.require(int(params.get("ndt_min_effective_correspondences", 0)) >= 50, "ndt_min_effective_correspondences must be >= 50")
    result.require(float(params.get("max_translation_correction", 999.0)) <= 3.0, "max_translation_correction must remain bounded")
    result.require(float(params.get("max_rotation_correction_deg", 999.0)) <= 15.0, "max_rotation_correction_deg must remain bounded")


def main() -> int:
    result = CheckResult()
    check_nav2_stack(result)
    check_localization(result)
    check_pcd_relocalization(result)
    check_state_bridge(result)
    check_real_lidar(result)
    check_goal_bridge(result)
    check_pose_goal_controller_3d(result)
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
