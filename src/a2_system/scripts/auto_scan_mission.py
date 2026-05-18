#!/usr/bin/env python3

from __future__ import annotations

import math
import os
import time
import csv
import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from a2_interfaces.srv import ManageMap, SetMode
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, String

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover - depends on runtime environment
    NavigateToPose = None

try:
    from a2_interfaces.action import RunMission as RunMissionAction
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    _HAS_RUN_MISSION_ACTION = True
except ImportError:  # pragma: no cover - depends on runtime environment
    RunMissionAction = None  # type: ignore[assignment]
    ActionServer = None  # type: ignore[assignment]
    CancelResponse = None  # type: ignore[assignment]
    GoalResponse = None  # type: ignore[assignment]
    _HAS_RUN_MISSION_ACTION = False


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def quaternion_to_yaw(orientation: Any) -> float:
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def parse_status_string(payload: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for item in (payload or "").split(";"):
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        fields[key] = value
    return fields


@dataclass
class WaypointSpec:
    waypoint_id: str
    x: float
    y: float
    yaw: float
    dwell_sec: float
    note: str


class AutoScanMission(Node):
    def __init__(self) -> None:
        super().__init__("auto_scan_mission")
        self.mission_name = self.declare_parameter("mission_name", "auto_scan").value
        self.dry_run = bool(self.declare_parameter("dry_run", False).value)
        self.dry_run_require_action_server = bool(
            self.declare_parameter("dry_run_require_action_server", False).value
        )
        self.waypoints_file = os.path.expandvars(
            os.path.expanduser(self.declare_parameter("waypoints_file", "").value)
        )
        self.reports_root = Path(
            os.path.expandvars(
                os.path.expanduser(
                    self.declare_parameter("reports_root", "${HOME}/a2_system_ws/runtime/reports/scan_mission").value
                )
            )
        )
        self.map_topic = self.declare_parameter("map_topic", "/map").value
        self.pose_topic = self.declare_parameter("pose_topic", "/odom").value
        self.pose_msg_type = self.declare_parameter("pose_msg_type", "nav_msgs/msg/Odometry").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.pointcloud_topic = self.declare_parameter("pointcloud_topic", "/jt128/front/points").value
        self.localization_ok_topic = self.declare_parameter("localization_ok_topic", "/a2/localization_ok").value
        self.localization_status_topic = self.declare_parameter(
            "localization_status_topic", "/a2/localization/status"
        ).value
        self.real_report_topic = self.declare_parameter("real_report_topic", "/a2/real/report").value
        self.map_manager_status_topic = self.declare_parameter(
            "map_manager_status_topic", "/a2/map_manager/status"
        ).value
        self.active_map_topic = self.declare_parameter("active_map_topic", "/a2/map_manager/active_map").value
        self.nav_status_topic = self.declare_parameter("nav_status_topic", "/a2/nav2/status").value
        self.mission_status_topic = self.declare_parameter("mission_status_topic", "/a2/scan_mission/status").value
        self.mission_report_topic = self.declare_parameter("mission_report_topic", "/a2/scan_mission/report").value
        self.mission_progress_topic = self.declare_parameter(
            "mission_progress_topic", "/a2/scan_mission/progress"
        ).value
        self.mission_goal_topic = self.declare_parameter("mission_goal_topic", "/a2/scan_mission/goal").value
        self.goal_frame = self.declare_parameter("goal_frame", "map").value
        self.require_map_frame = bool(self.declare_parameter("require_map_frame", True).value)
        self.validate_waypoints_against_map = bool(
            self.declare_parameter("validate_waypoints_against_map", False).value
        )
        self.allow_unknown_cells = bool(self.declare_parameter("allow_unknown_cells", False).value)
        self.occupied_threshold = int(self.declare_parameter("occupied_threshold", 65).value)
        self.min_clearance_cells = int(self.declare_parameter("min_clearance_cells", 0).value)
        self.navigation_backend = self.declare_parameter("navigation_backend", "pose_topic_3d").value
        self.pose_goal_topic = self.declare_parameter("pose_goal_topic", "/a2/nav3/goal_pose").value
        self.navigate_action_name = self.declare_parameter("navigate_action_name", "/navigate_to_pose").value
        self.manage_map_service = self.declare_parameter("manage_map_service", "/map_manager/manage_map").value
        self.set_mode_service = self.declare_parameter("set_mode_service", "/map_manager/set_mode").value
        self.mission_mode = self.declare_parameter("mission_mode", "mapping").value
        self.preflight_timeout_sec = float(self.declare_parameter("preflight_timeout_sec", 20.0).value)
        self.goal_response_timeout_sec = float(
            self.declare_parameter("goal_response_timeout_sec", 5.0).value
        )
        self.goal_result_timeout_sec = float(self.declare_parameter("goal_result_timeout_sec", 120.0).value)
        self.settle_time_sec = float(self.declare_parameter("settle_time_sec", 1.5).value)
        self.stop_on_failure = bool(self.declare_parameter("stop_on_failure", True).value)
        self.require_real_ready = bool(self.declare_parameter("require_real_ready", True).value)
        self.require_localization_ready = bool(
            self.declare_parameter("require_localization_ready", True).value
        )
        self.save_map_on_finish = bool(self.declare_parameter("save_map_on_finish", True).value)
        self.save_map_on_failure = bool(self.declare_parameter("save_map_on_failure", False).value)
        self.saved_map_prefix = self.declare_parameter("saved_map_prefix", "scan_mission").value
        self.position_pass_threshold_m = float(
            self.declare_parameter("position_pass_threshold_m", 0.12).value
        )
        self.position_warn_threshold_m = float(
            self.declare_parameter("position_warn_threshold_m", 0.20).value
        )
        self.yaw_pass_threshold_rad = float(self.declare_parameter("yaw_pass_threshold_rad", 0.15).value)
        self.yaw_warn_threshold_rad = float(self.declare_parameter("yaw_warn_threshold_rad", 0.30).value)

        # Recovery FSM — active on both Nav2 and pose_topic_3d backends (gated by `recovery_enabled`).
        # Recovery Twist hints are published on two topics:
        #   1. recovery_cmd_topic (/a2/recovery/cmd_vel) → consumed by DWA-Lite planner when active
        #   2. recovery_direct_cmd_topic (/cmd_vel)        → consumed by collision_monitor when Nav2
        #      DWB is the active planner (goal is cancelled, so no conflict on /cmd_vel)
        self.recovery_enabled = bool(self.declare_parameter("recovery_enabled", True).value)
        self.recovery_cmd_topic = self.declare_parameter("recovery_cmd_topic", "/a2/recovery/cmd_vel").value
        self.recovery_direct_cmd_topic = self.declare_parameter("recovery_direct_cmd_topic", "/cmd_vel").value
        self.max_recovery_attempts = int(self.declare_parameter("max_recovery_attempts", 2).value)
        self.nav3_status_topic = self.declare_parameter("nav3_status_topic", "/a2/nav3/status").value
        self.recovery_total_budget_sec = float(self.declare_parameter("recovery_total_budget_sec", 12.0).value)
        self.recovery_spin_sec = float(self.declare_parameter("recovery_spin_sec", 6.0).value)
        self.recovery_backup_sec = float(self.declare_parameter("recovery_backup_sec", 3.0).value)
        self.recovery_lateral_sec = float(self.declare_parameter("recovery_lateral_sec", 3.0).value)
        self.recovery_spin_rate = float(self.declare_parameter("recovery_spin_rate", 0.4).value)
        self.recovery_backup_speed = float(self.declare_parameter("recovery_backup_speed", 0.10).value)
        self.recovery_lateral_speed = float(self.declare_parameter("recovery_lateral_speed", 0.10).value)
        self.recovery_publish_hz = float(self.declare_parameter("recovery_publish_hz", 10.0).value)

        self.map_received = False
        self.pointcloud_received = False
        self.latest_map: OccupancyGrid | None = None
        self.latest_pose: PoseWithCovarianceStamped | None = None
        self.latest_odom: Odometry | None = None
        self.localization_ok = False
        self.localization_status_raw = ""
        self.real_report_raw = ""
        self.map_manager_status_raw = ""
        self.active_map = ""
        self.nav_status_raw = ""
        self.localization_drop_events = 0
        self.real_not_ready_events = 0
        self._last_localization_ok: bool | None = None
        self._last_real_ready: bool | None = None
        self._mission_running = False

        self.status_pub = self.create_publisher(String, self.mission_status_topic, 10)
        self.report_pub = self.create_publisher(String, self.mission_report_topic, 10)
        self.progress_pub = self.create_publisher(Float32, self.mission_progress_topic, 10)
        self.goal_pub = self.create_publisher(PoseStamped, self.mission_goal_topic, 10)
        self.pose_goal_pub = self.create_publisher(PoseStamped, self.pose_goal_topic, 10)
        self.recovery_cmd_pub = self.create_publisher(Twist, self.recovery_cmd_topic, 10)
        self.recovery_direct_cmd_pub = self.create_publisher(Twist, self.recovery_direct_cmd_topic, 10)
        self.nav3_status_raw = ""
        self.create_subscription(String, self.nav3_status_topic, self._on_nav3_status, 10)

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, transient_qos)
        if self.pose_msg_type == "nav_msgs/msg/Odometry":
            self.create_subscription(Odometry, self.pose_topic, self.on_odom, 20)
        else:
            self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.on_pose, transient_qos)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self.on_pointcloud, 10)
        self.create_subscription(Bool, self.localization_ok_topic, self.on_localization_ok, 10)
        self.create_subscription(String, self.localization_status_topic, self.on_localization_status, 10)
        self.create_subscription(String, self.real_report_topic, self.on_real_report, 10)
        self.create_subscription(String, self.map_manager_status_topic, self.on_map_manager_status, 10)
        self.create_subscription(String, self.active_map_topic, self.on_active_map, 10)
        self.create_subscription(String, self.nav_status_topic, self.on_nav_status, 10)

        self.manage_map_client = self.create_client(ManageMap, self.manage_map_service)
        self.set_mode_client = self.create_client(SetMode, self.set_mode_service)
        self.navigate_client = (
            ActionClient(self, NavigateToPose, self.navigate_action_name)
            if NavigateToPose is not None and self.navigation_backend == "nav2"
            else None
        )

        self.report_entries: list[dict[str, Any]] = []
        self.route_validation_entries: list[dict[str, Any]] = []
        self.saved_map_id: str | None = None
        self.saved_map_message: str | None = None
        self.final_outcome = "not_started"
        self.final_reason = "not_started"
        self.last_feedback_distance: float | None = None
        self.total_waypoints = 0

    def on_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        self.map_received = True

    def on_pointcloud(self, _msg: PointCloud2) -> None:
        self.pointcloud_received = True

    def on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.latest_pose = msg

    def on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def on_localization_ok(self, msg: Bool) -> None:
        previous = self._last_localization_ok
        self.localization_ok = msg.data
        self._last_localization_ok = msg.data
        if self._mission_running and previous is True and msg.data is False:
            self.localization_drop_events += 1

    def on_localization_status(self, msg: String) -> None:
        self.localization_status_raw = msg.data

    def on_real_report(self, msg: String) -> None:
        self.real_report_raw = msg.data
        report_fields = parse_status_string(msg.data)
        ready = report_fields.get("ready", "false").lower() == "true"
        previous = self._last_real_ready
        self._last_real_ready = ready
        if self._mission_running and previous is True and ready is False:
            self.real_not_ready_events += 1

    def on_map_manager_status(self, msg: String) -> None:
        self.map_manager_status_raw = msg.data

    def on_active_map(self, msg: String) -> None:
        self.active_map = msg.data

    def on_nav_status(self, msg: String) -> None:
        self.nav_status_raw = msg.data

    def publish_status(self, state: str, reason: str, ready: bool = True, **fields: Any) -> None:
        payload = {
            "state": state,
            "ready": str(bool(ready)).lower(),
            "reason": reason,
            "mission": self.mission_name,
        }
        for key, value in fields.items():
            payload[key] = str(value)
        ordered = ";".join(f"{key}={value}" for key, value in payload.items())
        self.status_pub.publish(String(data=ordered))
        self.get_logger().info(f"Mission status: {ordered}")
        # Mirror to the active RunMission action goal handle, if any.
        cb = getattr(self, "_action_feedback_cb", None)
        if cb is not None:
            try:
                cb(state, reason, ordered)
            except Exception as exc:  # pragma: no cover - best-effort feedback
                self.get_logger().warning(f"action feedback hook failed: {exc}")

    def spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.1, deadline - time.monotonic()))

    def wait_for_future(self, future: Any, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def load_waypoints(self) -> list[WaypointSpec]:
        if not self.waypoints_file:
            raise RuntimeError("waypoints_file parameter is empty")
        path = Path(self.waypoints_file)
        if not path.exists():
            raise FileNotFoundError(f"waypoints file not found: {path}")
        with path.open("r", encoding="utf-8") as handle:
            payload = yaml.safe_load(handle) or {}
        raw_waypoints = payload.get("waypoints", payload if isinstance(payload, list) else [])
        if not isinstance(raw_waypoints, list) or not raw_waypoints:
            raise RuntimeError("waypoints file does not contain a non-empty `waypoints` list")
        mission_name_from_file = payload.get("mission_name") if isinstance(payload, dict) else None
        if mission_name_from_file and self.mission_name == "auto_scan":
            self.mission_name = str(mission_name_from_file)
        loaded: list[WaypointSpec] = []
        seen_ids: set[str] = set()
        for index, item in enumerate(raw_waypoints, start=1):
            if not isinstance(item, dict):
                raise RuntimeError(f"waypoint #{index} is not a mapping")
            waypoint_id = (
                str(item.get("id") or item.get("name") or f"wp_{index:02d}")
            )
            if waypoint_id in seen_ids:
                raise RuntimeError(f"duplicate waypoint id: {waypoint_id}")
            seen_ids.add(waypoint_id)
            try:
                x = float(item["x"])
                y = float(item["y"])
                yaw = float(item.get("yaw", 0.0))
                dwell_sec = float(item.get("dwell_sec", 0.0))
            except KeyError as exc:
                raise RuntimeError(f"waypoint {waypoint_id} missing required field: {exc}") from exc
            except (TypeError, ValueError) as exc:
                raise RuntimeError(f"waypoint {waypoint_id} contains non-numeric pose data") from exc
            if not all(math.isfinite(value) for value in (x, y, yaw, dwell_sec)):
                raise RuntimeError(f"waypoint {waypoint_id} contains non-finite values")
            if dwell_sec < 0.0:
                raise RuntimeError(f"waypoint {waypoint_id} dwell_sec must be >= 0")
            loaded.append(
                WaypointSpec(
                    waypoint_id=waypoint_id,
                    x=x,
                    y=y,
                    yaw=normalize_angle(yaw),
                    dwell_sec=dwell_sec,
                    note=str(item.get("note", "")),
                )
            )
        return loaded

    def map_metadata(self) -> dict[str, Any]:
        if self.latest_map is None:
            return {"loaded": False}
        return {
            "loaded": True,
            "frame_id": self.latest_map.header.frame_id,
            "width": int(self.latest_map.info.width),
            "height": int(self.latest_map.info.height),
            "resolution": float(self.latest_map.info.resolution),
            "origin_x": float(self.latest_map.info.origin.position.x),
            "origin_y": float(self.latest_map.info.origin.position.y),
        }

    def world_to_map_cell(self, x: float, y: float) -> tuple[int, int] | None:
        if self.latest_map is None or self.latest_map.info.resolution <= 0.0:
            return None
        origin = self.latest_map.info.origin
        origin_yaw = quaternion_to_yaw(origin.orientation)
        dx = x - float(origin.position.x)
        dy = y - float(origin.position.y)
        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        cell_x = int(math.floor(local_x / float(self.latest_map.info.resolution)))
        cell_y = int(math.floor(local_y / float(self.latest_map.info.resolution)))
        if cell_x < 0 or cell_y < 0:
            return None
        if cell_x >= int(self.latest_map.info.width) or cell_y >= int(self.latest_map.info.height):
            return None
        return cell_x, cell_y

    def map_cell_value(self, cell_x: int, cell_y: int) -> int | None:
        if self.latest_map is None:
            return None
        width = int(self.latest_map.info.width)
        height = int(self.latest_map.info.height)
        if cell_x < 0 or cell_y < 0 or cell_x >= width or cell_y >= height:
            return None
        index = cell_y * width + cell_x
        if index < 0 or index >= len(self.latest_map.data):
            return None
        return int(self.latest_map.data[index])

    def validate_waypoint_against_map(self, waypoint: WaypointSpec) -> tuple[bool, str, dict[str, Any]]:
        details: dict[str, Any] = {
            "waypoint_id": waypoint.waypoint_id,
            "target_x": waypoint.x,
            "target_y": waypoint.y,
            "target_yaw": waypoint.yaw,
            "enabled": self.validate_waypoints_against_map,
        }
        if not self.validate_waypoints_against_map:
            return True, "map_validation_disabled", details
        if self.latest_map is None:
            return False, "map_missing", details
        map_frame = self.latest_map.header.frame_id
        details["map_frame"] = map_frame
        details["goal_frame"] = self.goal_frame
        if self.require_map_frame and map_frame and map_frame != self.goal_frame:
            return False, "map_frame_mismatch", details
        cell = self.world_to_map_cell(waypoint.x, waypoint.y)
        details["map_cell"] = cell
        if cell is None:
            return False, "outside_map_bounds", details

        center_value = self.map_cell_value(*cell)
        details["cell_value"] = center_value
        radius = max(0, self.min_clearance_cells)
        details["min_clearance_cells"] = radius
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                check_cell = (cell[0] + dx, cell[1] + dy)
                value = self.map_cell_value(*check_cell)
                if value is None:
                    details["blocking_cell"] = check_cell
                    return False, "clearance_outside_map", details
                if value < 0 and not self.allow_unknown_cells:
                    details["blocking_cell"] = check_cell
                    details["blocking_value"] = value
                    return False, "unknown_cell_blocked", details
                if value >= self.occupied_threshold:
                    details["blocking_cell"] = check_cell
                    details["blocking_value"] = value
                    return False, "occupied_cell_blocked", details
        return True, "map_cell_free", details

    def validate_route_against_map(self, waypoints: list[WaypointSpec]) -> bool:
        self.route_validation_entries = []
        all_valid = True
        for waypoint in waypoints:
            valid, reason, details = self.validate_waypoint_against_map(waypoint)
            entry = {
                "waypoint_id": waypoint.waypoint_id,
                "valid": valid,
                "reason": reason,
                "details": details,
            }
            self.route_validation_entries.append(entry)
            if not valid:
                all_valid = False
        return all_valid

    def current_pose_dict(self) -> dict[str, Any] | None:
        if self.latest_pose is not None:
            pose = self.latest_pose.pose.pose
            covariance = self.latest_pose.pose.covariance
            frame_id = self.latest_pose.header.frame_id
        else:
            latest_odom = getattr(self, "latest_odom", None)
            if latest_odom is None:
                return None
            pose = latest_odom.pose.pose
            covariance = latest_odom.pose.covariance
            frame_id = latest_odom.header.frame_id
        return {
            "frame_id": frame_id,
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": quaternion_to_yaw(pose.orientation),
            "cov_x": float(covariance[0]),
            "cov_y": float(covariance[7]),
            "cov_yaw": float(covariance[35]),
        }

    def real_ready(self) -> bool:
        fields = parse_status_string(self.real_report_raw)
        return fields.get("ready", "false").lower() == "true"

    def preflight_ready(self) -> tuple[bool, str]:
        if self.navigation_backend == "nav2" and (self.navigate_client is None or NavigateToPose is None):
            return False, "navigate_action_type_missing"
        if self.navigation_backend == "pose_topic_3d" and not self.pointcloud_received:
            return False, "pointcloud_missing"
        if self.navigation_backend == "nav2" and (not self.map_received or self.latest_map is None):
            return False, "map_missing"
        if self.navigation_backend == "nav2" and self.require_map_frame and self.latest_map.header.frame_id and self.latest_map.header.frame_id != self.goal_frame:
            return False, "map_frame_mismatch"
        if self.current_pose_dict() is None:
            return False, "pose_missing"
        if self.require_localization_ready and not self.localization_ok:
            return False, "localization_not_ready"
        if self.require_real_ready and not self.real_ready():
            return False, "real_readiness_not_ready"
        if (
            self.navigation_backend == "nav2"
            and (not self.dry_run or self.dry_run_require_action_server)
            and not self.navigate_client.wait_for_server(timeout_sec=0.1)
        ):
            return False, "navigate_action_not_ready"
        return True, "ok"

    def wait_for_preflight(self) -> None:
        deadline = time.monotonic() + self.preflight_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            ready, reason = self.preflight_ready()
            if ready:
                self.publish_status("preflight_ready", reason)
                return
            self.publish_status("preflight_wait", reason, ready=False)
            rclpy.spin_once(self, timeout_sec=0.1)
        ready, reason = self.preflight_ready()
        raise RuntimeError(f"preflight timeout: {reason}")

    def call_set_mode(self) -> None:
        if not self.mission_mode:
            return
        if not self.set_mode_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("set_mode service unavailable")
        request = SetMode.Request()
        request.mode = self.mission_mode
        future = self.set_mode_client.call_async(request)
        if not self.wait_for_future(future, 5.0):
            raise RuntimeError("set_mode call timed out")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f"set_mode failed: {getattr(response, 'message', 'unknown')}")

    def build_goal(self, waypoint: WaypointSpec) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = self.goal_frame
        if self.navigation_backend != "nav2":
            goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = waypoint.x
        goal.pose.position.y = waypoint.y
        _, _, goal.pose.orientation.z, goal.pose.orientation.w = yaw_to_quaternion(waypoint.yaw)
        return goal

    def feedback_callback(self, feedback_msg: Any) -> None:
        feedback = feedback_msg.feedback
        distance_remaining = getattr(feedback, "distance_remaining", None)
        if distance_remaining is not None:
            self.last_feedback_distance = float(distance_remaining)

    def execute_waypoint(self, waypoint: WaypointSpec, index: int, total: int) -> dict[str, Any]:
        start_time = time.monotonic()
        localization_drops_before = self.localization_drop_events
        real_not_ready_before = self.real_not_ready_events
        valid, reason, details = self.validate_waypoint_against_map(waypoint)
        if not valid:
            return self.finish_waypoint_result(
                waypoint,
                f"precheck_failed:{reason}",
                False,
                start_time,
                localization_drops_before,
                real_not_ready_before,
                route_validation=details,
            )

        target = self.build_goal(waypoint)
        self.goal_pub.publish(target)
        self.progress_pub.publish(Float32(data=float(index - 1) / float(total)))
        self.publish_status(
            "waypoint_dispatch",
            "sending_goal",
            waypoint=waypoint.waypoint_id,
            index=index,
            total=total,
        )

        if self.navigation_backend == "pose_topic_3d":
            return self.execute_pose_topic_waypoint(
                waypoint,
                target,
                start_time,
                localization_drops_before,
                real_not_ready_before,
            )

        goal = NavigateToPose.Goal()
        goal.pose = target
        self.last_feedback_distance = None

        goal_future = self.navigate_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        if not self.wait_for_future(goal_future, self.goal_response_timeout_sec):
            return self.finish_waypoint_result(
                waypoint, "goal_response_timeout", False, start_time, localization_drops_before, real_not_ready_before
            )
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return self.finish_waypoint_result(
                waypoint, "goal_rejected", False, start_time, localization_drops_before, real_not_ready_before
            )

        result_future = goal_handle.get_result_async()
        if not self.wait_for_future(result_future, self.goal_result_timeout_sec):
            goal_handle.cancel_goal_async()
            # ── Recovery FSM for Nav2 path ──────────────────────────
            recovery_info: dict[str, Any] = {"triggered": False, "recovered": False, "attempts": 0}
            if self.recovery_enabled:
                recovery_info = self._run_recovery_nav2(waypoint, target, start_time)
            result = self.finish_waypoint_result(
                waypoint,
                "goal_result_timeout" if not recovery_info.get("recovered") else "recovered_continue",
                recovery_info.get("recovered", False),
                start_time,
                localization_drops_before,
                real_not_ready_before,
                recovery=recovery_info,
            )
            return result

        result = result_future.result()
        status_code = result.status if result is not None else GoalStatus.STATUS_UNKNOWN
        if waypoint.dwell_sec > 0.0:
            self.spin_for(waypoint.dwell_sec)
        self.spin_for(self.settle_time_sec)

        recovery_info = {}
        if status_code == GoalStatus.STATUS_ABORTED and self.recovery_enabled:
            recovery_info = self._run_recovery_nav2(waypoint, target, start_time)
            if recovery_info.get("recovered"):
                status_code = GoalStatus.STATUS_SUCCEEDED

        if status_code == GoalStatus.STATUS_SUCCEEDED:
            state = "succeeded"
        elif status_code == GoalStatus.STATUS_CANCELED:
            state = "canceled"
        elif status_code == GoalStatus.STATUS_ABORTED:
            state = "aborted"
        else:
            state = f"status_{status_code}"
        return self.finish_waypoint_result(
            waypoint,
            state,
            status_code == GoalStatus.STATUS_SUCCEEDED,
            start_time,
            localization_drops_before,
            real_not_ready_before,
            recovery=recovery_info if recovery_info else None,
        )

    def _on_nav3_status(self, msg: String) -> None:
        self.nav3_status_raw = msg.data

    def _nav3_state(self) -> str:
        return parse_status_string(self.nav3_status_raw).get("state", "")

    def _publish_zero_recovery(self) -> None:
        zero = Twist()
        self.recovery_cmd_pub.publish(zero)
        self.recovery_direct_cmd_pub.publish(zero)

    def _publish_recovery_twist(self, vx: float, vy: float, wz: float) -> None:
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        self.recovery_cmd_pub.publish(cmd)
        self.recovery_direct_cmd_pub.publish(cmd)

    def _check_pose_reached(self, waypoint: "WaypointSpec") -> tuple[bool, float | None]:
        pose = self.current_pose_dict()
        if pose is None:
            return False, None
        distance = math.hypot(float(pose["x"]) - waypoint.x, float(pose["y"]) - waypoint.y)
        yaw_error = abs(normalize_angle(float(pose["yaw"]) - waypoint.yaw))
        self.last_feedback_distance = distance
        reached = distance <= self.position_pass_threshold_m and yaw_error <= self.yaw_pass_threshold_rad
        return reached, distance

    def _drive_to_pose_topic_goal(self, waypoint: "WaypointSpec", deadline: float) -> tuple[bool, str]:
        """Track an already-published pose goal until reached, deadline, or planner block.

        Returns (reached, reason). reason is one of: 'reached', 'deadline',
        'planner_blocked', 'cancelled'.
        """
        while rclpy.ok() and time.monotonic() < deadline:
            reached, _ = self._check_pose_reached(waypoint)
            if reached:
                return True, "reached"
            if self.recovery_enabled and self._nav3_state() == "blocked":
                return False, "planner_blocked"
            rclpy.spin_once(self, timeout_sec=0.1)
        if not rclpy.ok():
            return False, "cancelled"
        return False, "deadline"

    def _run_recovery_fsm(self, waypoint: "WaypointSpec", target: PoseStamped, deadline: float) -> dict[str, Any]:
        """Try to unblock by emitting Twist hints; resume goal tracking after each step.

        The C++ obstacle_aware_local_planner_3d adopts our recovery_cmd_topic input only after
        its own hard-clearance veto, so this never bypasses the safety pipeline. Each step
        re-publishes the goal so that as soon as the planner becomes unblocked it resumes.
        """
        info: dict[str, Any] = {
            "triggered": True,
            "sequence": [],
            "recovered": False,
            "duration_sec": 0.0,
        }
        if not self.recovery_enabled:
            info["triggered"] = False
            return info

        budget_deadline = min(deadline, time.monotonic() + self.recovery_total_budget_sec)
        period = 1.0 / max(1.0, self.recovery_publish_hz)
        t_start = time.monotonic()

        def step(name: str, duration: float, twist_fn) -> str:
            """Execute one recovery step. Returns 'reached' | 'recovered' | 'expired'."""
            info["sequence"].append(name)
            self.publish_status(
                "recovery_active", f"step={name}", ready=True,
                waypoint=waypoint.waypoint_id,
            )
            step_deadline = min(budget_deadline, time.monotonic() + duration)
            tick = 0
            while rclpy.ok() and time.monotonic() < step_deadline:
                vx, vy, wz = twist_fn(tick)
                self._publish_recovery_twist(vx, vy, wz)
                rclpy.spin_once(self, timeout_sec=period)
                tick += 1
                reached, _ = self._check_pose_reached(waypoint)
                if reached:
                    self._publish_zero_recovery()
                    return "reached"
                # As soon as planner is no longer blocked, hand control back.
                if self._nav3_state() not in ("blocked", ""):
                    self._publish_zero_recovery()
                    self.pose_goal_pub.publish(target)
                    return "recovered"
            self._publish_zero_recovery()
            return "expired"

        # Step 1: spin in place, alternating direction.
        outcome = step(
            "spin_probe", self.recovery_spin_sec,
            lambda tick: (0.0, 0.0,
                          self.recovery_spin_rate if (tick // max(1, int(self.recovery_publish_hz))) % 2 == 0
                          else -self.recovery_spin_rate),
        )
        if outcome == "reached":
            info["recovered"] = True
            info["duration_sec"] = time.monotonic() - t_start
            return info
        if outcome == "recovered":
            ok, why = self._drive_to_pose_topic_goal(waypoint, deadline)
            info["recovered"] = ok
            info["duration_sec"] = time.monotonic() - t_start
            info["resume_reason"] = why
            if ok:
                return info
            # else fall through to next step

        if time.monotonic() >= budget_deadline:
            info["duration_sec"] = time.monotonic() - t_start
            return info

        # Step 2: short backup.
        outcome = step(
            "backup_probe", self.recovery_backup_sec,
            lambda _tick: (-self.recovery_backup_speed, 0.0, 0.0),
        )
        if outcome == "reached":
            info["recovered"] = True
            info["duration_sec"] = time.monotonic() - t_start
            return info
        if outcome == "recovered":
            ok, why = self._drive_to_pose_topic_goal(waypoint, deadline)
            info["recovered"] = ok
            info["duration_sec"] = time.monotonic() - t_start
            info["resume_reason"] = why
            if ok:
                return info

        if time.monotonic() >= budget_deadline:
            info["duration_sec"] = time.monotonic() - t_start
            return info

        # Step 3: lateral probe (right then left).
        half = max(0.1, self.recovery_lateral_sec / 2.0)
        outcome = step(
            "lateral_right",
            half,
            lambda _tick: (0.0, -self.recovery_lateral_speed, 0.0),
        )
        if outcome == "recovered":
            ok, why = self._drive_to_pose_topic_goal(waypoint, deadline)
            info["recovered"] = ok
            info["duration_sec"] = time.monotonic() - t_start
            info["resume_reason"] = why
            if ok:
                return info
        if outcome == "reached":
            info["recovered"] = True
            info["duration_sec"] = time.monotonic() - t_start
            return info

        outcome = step(
            "lateral_left",
            half,
            lambda _tick: (0.0, self.recovery_lateral_speed, 0.0),
        )
        if outcome == "reached":
            info["recovered"] = True
        elif outcome == "recovered":
            ok, why = self._drive_to_pose_topic_goal(waypoint, deadline)
            info["recovered"] = ok
            info["resume_reason"] = why
        info["duration_sec"] = time.monotonic() - t_start
        return info

    def _run_recovery_nav2(
        self, waypoint: "WaypointSpec", target: PoseStamped, _wp_start_time: float
    ) -> dict[str, Any]:
        """Nav2-backend recovery: emit Twist hints, then re-send NavigateToPose goal.

        Unlike the pose_topic_3d path which checks _nav3_state() for "blocked", the Nav2
        path simply cancels the timed-out goal, executes the recovery sequence, and
        retries the waypoint up to max_recovery_attempts times.
        """
        info: dict[str, Any] = {
            "triggered": True,
            "sequence": [],
            "recovered": False,
            "duration_sec": 0.0,
            "attempts": 0,
        }
        if not self.recovery_enabled:
            info["triggered"] = False
            return info

        t_start = time.monotonic()
        period = 1.0 / max(1.0, self.recovery_publish_hz)

        def run_step(name: str, duration: float, twist_fn) -> bool:
            """Returns True if the goal was reached during or after this step."""
            info["sequence"].append(name)
            self.publish_status(
                "recovery_active", f"step={name};backend=nav2", ready=True,
                waypoint=waypoint.waypoint_id,
            )
            step_deadline = time.monotonic() + duration
            tick = 0
            while rclpy.ok() and time.monotonic() < step_deadline:
                vx, vy, wz = twist_fn(tick)
                self._publish_recovery_twist(vx, vy, wz)
                rclpy.spin_once(self, timeout_sec=period)
                tick += 1
                reached, _ = self._check_pose_reached(waypoint)
                if reached:
                    self._publish_zero_recovery()
                    return True
            self._publish_zero_recovery()
            return False

        for attempt in range(self.max_recovery_attempts):
            info["attempts"] = attempt + 1
            reached = run_step(
                "spin_probe", self.recovery_spin_sec,
                lambda tick: (0.0, 0.0,
                              self.recovery_spin_rate if (tick // max(1, int(self.recovery_publish_hz))) % 2 == 0
                              else -self.recovery_spin_rate),
            )
            if reached:
                info["recovered"] = True
                info["duration_sec"] = time.monotonic() - t_start
                return info

            # Re-send Nav2 goal and wait (shorter timeout for recovery retry).
            try:
                goal = NavigateToPose.Goal()
                goal.pose = target
                goal_future = self.navigate_client.send_goal_async(goal)
                if not self.wait_for_future(goal_future, 10.0):
                    continue
                goal_handle = goal_future.result()
                if goal_handle is None or not goal_handle.accepted:
                    continue
                result_future = goal_handle.get_result_async()
                retry_timeout = min(
                    self.goal_result_timeout_sec / max(1, attempt + 1),
                    self.recovery_total_budget_sec,
                )
                if not self.wait_for_future(result_future, retry_timeout):
                    goal_handle.cancel_goal_async()
                    continue
                result = result_future.result()
                if result is not None and result.status == GoalStatus.STATUS_SUCCEEDED:
                    info["recovered"] = True
                    info["duration_sec"] = time.monotonic() - t_start
                    return info
            except Exception:
                continue

        info["duration_sec"] = time.monotonic() - t_start
        return info

    def execute_pose_topic_waypoint(
        self,
        waypoint: WaypointSpec,
        target: PoseStamped,
        start_time: float,
        localization_drops_before: int,
        real_not_ready_before: int,
    ) -> dict[str, Any]:
        self.pose_goal_pub.publish(target)
        deadline = time.monotonic() + self.goal_result_timeout_sec
        recovery_info: dict[str, Any] | None = None
        recovery_attempts = 0
        max_recovery_attempts = 1

        while True:
            reached, reason = self._drive_to_pose_topic_goal(waypoint, deadline)
            if reached:
                if waypoint.dwell_sec > 0.0:
                    self.spin_for(waypoint.dwell_sec)
                self.spin_for(self.settle_time_sec)
                state = "succeeded" if recovery_info is None else "recovered_continue"
                result = self.finish_waypoint_result(
                    waypoint,
                    state,
                    True,
                    start_time,
                    localization_drops_before,
                    real_not_ready_before,
                )
                if recovery_info is not None:
                    result["recovery"] = recovery_info
                return result

            if (
                self.recovery_enabled
                and reason in ("planner_blocked", "deadline")
                and recovery_attempts < max_recovery_attempts
                and time.monotonic() < deadline
            ):
                recovery_attempts += 1
                recovery_info = self._run_recovery_fsm(waypoint, target, deadline)
                # If recovery brought us into tracking, loop back and continue waiting on the goal.
                if recovery_info.get("recovered", False):
                    # Re-evaluate goal (could already have been reached during recovery loop).
                    reached2, _ = self._check_pose_reached(waypoint)
                    if reached2:
                        if waypoint.dwell_sec > 0.0:
                            self.spin_for(waypoint.dwell_sec)
                        self.spin_for(self.settle_time_sec)
                        result = self.finish_waypoint_result(
                            waypoint, "recovered_continue", True, start_time,
                            localization_drops_before, real_not_ready_before,
                        )
                        result["recovery"] = recovery_info
                        return result
                    # planner is moving again — keep tracking until deadline
                    self.pose_goal_pub.publish(target)
                    continue
                # Recovery failed: fall through to timeout failure with recovery info attached.
            break

        result = self.finish_waypoint_result(
            waypoint,
            "goal_result_timeout",
            False,
            start_time,
            localization_drops_before,
            real_not_ready_before,
        )
        if recovery_info is not None:
            result["recovery"] = recovery_info
        return result

    def finish_waypoint_result(
        self,
        waypoint: WaypointSpec,
        state: str,
        nav_success: bool,
        start_time: float,
        localization_drops_before: int,
        real_not_ready_before: int,
        route_validation: dict[str, Any] | None = None,
        recovery: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        final_pose = self.current_pose_dict()
        position_error_m = None
        yaw_error_rad = None
        validation = "fail"
        if final_pose is not None:
            dx = final_pose["x"] - waypoint.x
            dy = final_pose["y"] - waypoint.y
            position_error_m = math.hypot(dx, dy)
            yaw_error_rad = abs(normalize_angle(final_pose["yaw"] - waypoint.yaw))
            if nav_success and position_error_m <= self.position_pass_threshold_m and yaw_error_rad <= self.yaw_pass_threshold_rad:
                validation = "pass"
            elif nav_success and position_error_m <= self.position_warn_threshold_m and yaw_error_rad <= self.yaw_warn_threshold_rad:
                validation = "warn"
            else:
                validation = "fail"
        duration_sec = time.monotonic() - start_time
        result = {
            "waypoint_id": waypoint.waypoint_id,
            "target_x": waypoint.x,
            "target_y": waypoint.y,
            "target_yaw": waypoint.yaw,
            "note": waypoint.note,
            "state": state,
            "nav_success": nav_success,
            "validation": validation,
            "duration_sec": duration_sec,
            "last_feedback_distance": self.last_feedback_distance,
            "position_error_m": position_error_m,
            "yaw_error_rad": yaw_error_rad,
            "final_pose": final_pose,
            "route_validation": route_validation,
            "localization_drop_events": self.localization_drop_events - localization_drops_before,
            "real_not_ready_events": self.real_not_ready_events - real_not_ready_before,
            "localization_ok_end": self.localization_ok,
            "localization_status": self.localization_status_raw,
            "real_report": self.real_report_raw,
            "nav_status": self.nav_status_raw,
        }
        if recovery:
            result["recovery"] = recovery
        self.report_entries.append(result)
        self.progress_pub.publish(
            Float32(data=float(len(self.report_entries)) / max(1.0, float(self.total_waypoints)))
        )
        self.publish_status(
            "waypoint_complete",
            state,
            ready=nav_success,
            waypoint=waypoint.waypoint_id,
            validation=validation,
        )
        return result

    def maybe_save_map(self, save_requested: bool) -> None:
        if not save_requested:
            return
        if not self.manage_map_client.wait_for_service(timeout_sec=2.0):
            self.saved_map_message = "manage_map service unavailable"
            return
        request = ManageMap.Request()
        request.command = "save"
        request.map_id = f"{self.saved_map_prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        future = self.manage_map_client.call_async(request)
        if not self.wait_for_future(future, 10.0):
            self.saved_map_message = "map save timed out"
            return
        response = future.result()
        if response is None:
            self.saved_map_message = "map save returned no response"
            return
        self.saved_map_message = response.message
        if response.success:
            self.saved_map_id = request.map_id

    def build_report_summary(self, waypoints: list[WaypointSpec]) -> dict[str, Any]:
        position_errors = [
            float(entry["position_error_m"])
            for entry in self.report_entries
            if entry.get("position_error_m") is not None
        ]
        yaw_errors = [
            float(entry["yaw_error_rad"])
            for entry in self.report_entries
            if entry.get("yaw_error_rad") is not None
        ]
        validation_counts: dict[str, int] = {}
        state_counts: dict[str, int] = {}
        for entry in self.report_entries:
            validation = str(entry.get("validation", "unknown"))
            state = str(entry.get("state", "unknown"))
            validation_counts[validation] = validation_counts.get(validation, 0) + 1
            state_counts[state] = state_counts.get(state, 0) + 1
        planned = len(waypoints)
        executed = len(self.report_entries)
        succeeded = state_counts.get("succeeded", 0) + state_counts.get("recovered_continue", 0)
        return {
            "mission": self.mission_name,
            "dry_run": self.dry_run,
            "outcome": self.final_outcome,
            "reason": self.final_reason,
            "waypoints_planned": planned,
            "waypoints_executed": executed,
            "success_rate": float(succeeded / planned) if planned else 0.0,
            "state_counts": state_counts,
            "validation_counts": validation_counts,
            "avg_position_error_m": sum(position_errors) / len(position_errors) if position_errors else None,
            "max_position_error_m": max(position_errors) if position_errors else None,
            "avg_yaw_error_rad": sum(yaw_errors) / len(yaw_errors) if yaw_errors else None,
            "max_yaw_error_rad": max(yaw_errors) if yaw_errors else None,
            "localization_drop_events": self.localization_drop_events,
            "real_not_ready_events": self.real_not_ready_events,
            "active_map": self.active_map or None,
            "saved_map_id": self.saved_map_id,
            "saved_map_message": self.saved_map_message,
        }

    def write_structured_reports(self, report_path: Path, waypoints: list[WaypointSpec]) -> tuple[Path, Path]:
        summary = self.build_report_summary(waypoints)
        json_path = report_path.with_suffix(".json")
        csv_path = report_path.with_suffix(".csv")
        payload = {
            "summary": summary,
            "mission_contract": {
                "goal_frame": self.goal_frame,
                "validate_waypoints_against_map": self.validate_waypoints_against_map,
                "allow_unknown_cells": self.allow_unknown_cells,
                "occupied_threshold": self.occupied_threshold,
                "min_clearance_cells": self.min_clearance_cells,
                "stop_on_failure": self.stop_on_failure,
                "position_pass_threshold_m": self.position_pass_threshold_m,
                "position_warn_threshold_m": self.position_warn_threshold_m,
                "yaw_pass_threshold_rad": self.yaw_pass_threshold_rad,
                "yaw_warn_threshold_rad": self.yaw_warn_threshold_rad,
            },
            "preflight_snapshot": {
                "map_metadata": self.map_metadata(),
                "current_pose": self.current_pose_dict(),
                "localization_ok": self.localization_ok,
                "localization_status": self.localization_status_raw,
                "real_report": self.real_report_raw,
                "map_manager_status": self.map_manager_status_raw,
            },
            "route_validation": self.route_validation_entries,
            "waypoints": self.report_entries,
        }
        json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

        fieldnames = [
            "waypoint_id",
            "state",
            "nav_success",
            "validation",
            "duration_sec",
            "target_x",
            "target_y",
            "target_yaw",
            "position_error_m",
            "yaw_error_rad",
            "localization_drop_events",
            "real_not_ready_events",
            "localization_ok_end",
            "last_feedback_distance",
            "note",
        ]
        with csv_path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for entry in self.report_entries:
                writer.writerow({key: entry.get(key) for key in fieldnames})
        return json_path, csv_path

    def write_report(self, waypoints: list[WaypointSpec]) -> Path:
        self.reports_root.mkdir(parents=True, exist_ok=True)
        report_path = self.reports_root / f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{self.mission_name}.md"
        json_path = report_path.with_suffix(".json")
        csv_path = report_path.with_suffix(".csv")
        summary = self.build_report_summary(waypoints)
        lines: list[str] = [
            "# Auto Scan Mission Report",
            "",
            "## Summary",
            "",
            f"- Mission: `{self.mission_name}`",
            f"- Dry run: `{self.dry_run}`",
            f"- Outcome: `{self.final_outcome}`",
            f"- Reason: `{self.final_reason}`",
            f"- Waypoints planned: `{len(waypoints)}`",
            f"- Waypoints executed: `{len(self.report_entries)}`",
            f"- Localization drop events: `{self.localization_drop_events}`",
            f"- Real readiness drop events: `{self.real_not_ready_events}`",
            f"- Active map at finish: `{self.active_map or 'none'}`",
            f"- Saved map id: `{self.saved_map_id or 'none'}`",
            f"- Saved map result: `{self.saved_map_message or 'not_requested'}`",
            f"- Success rate: `{summary['success_rate']:.3f}`",
            f"- Average XY error m: `{summary['avg_position_error_m']}`",
            f"- Max XY error m: `{summary['max_position_error_m']}`",
            f"- Average yaw error rad: `{summary['avg_yaw_error_rad']}`",
            f"- Max yaw error rad: `{summary['max_yaw_error_rad']}`",
            f"- JSON report: `{json_path}`",
            f"- CSV report: `{csv_path}`",
            "",
            "## Preflight Snapshot",
            "",
            f"- Map metadata: `{self.map_metadata()}`",
            f"- Current pose: `{self.current_pose_dict()}`",
            f"- Localization ok: `{self.localization_ok}`",
            f"- Localization status: `{self.localization_status_raw}`",
            f"- Real report: `{self.real_report_raw}`",
            f"- Map manager status: `{self.map_manager_status_raw}`",
            "",
            "## Mission Contract",
            "",
            f"- Goal frame: `{self.goal_frame}`",
            f"- Validate waypoints against map: `{self.validate_waypoints_against_map}`",
            f"- Allow unknown cells: `{self.allow_unknown_cells}`",
            f"- Occupied threshold: `{self.occupied_threshold}`",
            f"- Min clearance cells: `{self.min_clearance_cells}`",
            f"- Stop on failure: `{self.stop_on_failure}`",
            f"- Position pass/warn threshold m: `{self.position_pass_threshold_m}` / `{self.position_warn_threshold_m}`",
            f"- Yaw pass/warn threshold rad: `{self.yaw_pass_threshold_rad}` / `{self.yaw_warn_threshold_rad}`",
            "",
            "## Route Validation",
            "",
        ]
        if self.route_validation_entries:
            for entry in self.route_validation_entries:
                lines.extend(
                    [
                        f"- `{entry['waypoint_id']}`: valid=`{entry['valid']}`, reason=`{entry['reason']}`, details=`{entry['details']}`",
                    ]
                )
        else:
            lines.append("- No route validation entries recorded.")
        lines.extend(["", "## Waypoints", ""])
        for entry in self.report_entries:
            lines.extend(
                [
                    f"### {entry['waypoint_id']}",
                    "",
                    f"- State: `{entry['state']}`",
                    f"- Validation: `{entry['validation']}`",
                    f"- Duration sec: `{entry['duration_sec']:.2f}`",
                    f"- Target: `({entry['target_x']:.3f}, {entry['target_y']:.3f}, {entry['target_yaw']:.3f})`",
                    f"- Final pose: `{entry['final_pose']}`",
                    f"- Route validation: `{entry['route_validation']}`",
                    f"- Position error m: `{entry['position_error_m']}`",
                    f"- Yaw error rad: `{entry['yaw_error_rad']}`",
                    f"- Feedback distance remaining: `{entry['last_feedback_distance']}`",
                    f"- Localization drop events: `{entry['localization_drop_events']}`",
                    f"- Real readiness drop events: `{entry['real_not_ready_events']}`",
                    f"- Localization status end: `{entry['localization_status']}`",
                    f"- Real report end: `{entry['real_report']}`",
                    f"- Nav status end: `{entry['nav_status']}`",
                    "",
                ]
            )
        if len(self.report_entries) < len(waypoints):
            remaining = [waypoint.waypoint_id for waypoint in waypoints[len(self.report_entries):]]
            lines.extend(
                [
                    "## Remaining Waypoints",
                    "",
                    f"- Not executed: `{remaining}`",
                    "",
                ]
            )
        report_path.write_text("\n".join(lines), encoding="utf-8")
        self.write_structured_reports(report_path, waypoints)
        return report_path

    def run(self) -> Path:
        waypoints: list[WaypointSpec] = []
        self._mission_running = True
        try:
            waypoints = self.load_waypoints()
            self.total_waypoints = len(waypoints)
            self.publish_status(
                "mission_start",
                "loading_waypoints",
                total=len(waypoints),
                dry_run=self.dry_run,
            )
            if self.dry_run:
                self.publish_status("dry_run", "skipping_mode_switch")
            else:
                self.call_set_mode()
            self.wait_for_preflight()

            route_valid = self.validate_route_against_map(waypoints)
            if not route_valid:
                self.final_outcome = "failed"
                self.final_reason = "route_validation_failed"
                self.publish_status("mission_blocked", self.final_reason, ready=False)
            elif self.dry_run:
                self.progress_pub.publish(Float32(data=1.0))
                self.final_outcome = "dry_run_succeeded"
                self.final_reason = "route_validation_passed"
                self.publish_status("dry_run_complete", self.final_reason)
            else:
                for index, waypoint in enumerate(waypoints, start=1):
                    result = self.execute_waypoint(waypoint, index, len(waypoints))
                    if result["validation"] == "fail" and self.stop_on_failure:
                        self.final_outcome = "failed"
                        self.final_reason = f"waypoint_failed:{waypoint.waypoint_id}"
                        break
                    if result["state"] not in {"succeeded", "recovered_continue"} and self.stop_on_failure:
                        self.final_outcome = "failed"
                        self.final_reason = f"waypoint_state:{waypoint.waypoint_id}:{result['state']}"
                        break
                else:
                    states = {entry["state"] for entry in self.report_entries}
                    validations = {entry["validation"] for entry in self.report_entries}
                    if states.issubset({"succeeded", "recovered_continue"}) and validations == {"pass"}:
                        self.final_outcome = "succeeded"
                        self.final_reason = "all_waypoints_completed"
                    elif "fail" in validations or any(state not in {"succeeded", "recovered_continue"} for state in states):
                        self.final_outcome = "completed_with_findings"
                        self.final_reason = "waypoint_validation_findings"
                    else:
                        self.final_outcome = "completed_with_warnings"
                        self.final_reason = "waypoint_warning_findings"

            if not self.dry_run:
                if self.final_outcome == "failed":
                    self.maybe_save_map(self.save_map_on_failure)
                else:
                    self.maybe_save_map(self.save_map_on_finish)
        except Exception as exc:
            self.final_outcome = "failed"
            self.final_reason = f"exception:{exc}"
            self.get_logger().error(str(exc))
        finally:
            self._mission_running = False

        report_path = self.write_report(waypoints)
        self.report_pub.publish(String(data=str(report_path)))
        ready = self.final_outcome in {"succeeded", "dry_run_succeeded"}
        self.publish_status("mission_complete", self.final_reason, ready=ready)
        return report_path

    # ------------------------------------------------------------------
    # ROS 2 Action Server (a2_interfaces.action.RunMission)
    # ------------------------------------------------------------------
    def install_action_server(self) -> bool:
        """Install the RunMission action server. Idempotent. Returns False if the
        action interface is unavailable (e.g. partial install)."""
        if not _HAS_RUN_MISSION_ACTION:
            self.get_logger().warning(
                "RunMission action interface unavailable; action server not installed"
            )
            return False
        if getattr(self, "_run_mission_action_server", None) is not None:
            return True
        self._action_feedback_cb = None
        self._cancel_requested = False
        self._run_mission_action_server = ActionServer(
            self,
            RunMissionAction,
            "run_mission",
            execute_callback=self._run_mission_execute,
            goal_callback=self._run_mission_goal_cb,
            cancel_callback=self._run_mission_cancel_cb,
        )
        # Snapshot params so per-goal overrides can be reset between goals.
        self._param_defaults = {
            "mission_name": self.mission_name,
            "waypoints_file": self.waypoints_file,
            "require_real_ready": self.require_real_ready,
            "stop_on_failure": self.stop_on_failure,
        }
        self.get_logger().info("RunMission action server installed at /run_mission")
        return True

    def _run_mission_goal_cb(self, _goal_request) -> Any:
        # One mission at a time; reject if another is running.
        if self._mission_running:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _run_mission_cancel_cb(self, _goal_handle) -> Any:
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    def is_cancel_requested(self) -> bool:
        return bool(getattr(self, "_cancel_requested", False))

    def _publish_action_feedback(self, goal_handle, state: str, reason: str, last_kv: str) -> None:
        try:
            fb = RunMissionAction.Feedback()
            fb.state = state
            fb.reason = reason
            fb.current_waypoint_index = int(len(self.report_entries))
            fb.total_waypoints = int(self.total_waypoints)
            denom = max(1, self.total_waypoints)
            fb.progress = float(len(self.report_entries)) / float(denom)
            fb.last_status_kv = last_kv
            goal_handle.publish_feedback(fb)
        except Exception as exc:  # pragma: no cover - best-effort
            self.get_logger().warning(f"publish_feedback failed: {exc}")

    def _run_mission_execute(self, goal_handle):
        request = goal_handle.request
        # Apply per-goal overrides.
        if request.mission_name:
            self.mission_name = request.mission_name
        if request.waypoints_file:
            self.waypoints_file = os.path.expandvars(os.path.expanduser(request.waypoints_file))
        self.require_real_ready = bool(request.require_real_ready)
        self.stop_on_failure = bool(request.stop_on_failure)

        # Reset per-goal aggregation state.
        self.report_entries = []
        self.route_validation_entries = []
        self.saved_map_id = None
        self.saved_map_message = None
        self.final_outcome = "not_started"
        self.final_reason = "not_started"
        self.last_feedback_distance = None
        self.total_waypoints = 0
        self._cancel_requested = False
        self._action_feedback_cb = lambda state, reason, kv: self._publish_action_feedback(
            goal_handle, state, reason, kv
        )
        report_path: Path | None = None
        try:
            report_path = self.run()
        except Exception as exc:  # pragma: no cover - run() handles most cases internally
            self.get_logger().error(f"RunMission execute failed: {exc}")
            self.final_outcome = "failed"
            self.final_reason = f"exception:{exc}"
        finally:
            self._action_feedback_cb = None
            # Restore defaults so a subsequent goal (or shutdown) sees pristine values.
            d = getattr(self, "_param_defaults", {})
            self.mission_name = d.get("mission_name", self.mission_name)
            self.waypoints_file = d.get("waypoints_file", self.waypoints_file)
            self.require_real_ready = d.get("require_real_ready", self.require_real_ready)
            self.stop_on_failure = d.get("stop_on_failure", self.stop_on_failure)

        result = RunMissionAction.Result()
        # Determine action terminal state.
        success = self.final_outcome in {"succeeded", "dry_run_succeeded"}
        if self._cancel_requested:
            goal_handle.canceled()
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        result.success = bool(success)
        result.reason = str(self.final_reason)
        result.report_json_path = ""
        result.report_csv_path = ""
        if report_path is not None:
            result.report_json_path = str(report_path.with_suffix(".json"))
            result.report_csv_path = str(report_path.with_suffix(".csv"))
        result.succeeded_waypoints = int(
            sum(
                1 for e in self.report_entries
                if e.get("state") in ("succeeded", "recovered_continue")
            )
        )
        result.total_waypoints = int(self.total_waypoints)
        return result


def main() -> None:
    rclpy.init()
    node = AutoScanMission()
    enable_action_server = bool(
        node.declare_parameter("enable_action_server", True).value
    )
    try:
        if enable_action_server and node.install_action_server():
            node.get_logger().info(
                "auto_scan_mission running as action server (no immediate run)"
            )
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:  # pragma: no cover
                pass
        else:
            report_path = node.run()
            node.get_logger().info(f"Mission report written to: {report_path}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
