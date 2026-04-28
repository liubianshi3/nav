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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover - depends on runtime environment
    NavigateToPose = None


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
        self.pose_topic = self.declare_parameter("pose_topic", "/amcl_pose").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
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
            self.declare_parameter("validate_waypoints_against_map", True).value
        )
        self.allow_unknown_cells = bool(self.declare_parameter("allow_unknown_cells", False).value)
        self.occupied_threshold = int(self.declare_parameter("occupied_threshold", 65).value)
        self.min_clearance_cells = int(self.declare_parameter("min_clearance_cells", 0).value)
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

        self.map_received = False
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

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, transient_qos)
        self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.on_pose, transient_qos)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
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
            if NavigateToPose is not None
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
        if self.latest_pose is None:
            return None
        pose = self.latest_pose.pose.pose
        covariance = self.latest_pose.pose.covariance
        return {
            "frame_id": self.latest_pose.header.frame_id,
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
        if self.navigate_client is None or NavigateToPose is None:
            return False, "navigate_action_type_missing"
        if not self.map_received or self.latest_map is None:
            return False, "map_missing"
        if self.require_map_frame and self.latest_map.header.frame_id and self.latest_map.header.frame_id != self.goal_frame:
            return False, "map_frame_mismatch"
        if self.latest_pose is None:
            return False, "pose_missing"
        if self.require_localization_ready and not self.localization_ok:
            return False, "localization_not_ready"
        if self.require_real_ready and not self.real_ready():
            return False, "real_readiness_not_ready"
        if (not self.dry_run or self.dry_run_require_action_server) and not self.navigate_client.wait_for_server(timeout_sec=0.1):
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
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.goal_frame
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
            return self.finish_waypoint_result(
                waypoint, "goal_result_timeout", False, start_time, localization_drops_before, real_not_ready_before
            )

        result = result_future.result()
        status_code = result.status if result is not None else GoalStatus.STATUS_UNKNOWN
        if waypoint.dwell_sec > 0.0:
            self.spin_for(waypoint.dwell_sec)
        self.spin_for(self.settle_time_sec)

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
        )

    def finish_waypoint_result(
        self,
        waypoint: WaypointSpec,
        state: str,
        nav_success: bool,
        start_time: float,
        localization_drops_before: int,
        real_not_ready_before: int,
        route_validation: dict[str, Any] | None = None,
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
        succeeded = state_counts.get("succeeded", 0)
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
                    if result["state"] not in {"succeeded"} and self.stop_on_failure:
                        self.final_outcome = "failed"
                        self.final_reason = f"waypoint_state:{waypoint.waypoint_id}:{result['state']}"
                        break
                else:
                    states = {entry["state"] for entry in self.report_entries}
                    validations = {entry["validation"] for entry in self.report_entries}
                    if states == {"succeeded"} and validations == {"pass"}:
                        self.final_outcome = "succeeded"
                        self.final_reason = "all_waypoints_completed"
                    elif "fail" in validations or any(state != "succeeded" for state in states):
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


def main() -> None:
    rclpy.init()
    node = AutoScanMission()
    try:
        report_path = node.run()
        node.get_logger().info(f"Mission report written to: {report_path}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
