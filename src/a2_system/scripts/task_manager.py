#!/usr/bin/env python3

from __future__ import annotations

import math
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from a2_interfaces.srv import ManageMap, NavCommand, SetMode
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover - depends on runtime environment
    NavigateToPose = None

try:
    from a2_interfaces.action import RunMission as RunMissionAction
    _HAS_RUN_MISSION_ACTION = True
except ImportError:  # pragma: no cover - depends on runtime environment
    RunMissionAction = None  # type: ignore[assignment]
    _HAS_RUN_MISSION_ACTION = False


ROUTE_ID_RE = re.compile(r"^[A-Za-z0-9_.-]+$")


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(orientation: Any) -> float:
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def parse_status_string(payload: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for item in (payload or "").split(";"):
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        fields[key] = value
    return fields


@dataclass(frozen=True)
class RouteWaypoint:
    waypoint_id: str
    x: float
    y: float
    yaw: float
    dwell_sec: float
    note: str


def validate_route_id(route_id: str) -> str:
    normalized = (route_id or "").strip()
    if not normalized:
        raise RuntimeError("route_id is empty")
    if not ROUTE_ID_RE.fullmatch(normalized):
        raise RuntimeError(f"invalid route_id: {route_id}")
    return normalized


def normalize_route_payload(payload: Any, *, default_mission_name: str | None = None) -> tuple[str, list[RouteWaypoint]]:
    if isinstance(payload, dict):
        raw_waypoints = payload.get("waypoints", [])
        mission_name = str(payload.get("mission_name") or default_mission_name or "auto_scan")
    elif isinstance(payload, list):
        raw_waypoints = payload
        mission_name = str(default_mission_name or "auto_scan")
    else:
        raise RuntimeError("route payload must be a mapping or a list")

    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise RuntimeError("route payload must contain a non-empty `waypoints` list")

    loaded: list[RouteWaypoint] = []
    seen_ids: set[str] = set()
    for index, item in enumerate(raw_waypoints, start=1):
        if not isinstance(item, dict):
            raise RuntimeError(f"waypoint #{index} is not a mapping")
        waypoint_id = str(item.get("id") or item.get("name") or f"wp_{index:02d}")
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
            RouteWaypoint(
                waypoint_id=waypoint_id,
                x=x,
                y=y,
                yaw=normalize_angle(yaw),
                dwell_sec=dwell_sec,
                note=str(item.get("note", "")),
            )
        )
    return mission_name, loaded


def normalize_route_yaml(route_yaml: str, *, default_mission_name: str | None = None) -> tuple[str, list[RouteWaypoint], str]:
    payload = yaml.safe_load(route_yaml or "") or {}
    mission_name, waypoints = normalize_route_payload(payload, default_mission_name=default_mission_name)
    normalized_payload = {
        "mission_name": mission_name,
        "waypoints": [
            {
                "id": waypoint.waypoint_id,
                "x": waypoint.x,
                "y": waypoint.y,
                "yaw": waypoint.yaw,
                "dwell_sec": waypoint.dwell_sec,
                "note": waypoint.note,
            }
            for waypoint in waypoints
        ],
    }
    return mission_name, waypoints, yaml.safe_dump(normalized_payload, sort_keys=False, allow_unicode=True)


def route_path(route_root: Path, route_id: str) -> Path:
    return route_root / f"{validate_route_id(route_id)}.yaml"


def list_routes(route_root: Path) -> list[str]:
    if not route_root.exists():
        return []
    return sorted(path.stem for path in route_root.glob("*.yaml") if path.is_file())


def load_route(route_root: Path, route_id: str) -> tuple[Path, str]:
    path = route_path(route_root, route_id)
    if not path.exists():
        raise FileNotFoundError(f"route not found: {route_id}")
    return path, path.read_text(encoding="utf-8")


def save_route(route_root: Path, route_id: str, route_yaml: str) -> tuple[Path, str]:
    path = route_path(route_root, route_id)
    path.parent.mkdir(parents=True, exist_ok=True)
    _, _, normalized_yaml = normalize_route_yaml(route_yaml, default_mission_name=route_id)
    path.write_text(normalized_yaml, encoding="utf-8")
    return path, normalized_yaml


def delete_route(route_root: Path, route_id: str) -> Path:
    path = route_path(route_root, route_id)
    if not path.exists():
        raise FileNotFoundError(f"route not found: {route_id}")
    path.unlink()
    return path


def default_auto_scan_script() -> Path:
    return Path(__file__).resolve().with_name("auto_scan_mission.py")


def build_auto_scan_command(
    auto_scan_script: Path,
    route_file: Path,
    *,
    mission_name: str,
    dry_run: bool,
    stop_on_failure: bool,
    save_map_on_finish: bool,
    save_map_on_failure: bool,
) -> list[str]:
    return [
        sys.executable,
        str(auto_scan_script),
        "--ros-args",
        "-p",
        f"waypoints_file:={route_file}",
        "-p",
        f"mission_name:={mission_name}",
        "-p",
        f"dry_run:={'true' if dry_run else 'false'}",
        "-p",
        f"stop_on_failure:={'true' if stop_on_failure else 'false'}",
        "-p",
        f"save_map_on_finish:={'true' if save_map_on_finish else 'false'}",
        "-p",
        f"save_map_on_failure:={'true' if save_map_on_failure else 'false'}",
    ]


class TaskManager(Node):
    def __init__(self) -> None:
        super().__init__("task_manager")
        self.runtime_mode = self.declare_parameter("runtime_mode", "real").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.navigation_backend = self.declare_parameter("navigation_backend", "pose_topic_3d").value
        self.pose_goal_topic = self.declare_parameter("pose_goal_topic", "/a2/nav3/goal_pose").value
        self.navigate_action_name = self.declare_parameter("navigate_action_name", "/navigate_to_pose").value
        self.manage_map_service = self.declare_parameter("manage_map_service", "/map_manager/manage_map").value
        self.set_mode_service = self.declare_parameter("set_mode_service", "/map_manager/set_mode").value
        self.initial_pose_topic = self.declare_parameter("initial_pose_topic", "/initialpose").value
        self.active_map_topic = self.declare_parameter("active_map_topic", "/a2/map_manager/active_map").value
        self.nav_status_topic = self.declare_parameter("nav_status_topic", "/a2/nav2/status").value
        self.mission_status_topic = self.declare_parameter("mission_status_topic", "/a2/scan_mission/status").value
        self.mission_report_topic = self.declare_parameter("mission_report_topic", "/a2/scan_mission/report").value
        self.task_status_topic = self.declare_parameter("task_status_topic", "/a2/task_manager/status").value
        self.task_report_topic = self.declare_parameter("task_report_topic", "/a2/task_manager/report").value
        raw_route_root = self.declare_parameter(
            "route_root", "${HOME}/a2_system_ws/runtime/routes"
        ).value
        self.route_root = Path(os.path.expandvars(os.path.expanduser(raw_route_root)))
        raw_auto_scan_script = self.declare_parameter(
            "auto_scan_script", str(default_auto_scan_script())
        ).value
        self.auto_scan_script = Path(os.path.expandvars(os.path.expanduser(raw_auto_scan_script)))
        self.command_timeout_sec = float(self.declare_parameter("command_timeout_sec", 10.0).value)
        self.route_stop_timeout_sec = float(self.declare_parameter("route_stop_timeout_sec", 5.0).value)
        # When true, dispatch missions via the RunMission ROS 2 action instead of
        # subprocess.Popen(auto_scan_mission.py). The mission node continues to publish
        # /a2/scan_mission/* String topics so external consumers (Web Console) are
        # unaffected. Defaults to false until the action server is verified at runtime.
        self.task_manager_use_action = bool(
            self.declare_parameter("task_manager_use_action", True).value
        )
        self.run_mission_action_name = self.declare_parameter(
            "run_mission_action_name", "/run_mission"
        ).value
        self.run_mission_goal_timeout_sec = float(
            self.declare_parameter("run_mission_goal_timeout_sec", 5.0).value
        )
        self.callback_group = ReentrantCallbackGroup()

        self.route_root.mkdir(parents=True, exist_ok=True)
        self.current_mode = "unknown"
        self.active_map = ""
        self.nav_status_raw = ""
        self.mission_status_raw = ""
        self.mission_report_path = ""
        self.last_status = ""
        self._active_goal_handle: Any | None = None
        self._route_process: subprocess.Popen[str] | None = None
        # When dispatching missions via the RunMission action, this holds the
        # active goal handle so we can cancel/stop later. None when no goal is in flight.
        self._route_action_goal_handle: Any | None = None
        self._route_action_result_future: Any | None = None
        self._route_state = "idle"
        self._route_id = ""
        self._route_path = ""

        self.status_pub = self.create_publisher(String, self.task_status_topic, 10)
        self.report_pub = self.create_publisher(String, self.task_report_topic, 10)
        self.pose_goal_pub = self.create_publisher(PoseStamped, self.pose_goal_topic, 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.initial_pose_topic, 10)
        self.create_subscription(String, self.active_map_topic, self.on_active_map, 10)
        self.create_subscription(String, self.nav_status_topic, self.on_nav_status, 10)
        self.create_subscription(String, self.mission_status_topic, self.on_mission_status, 10)
        self.create_subscription(String, self.mission_report_topic, self.on_mission_report, 10)
        self.command_service = self.create_service(
            NavCommand,
            "/a2/task_manager/command",
            self.handle_command,
            callback_group=self.callback_group,
        )
        self.manage_map_client = self.create_client(
            ManageMap,
            self.manage_map_service,
            callback_group=self.callback_group,
        )
        self.set_mode_client = self.create_client(
            SetMode,
            self.set_mode_service,
            callback_group=self.callback_group,
        )
        self.run_mission_client = None
        if self.task_manager_use_action:
            if not _HAS_RUN_MISSION_ACTION:
                self.get_logger().error(
                    "task_manager_use_action=true but a2_interfaces.action.RunMission is "
                    "not importable; falling back to subprocess mode"
                )
                self.task_manager_use_action = False
            else:
                self.run_mission_client = ActionClient(
                    self,
                    RunMissionAction,
                    self.run_mission_action_name,
                    callback_group=self.callback_group,
                )
        self.navigate_client = (
            ActionClient(
                self,
                NavigateToPose,
                self.navigate_action_name,
                callback_group=self.callback_group,
            )
            if NavigateToPose is not None and self.navigation_backend == "nav2"
            else None
        )
        self.create_timer(0.5, self.poll_route_process)
        self.publish_status("ready", "idle")

    def on_active_map(self, msg: String) -> None:
        self.active_map = msg.data
        self.publish_status("ready", "active_map_update")

    def on_nav_status(self, msg: String) -> None:
        self.nav_status_raw = msg.data
        self.publish_status("ready", "nav_status_update")

    def on_mission_status(self, msg: String) -> None:
        self.mission_status_raw = msg.data
        fields = parse_status_string(msg.data)
        state = fields.get("state")
        if state == "mission_complete":
            ready = fields.get("ready", "false").lower() == "true"
            self._route_state = "succeeded" if ready else "failed"
        elif state in {"mission_start", "preflight_ready", "waypoint_dispatch", "waypoint_complete"}:
            self._route_state = "running"
        self.publish_status("ready", "mission_status_update")

    def on_mission_report(self, msg: String) -> None:
        self.mission_report_path = msg.data
        self.report_pub.publish(msg)
        self.publish_status("ready", "mission_report_update")

    def _route_active(self) -> bool:
        return (
            self._route_process is not None
            or self._route_action_goal_handle is not None
        )

    def poll_route_process(self) -> None:
        if self._route_process is None:
            return
        return_code = self._route_process.poll()
        if return_code is None:
            return
        if self._route_state not in {"succeeded", "failed"}:
            self._route_state = "succeeded" if return_code == 0 else "failed"
        self.get_logger().info(
            f"Route process exited: route_id={self._route_id or 'none'}, code={return_code}, state={self._route_state}"
        )
        self._route_process = None
        self.publish_status("ready", f"route_process_exit:{return_code}")

    def wait_for_future(self, future: Any, timeout_sec: float) -> bool:
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.done()

    def publish_status(self, state: str, reason: str) -> None:
        ready = not self._route_active()
        status = (
            f"mode={self.runtime_mode};state={state};ready={str(bool(ready)).lower()};reason={reason};"
            f"current_mode={self.current_mode};active_map={self.active_map or 'none'};"
            f"route_state={self._route_state};route_id={self._route_id or 'none'};"
            f"route_path={self._route_path or 'none'};report_path={self.mission_report_path or 'none'}"
        )
        self.status_pub.publish(String(data=status))
        if status != self.last_status:
            self.get_logger().info(f"Task manager status: {status}")
            self.last_status = status

    def ensure_pose(self, pose: PoseStamped) -> PoseStamped:
        normalized = PoseStamped()
        normalized.header = pose.header
        normalized.pose = pose.pose
        if not normalized.header.frame_id:
            normalized.header.frame_id = self.map_frame
        if (
            self.navigation_backend != "nav2"
            and normalized.header.stamp.sec == 0
            and normalized.header.stamp.nanosec == 0
        ):
            normalized.header.stamp = self.get_clock().now().to_msg()
        orientation = normalized.pose.orientation
        if (
            abs(orientation.x) < 1e-9
            and abs(orientation.y) < 1e-9
            and abs(orientation.z) < 1e-9
            and abs(orientation.w) < 1e-9
        ):
            orientation.w = 1.0
        return normalized

    def call_manage_map(self, command: str, map_id: str) -> ManageMap.Response:
        if not self.manage_map_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("manage_map service unavailable")
        request = ManageMap.Request()
        request.command = command
        request.map_id = map_id
        future = self.manage_map_client.call_async(request)
        if not self.wait_for_future(future, self.command_timeout_sec):
            raise RuntimeError(f"manage_map {command} timed out")
        response = future.result()
        if response is None:
            raise RuntimeError(f"manage_map {command} returned no response")
        return response

    def call_set_mode(self, mode: str) -> SetMode.Response:
        if not self.set_mode_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("set_mode service unavailable")
        request = SetMode.Request()
        request.mode = mode
        future = self.set_mode_client.call_async(request)
        if not self.wait_for_future(future, self.command_timeout_sec):
            raise RuntimeError("set_mode timed out")
        response = future.result()
        if response is None:
            raise RuntimeError("set_mode returned no response")
        if response.success:
            self.current_mode = mode
        return response

    def send_goal(self, pose: PoseStamped) -> str:
        if self.navigation_backend == "pose_topic_3d":
            normalized = self.ensure_pose(pose)
            self.pose_goal_pub.publish(normalized)
            self.publish_status("goal_active", f"pose_topic_goal_published:{self.pose_goal_topic}")
            return "3D pose goal published"
        if self.navigate_client is None or NavigateToPose is None:
            raise RuntimeError("NavigateToPose action client unavailable")
        if self._active_goal_handle is not None:
            raise RuntimeError("another navigation goal is already active")
        if not self.navigate_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("NavigateToPose action server unavailable")

        goal = NavigateToPose.Goal()
        goal.pose = self.ensure_pose(pose)
        future = self.navigate_client.send_goal_async(goal)
        if not self.wait_for_future(future, self.command_timeout_sec):
            raise RuntimeError("send goal timed out")
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("navigation goal rejected")
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)
        self.publish_status("goal_active", "action_goal_accepted")
        return "navigation goal accepted"

    def on_goal_result(self, future: Any) -> None:
        reason = "action_goal_unknown"
        try:
            result = future.result()
            status = result.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                reason = "action_goal_succeeded"
            elif status == GoalStatus.STATUS_CANCELED:
                reason = "action_goal_canceled"
            elif status == GoalStatus.STATUS_ABORTED:
                reason = "action_goal_aborted"
            else:
                reason = f"action_goal_status_{status}"
        except Exception as exc:  # pragma: no cover - defensive path
            reason = f"action_goal_exception:{exc}"
        self._active_goal_handle = None
        self.publish_status("goal_complete", reason)

    def cancel_goal(self) -> str:
        if self._active_goal_handle is None:
            return "no active goal"
        future = self._active_goal_handle.cancel_goal_async()
        if not self.wait_for_future(future, self.command_timeout_sec):
            raise RuntimeError("cancel goal timed out")
        response = future.result()
        if response is None or not getattr(response, "goals_canceling", []):
            raise RuntimeError("cancel goal rejected")
        self.publish_status("goal_cancel_requested", "action_goal_cancel_requested")
        return "cancel requested"

    def publish_initial_pose(self, pose: PoseStamped) -> str:
        normalized = self.ensure_pose(pose)
        covariance = [0.0] * 36
        covariance[0] = 0.25
        covariance[7] = 0.25
        covariance[35] = 0.068
        for _ in range(3):
            msg = PoseWithCovarianceStamped()
            msg.header = normalized.header
            msg.pose.pose = normalized.pose
            msg.pose.covariance = covariance
            self.initial_pose_pub.publish(msg)
            time.sleep(0.1)
        self.publish_status("initial_pose_sent", "published_three_times")
        return "initial pose published"

    def _on_run_mission_feedback(self, fb_msg) -> None:
        try:
            fb = fb_msg.feedback
            self.mission_status_raw = fb.last_status_kv or self.mission_status_raw
            if fb.state == "mission_complete":
                # final feedback before result; do nothing here, result handler updates state
                pass
            else:
                self._route_state = "running"
        except Exception as exc:  # pragma: no cover - best-effort
            self.get_logger().warning(f"run_mission feedback handler failed: {exc}")

    def _on_run_mission_result(self, future) -> None:
        try:
            wrapped = future.result()
            status = getattr(wrapped, "status", GoalStatus.STATUS_UNKNOWN)
            result = getattr(wrapped, "result", None)
            success = bool(getattr(result, "success", False)) if result is not None else False
            self._route_state = "succeeded" if (
                status == GoalStatus.STATUS_SUCCEEDED and success
            ) else (
                "canceled" if status == GoalStatus.STATUS_CANCELED else "failed"
            )
            if result is not None and getattr(result, "report_json_path", ""):
                self.mission_report_path = result.report_json_path
        except Exception as exc:  # pragma: no cover - best-effort
            self.get_logger().warning(f"run_mission result handler failed: {exc}")
            self._route_state = "failed"
        finally:
            self._route_action_goal_handle = None
            self._route_action_result_future = None
            self.publish_status("ready", f"route_action_done:{self._route_state}")

    def _start_route_via_action(
        self,
        route_id: str,
        route_file: Path,
        request: NavCommand.Request,
        mission_name: str,
    ) -> tuple[str, str]:
        client = self.run_mission_client
        if client is None or not _HAS_RUN_MISSION_ACTION:
            raise RuntimeError("RunMission action client unavailable")
        if not client.wait_for_server(timeout_sec=self.run_mission_goal_timeout_sec):
            raise RuntimeError(
                f"RunMission action server '{self.run_mission_action_name}' not available"
            )
        goal = RunMissionAction.Goal()
        goal.mission_name = mission_name
        goal.waypoints_file = str(route_file)
        goal.require_real_ready = bool(getattr(request, "require_real_ready", True))
        goal.stop_on_failure = bool(request.stop_on_failure)

        send_future = client.send_goal_async(
            goal, feedback_callback=self._on_run_mission_feedback
        )
        if not self.wait_for_future(send_future, self.run_mission_goal_timeout_sec):
            raise RuntimeError("RunMission send_goal timed out")
        goal_handle = send_future.result()
        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            raise RuntimeError("RunMission goal rejected by mission server")
        self._route_action_goal_handle = goal_handle
        self._route_action_result_future = goal_handle.get_result_async()
        self._route_action_result_future.add_done_callback(self._on_run_mission_result)
        self._route_state = "running"
        self._route_id = route_id
        self._route_path = str(route_file)
        self.mission_report_path = ""
        self.mission_status_raw = ""
        self.publish_status("route_started", "mission_action_dispatched")
        return route_id, str(route_file)

    def start_route(self, request: NavCommand.Request) -> tuple[str, str]:
        if self._route_active():
            raise RuntimeError("another route mission is already running")
        if request.route_id:
            route_id = validate_route_id(request.route_id)
            route_file = route_path(self.route_root, route_id)
        elif request.waypoints_file:
            route_file = Path(os.path.expandvars(os.path.expanduser(request.waypoints_file))).resolve()
            route_id = route_file.stem
        else:
            raise RuntimeError("route_id or waypoints_file is required")
        if not route_file.exists():
            raise FileNotFoundError(f"route file not found: {route_file}")
        mission_name = (request.mission_name or route_id or "auto_scan").strip()

        if self.task_manager_use_action and self.run_mission_client is not None:
            return self._start_route_via_action(route_id, route_file, request, mission_name)

        command = build_auto_scan_command(
            self.auto_scan_script,
            route_file,
            mission_name=mission_name,
            dry_run=bool(request.dry_run),
            stop_on_failure=bool(request.stop_on_failure),
            save_map_on_finish=bool(request.save_map_on_finish),
            save_map_on_failure=bool(request.save_map_on_failure),
        )
        self.mission_report_path = ""
        self.mission_status_raw = ""
        self._route_process = subprocess.Popen(
            command,
            cwd=str(self.route_root),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
            env=os.environ.copy(),
        )
        self._route_state = "running"
        self._route_id = route_id
        self._route_path = str(route_file)
        self.publish_status("route_started", "mission_subprocess_spawned")
        return route_id, str(route_file)

    def stop_route(self) -> str:
        if self._route_action_goal_handle is not None:
            handle = self._route_action_goal_handle
            try:
                cancel_future = handle.cancel_goal_async()
                self.wait_for_future(cancel_future, self.route_stop_timeout_sec)
            except Exception as exc:  # pragma: no cover - best-effort cancel
                self.get_logger().warning(f"cancel_goal_async failed: {exc}")
            # Result callback will clear _route_action_goal_handle and update state.
            self._route_state = "stopping"
            self.publish_status("route_stopping", "mission_action_cancel_requested")
            return "route mission cancel requested"
        if self._route_process is None:
            self._route_state = "idle"
            return "no active route mission"
        process = self._route_process
        process.terminate()
        try:
            process.wait(timeout=self.route_stop_timeout_sec)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)
        self._route_process = None
        self._route_state = "stopped"
        self.publish_status("route_stopped", "mission_subprocess_terminated")
        return "route mission stopped"

    def populate_response(self, response: NavCommand.Response) -> NavCommand.Response:
        response.active_map = self.active_map
        response.current_mode = self.current_mode
        response.route_id = self._route_id
        response.route_path = self._route_path
        response.mission_state = self._route_state
        response.report_path = self.mission_report_path
        return response

    def handle_command(self, request: NavCommand.Request, response: NavCommand.Response) -> NavCommand.Response:
        command = (request.command or "").strip().lower()
        try:
            if command == "list_maps":
                result = self.call_manage_map("list", "")
                response.success = bool(result.success)
                response.message = result.message
                response.items = list(result.map_ids)
            elif command == "save_map":
                result = self.call_manage_map("save", request.map_id)
                response.success = bool(result.success)
                response.message = result.message
                response.items = list(result.map_ids)
            elif command == "load_map":
                result = self.call_manage_map("load", request.map_id)
                response.success = bool(result.success)
                response.message = result.message
                response.items = list(result.map_ids)
            elif command == "promote_map":
                result = self.call_manage_map("promote", request.map_id)
                response.success = bool(result.success)
                response.message = result.message
                response.items = list(result.map_ids)
            elif command == "set_mode":
                result = self.call_set_mode(request.mode)
                response.success = bool(result.success)
                response.message = result.message
            elif command == "send_goal":
                response.success = True
                response.message = self.send_goal(request.pose)
            elif command == "cancel_goal":
                response.success = True
                response.message = self.cancel_goal()
            elif command == "set_initial_pose":
                response.success = True
                response.message = self.publish_initial_pose(request.pose)
            elif command == "list_routes":
                response.success = True
                response.message = "listed routes"
                response.items = list_routes(self.route_root)
            elif command == "get_route":
                route_id = validate_route_id(request.route_id)
                path, route_yaml = load_route(self.route_root, route_id)
                response.success = True
                response.message = "loaded route"
                response.route_id = route_id
                response.route_path = str(path)
                response.route_yaml = route_yaml
            elif command == "save_route":
                route_id = validate_route_id(request.route_id)
                path, route_yaml = save_route(self.route_root, route_id, request.route_yaml)
                response.success = True
                response.message = "saved route"
                response.route_id = route_id
                response.route_path = str(path)
                response.route_yaml = route_yaml
                response.items = list_routes(self.route_root)
            elif command == "delete_route":
                route_id = validate_route_id(request.route_id)
                path = delete_route(self.route_root, route_id)
                response.success = True
                response.message = f"deleted route {route_id}"
                response.route_id = route_id
                response.route_path = str(path)
                response.items = list_routes(self.route_root)
            elif command == "run_route":
                route_id, path = self.start_route(request)
                response.success = True
                response.message = "route mission started"
                response.route_id = route_id
                response.route_path = path
                response.mission_state = self._route_state
            elif command == "stop_route":
                response.success = True
                response.message = self.stop_route()
                response.mission_state = self._route_state
            elif command == "route_status":
                response.success = True
                response.message = "route status"
                response.mission_state = self._route_state
                response.route_id = self._route_id
                response.route_path = self._route_path
                response.report_path = self.mission_report_path
            else:
                raise RuntimeError(f"unsupported command: {request.command}")
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.publish_status("error", f"{command or 'unknown'}:{exc}")
        return self.populate_response(response)


def main() -> None:
    rclpy.init()
    node = TaskManager()
    executor: MultiThreadedExecutor | None = None
    try:
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    finally:
        if node._route_process is not None and node._route_process.poll() is None:
            try:
                node._route_process.send_signal(signal.SIGTERM)
            except Exception:
                pass
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
