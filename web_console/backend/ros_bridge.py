from __future__ import annotations

import threading
import base64
import io
import json
import math
import struct
import time
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import BatteryState, CompressedImage, Image, PointCloud2

try:
    from PIL import Image as PilImage
except ImportError:  # pragma: no cover - optional runtime dependency
    PilImage = None

try:
    from a2_interfaces.msg import RobotState
except ImportError:  # pragma: no cover - runtime environment fallback
    RobotState = None

try:
    from a2_interfaces.msg import LightCommand
except ImportError:  # pragma: no cover - runtime environment fallback
    LightCommand = None

try:
    from a2_interfaces.srv import ManageMap, NavCommand
except ImportError:  # pragma: no cover - runtime environment fallback
    ManageMap = None
    NavCommand = None

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover - runtime environment fallback
    NavigateToPose = None

try:
    from unitree_api.msg import (
        Request,
        RequestHeader,
        RequestIdentity,
        RequestLease,
        RequestPolicy,
        Response,
    )
except ImportError:  # pragma: no cover - runtime environment fallback
    Request = None
    RequestHeader = None
    RequestIdentity = None
    RequestLease = None
    RequestPolicy = None
    Response = None

from .config import AppConfig
from .models import (
    BatterySnapshot,
    RecoveryStatus,
    DashboardSnapshot,
    CameraFrame,
    InitialPoseRequest,
    MapSnapshot,
    PointCloudSnapshot,
    NavigationGoal,
    NavigationGoalRequest,
    NavigationTaskState,
    Pose2D,
    RawStateSummary,
    RobotPose,
    RobotStatus,
    SystemHealth,
    TaskRouteStatus,
    TextStatus,
)
from .utils import deep_copy_model, dump_model, now_iso, parse_optional_bool, parse_status_string, quaternion_to_yaw
from .ws import WebSocketManager


class RosBridgeError(RuntimeError):
    """Raised when the backend cannot execute a ROS-side command."""


def _battery_percent_0_100(value: float | None) -> float | None:
    if value is None:
        return None
    try:
        raw = float(value)
    except Exception:
        return None
    if not math.isfinite(raw) or raw < 0.0:
        return None
    pct = raw * 100.0 if raw <= 1.0 else raw
    return float(max(0.0, min(100.0, pct)))


def _world_to_grid(map_snapshot: MapSnapshot, x: float, y: float) -> tuple[int, int] | None:
    if not map_snapshot.loaded or map_snapshot.resolution <= 0.0 or map_snapshot.width <= 0 or map_snapshot.height <= 0:
        return None
    dx = x - map_snapshot.origin.x
    dy = y - map_snapshot.origin.y
    cos_yaw = math.cos(-map_snapshot.origin.yaw)
    sin_yaw = math.sin(-map_snapshot.origin.yaw)
    local_x = cos_yaw * dx - sin_yaw * dy
    local_y = sin_yaw * dx + cos_yaw * dy
    grid_x = int(math.floor(local_x / map_snapshot.resolution))
    grid_y = int(math.floor(local_y / map_snapshot.resolution))
    if grid_x < 0 or grid_x >= map_snapshot.width or grid_y < 0 or grid_y >= map_snapshot.height:
        return None
    return grid_x, grid_y


def _grid_to_world(map_snapshot: MapSnapshot, grid_x: int, grid_y: int) -> tuple[float, float]:
    local_x = (grid_x + 0.5) * map_snapshot.resolution
    local_y = (grid_y + 0.5) * map_snapshot.resolution
    cos_yaw = math.cos(map_snapshot.origin.yaw)
    sin_yaw = math.sin(map_snapshot.origin.yaw)
    world_x = map_snapshot.origin.x + cos_yaw * local_x - sin_yaw * local_y
    world_y = map_snapshot.origin.y + sin_yaw * local_x + cos_yaw * local_y
    return world_x, world_y


def _grid_value(map_snapshot: MapSnapshot, grid_x: int, grid_y: int) -> int:
    if grid_x < 0 or grid_x >= map_snapshot.width or grid_y < 0 or grid_y >= map_snapshot.height:
        return 100
    index = grid_x + grid_y * map_snapshot.width
    if index < 0 or index >= len(map_snapshot.data):
        return 100
    return int(map_snapshot.data[index])


def _cell_has_clearance(
    map_snapshot: MapSnapshot,
    grid_x: int,
    grid_y: int,
    clearance_cells: int,
    occupancy_block_threshold: int,
) -> bool:
    if clearance_cells <= 0:
        value = _grid_value(map_snapshot, grid_x, grid_y)
        return value >= 0 and value < occupancy_block_threshold

    for dy in range(-clearance_cells, clearance_cells + 1):
        for dx in range(-clearance_cells, clearance_cells + 1):
            if dx * dx + dy * dy > clearance_cells * clearance_cells:
                continue
            value = _grid_value(map_snapshot, grid_x + dx, grid_y + dy)
            if value < 0 or value >= occupancy_block_threshold:
                return False
    return True


def _snap_pose_to_free_cell(
    map_snapshot: MapSnapshot,
    pose: NavigationGoal,
    clearance_m: float,
    max_radius_m: float,
    occupancy_block_threshold: int,
) -> tuple[NavigationGoal, bool]:
    requested = _world_to_grid(map_snapshot, pose.x, pose.y)
    if requested is None:
        raise RosBridgeError("选点超出地图范围")

    clearance_cells = max(0, int(math.ceil(clearance_m / map_snapshot.resolution)))
    max_radius_cells = max(0, int(math.ceil(max_radius_m / map_snapshot.resolution)))
    grid_x, grid_y = requested

    for current_clearance in (clearance_cells, 0):
        if _cell_has_clearance(map_snapshot, grid_x, grid_y, current_clearance, occupancy_block_threshold):
            return pose, False

        for radius in range(1, max_radius_cells + 1):
            best: tuple[int, int] | None = None
            best_distance: float | None = None
            for candidate_y in range(grid_y - radius, grid_y + radius + 1):
                for candidate_x in range(grid_x - radius, grid_x + radius + 1):
                    if max(abs(candidate_x - grid_x), abs(candidate_y - grid_y)) != radius:
                        continue
                    if not _cell_has_clearance(
                        map_snapshot,
                        candidate_x,
                        candidate_y,
                        current_clearance,
                        occupancy_block_threshold,
                    ):
                        continue
                    distance = math.hypot(candidate_x - grid_x, candidate_y - grid_y)
                    if best_distance is None or distance < best_distance:
                        best = (candidate_x, candidate_y)
                        best_distance = distance
            if best is None:
                continue
            snapped_x, snapped_y = _grid_to_world(map_snapshot, best[0], best[1])
            return pose.model_copy(update={"x": snapped_x, "y": snapped_y}), True

    raise RosBridgeError("选点附近没有可行栅格")


class RosBridgeNode(Node):
    def __init__(self, config: AppConfig, ws_manager: WebSocketManager) -> None:
        super().__init__("a2_web_console")
        self.config = config
        self.ws_manager = ws_manager
        self._lock = threading.RLock()
        self._navigation_lock = threading.Lock()
        self._active_goal_handle: Any | None = None

        self.map_snapshot = MapSnapshot()
        self.pointcloud_snapshot = PointCloudSnapshot()
        self.pose = RobotPose()
        self.status = RobotStatus(
            planner_type="SmacPlannerHybrid",
            bt_filename="a2_navigate_3d.xml",
        )
        self.navigation = NavigationTaskState(updated_at=now_iso())
        self.camera = CameraFrame()
        self.battery = BatterySnapshot()
        self.recovery_status = RecoveryStatus()
        self.health = SystemHealth(ros_connected=True)
        self._last_tf_frame: str | None = None
        self._last_camera_publish_monotonic = 0.0
        self._last_primary_pointcloud_monotonic = 0.0
        self._last_fallback_pointcloud_monotonic = 0.0
        self._last_pose_monotonic = 0.0
        self._last_battery_monotonic = 0.0
        self._last_battery_warn_monotonic = 0.0
        self._active_pose_goal: NavigationGoal | None = None
        self._active_pose_goal_started_at: float | None = None
        self._native_slam_response_cv = threading.Condition()
        self._native_slam_responses: dict[int, dict[str, Any]] = {}
        self.manage_map_client = None
        self.task_command_client = None

        self._setup_subscriptions()
        if ManageMap is not None:
            self.manage_map_client = self.create_client(
                ManageMap, self.config.ros.manage_map_service
            )
        else:
            self.get_logger().warning(
                "a2_interfaces.srv.ManageMap is unavailable. Map save/load bridge will be disabled."
            )
        if NavCommand is not None:
            self.task_command_client = self.create_client(
                NavCommand, self.config.ros.task_manager_service
            )
        else:
            self.get_logger().warning(
                "a2_interfaces.srv.NavCommand is unavailable. Task manager bridge will be disabled."
            )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.config.navigation.initial_pose_topic,
            10,
        )
        self.pose_goal_publisher = self.create_publisher(
            PoseStamped,
            self.config.navigation.goal_topic,
            10,
        )
        self.cancel_stop_publisher = self.create_publisher(
            Twist,
            self.config.navigation.cancel_stop_topic,
            10,
        )
        self.light_command_publisher = None
        if LightCommand is not None:
            self.light_command_publisher = self.create_publisher(
                LightCommand,
                self.config.ros.light_command_topic,
                10,
            )
        self.native_slam_publisher = None
        if self.config.native_slam.enabled and Request is not None:
            self.native_slam_publisher = self.create_publisher(
                Request,
                self.config.native_slam.request_topic,
                10,
            )
            if Response is not None:
                self.create_subscription(
                    Response,
                    self.config.native_slam.response_topic,
                    self._on_native_slam_response,
                    10,
                )
        elif self.config.native_slam.enabled:
            self.get_logger().warning(
                "unitree_api.msg.Request is unavailable. Native SLAM commands will be disabled."
            )
        self.action_client = None
        if self.config.navigation.backend == "nav2" and NavigateToPose is not None:
            self.action_client = ActionClient(self, NavigateToPose, self.config.navigation.action_name)
        if self.config.navigation.backend == "nav2" and self.action_client is None:
            self.get_logger().warning("nav2_msgs.action.NavigateToPose is unavailable. Navigation controls will be disabled.")
        health_period = max(0.2, 1.0 / max(self.config.health.health_broadcast_hz, 0.1))
        self.create_timer(health_period, self._publish_health)

    def _setup_subscriptions(self) -> None:
        ros = self.config.ros
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, ros.map_topic, self._on_map, latched_qos)
        self.create_subscription(OccupancyGrid, ros.map_topic, self._on_map, 10)
        self.create_subscription(
            PointCloud2,
            ros.pointcloud_topic,
            lambda msg: self._on_pointcloud(msg, ros.pointcloud_topic, primary=True),
            10,
        )
        if ros.pointcloud_fallback_topic and ros.pointcloud_fallback_topic != ros.pointcloud_topic:
            self.create_subscription(
                PointCloud2,
                ros.pointcloud_fallback_topic,
                lambda msg: self._on_pointcloud(msg, ros.pointcloud_fallback_topic, primary=False),
                10,
            )
        if ros.localization_pose_msg_type == "nav_msgs/msg/Odometry":
            self.create_subscription(Odometry, ros.localization_pose_topic, self._on_localization_odom, 20)
        else:
            self.create_subscription(
                PoseWithCovarianceStamped,
                ros.localization_pose_topic,
                self._on_localization_pose,
                20,
            )
        self.create_subscription(Odometry, ros.odom_topic, self._on_odom, 20)
        self.create_subscription(TFMessage, ros.tf_topic, self._on_tf, 20)
        self.create_subscription(String, ros.real_report_topic, self._on_real_report, 10)
        self.create_subscription(String, ros.lidar_status_topic, self._on_lidar_status, 10)
        self.create_subscription(String, ros.camera_status_topic, self._on_camera_status, 10)
        self.create_subscription(Bool, ros.localization_ok_topic, self._on_localization_ok, 10)
        self.create_subscription(String, ros.localization_status_topic, self._on_localization_status, 10)
        self.create_subscription(String, ros.relocalization_status_topic, self._on_relocalization_status, 10)
        self.create_subscription(String, ros.safety_status_topic, self._on_safety_status, 10)
        self.create_subscription(String, ros.map_manager_status_topic, self._on_map_manager_status, 10)
        self.create_subscription(String, ros.map_manager_active_map_topic, self._on_active_map, 10)
        self.create_subscription(String, ros.task_manager_status_topic, self._on_task_manager_status, 10)
        self.create_subscription(String, ros.pose_goal_status_topic, self._on_pose_goal_status, 10)
        self.create_subscription(String, ros.sdk_status_topic, self._on_sdk_status, 10)
        if RobotState is not None:
            self.create_subscription(RobotState, ros.raw_state_topic, self._on_raw_state, 10)
        else:
            self.get_logger().warning("a2_interfaces.msg.RobotState is unavailable. /a2/raw_state will be skipped.")
        battery_topics = [str(ros.battery_topic or "").strip()]
        for fallback in ["/battery", "/a2/battery"]:
            if fallback and fallback not in battery_topics:
                battery_topics.append(fallback)
        for topic in [t for t in battery_topics if t]:
            self.create_subscription(BatteryState, topic, self._on_battery, 10)
        self.create_subscription(String, ros.scan_mission_status_topic, self._on_scan_mission_status, 10)

        if self.config.camera.enabled:
            if self.config.camera.prefer_compressed:
                self.create_subscription(CompressedImage, ros.camera_compressed_topic, self._on_compressed_image, 5)
            self.create_subscription(Image, ros.camera_image_topic, self._on_image, 5)

    def _status_from_string(self, raw: str | None) -> TextStatus:
        parsed_raw, fields = parse_status_string(raw)
        return TextStatus(
            raw=parsed_raw,
            mode=fields.get("mode"),
            state=fields.get("state"),
            ready=parse_optional_bool(fields.get("ready")),
            reason=fields.get("reason"),
            fields=fields,
        )

    def _publish(self, event_type: str, payload: Any) -> None:
        self.ws_manager.broadcast_threadsafe({"type": event_type, "payload": payload})

    def _camera_throttle_ready(self) -> bool:
        max_hz = max(0.1, float(self.config.camera.max_broadcast_hz))
        now = time.monotonic()
        if now - self._last_camera_publish_monotonic < 1.0 / max_hz:
            return False
        self._last_camera_publish_monotonic = now
        return True

    def _set_health_error(self, message: str) -> None:
        with self._lock:
            self.health.last_error = message
        self._publish("health", self.get_health_dict())

    def _on_native_slam_response(self, msg: Any) -> None:
        response = {
            "request_id": int(msg.header.identity.id),
            "api_id": int(msg.header.identity.api_id),
            "code": int(msg.header.status.code),
            "data": msg.data,
        }
        with self._native_slam_response_cv:
            self._native_slam_responses[response["request_id"]] = response
            self._native_slam_response_cv.notify_all()

    def _publish_native_slam_request(self, api_id: int, data: dict[str, Any]) -> dict[str, Any]:
        if not self.config.native_slam.enabled:
            raise RosBridgeError("Native SLAM command path is disabled by config")
        if (
            self.native_slam_publisher is None
            or Request is None
            or RequestHeader is None
            or RequestIdentity is None
            or RequestLease is None
            or RequestPolicy is None
        ):
            raise RosBridgeError("unitree_api.msg.Request is unavailable")

        request_id = int(time.time() * 1000)
        request = Request()
        request.header = RequestHeader()
        request.header.identity = RequestIdentity()
        request.header.identity.api_id = int(api_id)
        request.header.identity.id = request_id
        request.header.lease = RequestLease()
        request.header.lease.id = 0
        request.header.policy = RequestPolicy()
        request.header.policy.priority = 0
        request.header.policy.noreply = False
        request.parameter = json.dumps({"data": data}, ensure_ascii=False)
        request.binary = []
        with self._native_slam_response_cv:
            self._native_slam_responses.pop(request_id, None)
        self.native_slam_publisher.publish(request)
        response: dict[str, Any] | None = None
        if Response is not None:
            timeout_sec = max(0.5, float(self.config.native_slam.response_timeout_sec))
            with self._native_slam_response_cv:
                ready = self._native_slam_response_cv.wait_for(
                    lambda: request_id in self._native_slam_responses,
                    timeout=timeout_sec,
                )
                if not ready:
                    raise RosBridgeError(
                        f"等待 Unitree SLAM 响应超时: api_id={api_id}, request_id={request_id}"
                    )
                response = self._native_slam_responses.pop(request_id)
            if int(response.get("code", -1)) != 0:
                raise RosBridgeError(
                    f"Unitree SLAM 命令失败: api_id={api_id}, code={response.get('code')}, data={response.get('data')}"
                )
        return {
            "api_id": int(api_id),
            "request_id": request_id,
            "payload": data,
            "request_topic": self.config.native_slam.request_topic,
            "response": response,
        }

    def start_native_mapping(self) -> dict[str, Any]:
        published = self._publish_native_slam_request(
            1801,
            {"slam_type": self.config.native_slam.mapping_type},
        )
        published["message"] = "已发送 Unitree 原生 SLAM 开始建图命令"
        return published

    def request_native_map_save(self, map_id: str) -> dict[str, Any]:
        safe_map_id = map_id.strip() or f"map_{int(time.time())}"
        filename = safe_map_id if safe_map_id.endswith(".pcd") else f"{safe_map_id}.pcd"
        save_root = Path(self.config.native_slam.save_root).expanduser()
        target_path = save_root / filename
        published = self._publish_native_slam_request(
            1802,
            {"address": str(target_path)},
        )
        published["path"] = str(target_path)
        published["message"] = "已发送 Unitree 原生 SLAM 保存地图命令"
        return published

    def request_native_initial_pose(self, pose: NavigationGoal, map_path: str) -> dict[str, Any]:
        published = self._publish_native_slam_request(
            1804,
            {
                "x": float(pose.x),
                "y": float(pose.y),
                "z": 0.0,
                "q_x": 0.0,
                "q_y": 0.0,
                "q_z": math.sin(float(pose.yaw) / 2.0),
                "q_w": math.cos(float(pose.yaw) / 2.0),
                "address": str(map_path),
            },
        )
        published["map_path"] = str(map_path)
        published["message"] = "已发送 Unitree 原生 3D 初始位姿"
        return published

    def save_managed_map(self, map_id: str) -> dict[str, Any]:
        if self.manage_map_client is None or ManageMap is None:
            raise RosBridgeError("manage_map service client 不可用")
        if not self.manage_map_client.wait_for_service(timeout_sec=2.0):
            raise RosBridgeError("manage_map service 不可用")

        request = ManageMap.Request()
        request.command = "save"
        request.map_id = map_id.strip()
        future = self.manage_map_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=8.0):
            raise RosBridgeError("manage_map save 超时")
        response = future.result()
        if response is None:
            raise RosBridgeError("manage_map save 未返回结果")
        if not response.success:
            raise RosBridgeError(response.message or "manage_map save 失败")
        return {
            "message": response.message,
            "map_ids": list(response.map_ids),
        }

    def _call_task_command(
        self,
        *,
        command: str,
        map_id: str = "",
        route_id: str = "",
        mode: str = "",
        mission_name: str = "",
        route_yaml: str = "",
        waypoints_file: str = "",
        dry_run: bool = False,
        stop_on_failure: bool = True,
        save_map_on_finish: bool = False,
        save_map_on_failure: bool = False,
        pose: PoseStamped | None = None,
    ) -> Any:
        if self.task_command_client is None or NavCommand is None:
            raise RosBridgeError("task_manager service client 不可用")
        if not self.task_command_client.wait_for_service(timeout_sec=2.0):
            raise RosBridgeError("task_manager service 不可用")

        request = NavCommand.Request()
        request.command = command
        request.map_id = map_id
        request.route_id = route_id
        request.mode = mode
        request.mission_name = mission_name
        request.route_yaml = route_yaml
        request.waypoints_file = waypoints_file
        request.dry_run = bool(dry_run)
        request.stop_on_failure = bool(stop_on_failure)
        request.save_map_on_finish = bool(save_map_on_finish)
        request.save_map_on_failure = bool(save_map_on_failure)
        if pose is not None:
            request.pose = pose

        future = self.task_command_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=8.0):
            raise RosBridgeError(f"task_manager {command} 超时")
        response = future.result()
        if response is None:
            raise RosBridgeError(f"task_manager {command} 未返回结果")
        if not response.success:
            raise RosBridgeError(response.message or f"task_manager {command} 失败")
        return response

    def _task_route_status_from_text_status(self) -> TaskRouteStatus:
        with self._lock:
            status = deep_copy_model(self.status.task_manager_status)

        def normalize_optional(value: str | None) -> str | None:
            normalized = (value or "").strip()
            if not normalized or normalized.lower() == "none":
                return None
            return normalized

        fields = dict(status.fields)
        return TaskRouteStatus(
            raw=status.raw,
            ready=status.ready,
            state=status.state,
            reason=status.reason,
            current_mode=normalize_optional(fields.get("current_mode")),
            active_map=normalize_optional(fields.get("active_map")),
            route_state=normalize_optional(fields.get("route_state")),
            route_id=normalize_optional(fields.get("route_id")),
            route_path=normalize_optional(fields.get("route_path")),
            report_path=normalize_optional(fields.get("report_path")),
            fields=fields,
        )

    def task_list_routes(self) -> list[str]:
        response = self._call_task_command(command="list_routes")
        return list(response.items)

    def task_get_route(self, route_id: str) -> dict[str, Any]:
        response = self._call_task_command(command="get_route", route_id=route_id.strip())
        return {
            "route_id": response.route_id,
            "route_path": response.route_path,
            "route_yaml": response.route_yaml,
        }

    def task_save_route(self, route_id: str, route_yaml: str) -> dict[str, Any]:
        response = self._call_task_command(
            command="save_route",
            route_id=route_id.strip(),
            route_yaml=route_yaml,
        )
        return {
            "route_id": response.route_id,
            "route_path": response.route_path,
            "route_yaml": response.route_yaml,
            "items": list(response.items),
        }

    def task_delete_route(self, route_id: str) -> list[str]:
        response = self._call_task_command(command="delete_route", route_id=route_id.strip())
        return list(response.items)

    def task_run_route(
        self,
        *,
        route_id: str,
        mission_name: str = "",
        dry_run: bool = False,
        stop_on_failure: bool = True,
        save_map_on_finish: bool = False,
        save_map_on_failure: bool = False,
    ) -> dict[str, Any]:
        response = self._call_task_command(
            command="run_route",
            route_id=route_id.strip(),
            mission_name=mission_name,
            dry_run=dry_run,
            stop_on_failure=stop_on_failure,
            save_map_on_finish=save_map_on_finish,
            save_map_on_failure=save_map_on_failure,
        )
        return {
            "route_id": response.route_id,
            "route_path": response.route_path,
            "mission_state": response.mission_state,
            "message": response.message,
        }

    def task_stop_route(self) -> TaskRouteStatus:
        self._call_task_command(command="stop_route")
        return self.task_route_status()

    def task_route_status(self) -> TaskRouteStatus:
        service_response = self._call_task_command(command="route_status")
        status = self._task_route_status_from_text_status()
        if not status.current_mode and service_response.current_mode:
            status.current_mode = service_response.current_mode
        if not status.active_map and service_response.active_map:
            status.active_map = service_response.active_map
        if not status.route_id and service_response.route_id:
            status.route_id = service_response.route_id
        if not status.route_path and service_response.route_path:
            status.route_path = service_response.route_path
        if not status.report_path and service_response.report_path:
            status.report_path = service_response.report_path
        if not status.route_state and service_response.mission_state:
            status.route_state = service_response.mission_state
        return status

    def _on_map(self, msg: OccupancyGrid) -> None:
        orientation = msg.info.origin.orientation
        map_snapshot = MapSnapshot(
            loaded=True,
            representation="occupancy_grid_2d",
            frame_id=msg.header.frame_id,
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin=Pose2D(
                x=msg.info.origin.position.x,
                y=msg.info.origin.position.y,
                yaw=quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w),
            ),
            stamp=now_iso(),
            data=list(msg.data),
        )
        with self._lock:
            self.map_snapshot = map_snapshot
            self.health.map_received = True
            self.health.last_map_update = map_snapshot.stamp
        self._publish("map", dump_model(map_snapshot))
        self._publish("health", self.get_health_dict())

    def _on_pointcloud(self, msg: PointCloud2, topic: str, *, primary: bool) -> None:
        try:
            pointcloud_snapshot = self._pointcloud_snapshot_from_msg(msg, topic)
        except Exception as exc:
            self._set_health_error(f"点云解析失败: {exc}")
            return
        now = time.monotonic()
        should_publish = False
        with self._lock:
            if primary:
                self._last_primary_pointcloud_monotonic = now
                self.pointcloud_snapshot = pointcloud_snapshot
                should_publish = True
            else:
                self._last_fallback_pointcloud_monotonic = now
                if self._should_use_fallback_pointcloud(now):
                    self.pointcloud_snapshot = pointcloud_snapshot
                    should_publish = True
            if not self.map_snapshot.loaded and pointcloud_snapshot.loaded:
                self.health.map_received = True
                self.health.last_map_update = pointcloud_snapshot.stamp
        if should_publish:
            self._publish_current_pointcloud_snapshot()

    def _should_use_fallback_pointcloud(self, now: float | None = None) -> bool:
        del now
        if self._last_primary_pointcloud_monotonic <= 0.0:
            return True
        if not self.pointcloud_snapshot.loaded:
            return True
        # The primary 3D map topic is keyframe-driven and may be intentionally
        # quiet between map updates. Keep showing the latest accumulated map
        # instead of replacing it with the raw live lidar fallback.
        return False

    def _publish_current_pointcloud_snapshot(self) -> None:
        self._publish("pointcloud", dump_model(self.pointcloud_snapshot))
        self._publish("health", self.get_health_dict())

    def _on_localization_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        robot_pose = RobotPose(
            available=True,
            source=self.config.ros.localization_pose_topic,
            frame_id=msg.header.frame_id,
            stamp=now_iso(),
            x=pose.position.x,
            y=pose.position.y,
            yaw=quaternion_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
            stale=False,
        )
        with self._lock:
            self.pose = robot_pose
            self.health.pose_received = True
            self.health.last_pose_update = robot_pose.stamp
            self._last_pose_monotonic = time.monotonic()
        self._publish("pose", dump_model(robot_pose))
        self._publish("health", self.get_health_dict())

    def _on_localization_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        robot_pose = RobotPose(
            available=True,
            source=self.config.ros.localization_pose_topic,
            frame_id=msg.header.frame_id,
            stamp=now_iso(),
            x=pose.position.x,
            y=pose.position.y,
            yaw=quaternion_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
            stale=False,
        )
        with self._lock:
            self.pose = robot_pose
            self.health.pose_received = True
            self.health.last_pose_update = robot_pose.stamp
            self._last_pose_monotonic = time.monotonic()
        self._publish("pose", dump_model(robot_pose))
        self._publish("health", self.get_health_dict())

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self.status.velocity_linear_x = msg.twist.twist.linear.x
            self.status.velocity_angular_z = msg.twist.twist.angular.z
        self._publish("status", dump_model(self.status))

    def _on_tf(self, msg: TFMessage) -> None:
        if not msg.transforms:
            return
        transform = msg.transforms[-1]
        with self._lock:
            self._last_tf_frame = transform.child_frame_id

    def _on_real_report(self, msg: String) -> None:
        parsed = self._status_from_string(msg.data)
        with self._lock:
            self.status.real_report = parsed
            self.status.system_ready = parsed.ready
        self._publish("status", dump_model(self.status))

    def _on_lidar_status(self, msg: String) -> None:
        with self._lock:
            self.status.lidar_status = self._status_from_string(msg.data)
        self._publish("status", dump_model(self.status))

    def _on_camera_status(self, msg: String) -> None:
        with self._lock:
            self.status.camera_status = self._status_from_string(msg.data)
        self._publish("status", dump_model(self.status))

    def _on_localization_ok(self, msg: Bool) -> None:
        with self._lock:
            self.status.localization_ok = msg.data
        self._publish("status", dump_model(self.status))

    def _on_localization_status(self, msg: String) -> None:
        with self._lock:
            self.status.localization_status = self._status_from_string(msg.data)
        self._publish("status", dump_model(self.status))

    def _on_relocalization_status(self, msg: String) -> None:
        """Parse NDT score from relocalization status string.

        Format: state=...;ready=...;score=X.XXX;...
        """
        parsed, _fields = parse_status_string(msg.data)
        score_val = None
        healthy_val = None
        try:
            score_str = _fields.get("score", "")
            if score_str:
                score_val = float(score_str)
        except (ValueError, TypeError):
            pass
        ready_str = _fields.get("ready", "false")
        healthy_val = ready_str.lower() == "true"
        with self._lock:
            self.status.relocalization_status = self._status_from_string(msg.data)
            self.status.ndt_score = score_val
            self.status.ndt_healthy = healthy_val
        self._publish("status", dump_model(self.status))

    def _on_safety_status(self, msg: String) -> None:
        with self._lock:
            self.status.safety_status = self._status_from_string(msg.data)
        self._publish("status", dump_model(self.status))

    def _on_map_manager_status(self, msg: String) -> None:
        with self._lock:
            self.status.map_manager_status = self._status_from_string(msg.data)
        self._publish("status", dump_model(self.status))

    def _on_active_map(self, msg: String) -> None:
        with self._lock:
            self.status.active_map = msg.data or None
        self._publish("status", dump_model(self.status))

    def _on_task_manager_status(self, msg: String) -> None:
        with self._lock:
            self.status.task_manager_status = self._status_from_string(msg.data)
        self._publish("status", dump_model(self.status))

    def _on_pose_goal_status(self, msg: String) -> None:
        status = self._status_from_string(msg.data)
        state = status.state or ""
        reason = status.reason or state or "unknown"
        feedback: dict[str, Any] = {}
        for key in ("distance", "yaw_error", "vx", "vy", "wz"):
            value = status.fields.get(key)
            if value is None:
                continue
            try:
                feedback[key] = float(value)
            except ValueError:
                feedback[key] = value
        if "distance" not in feedback and reason.startswith("distance="):
            try:
                feedback["distance"] = float(reason.split("=", 1)[1])
            except ValueError:
                pass
        if "distance" in feedback:
            feedback["distance_remaining"] = feedback["distance"]
        if status.raw:
            feedback["controller_status"] = status.raw

        with self._lock:
            if self.config.navigation.backend != "pose_topic_3d":
                return
            self.navigation.action_server_ready = True
            self.navigation.backend = "pose_topic_3d"
            self.navigation.feedback = feedback
            self.navigation.updated_at = now_iso()
            if state in {"goal_active", "running", "blocked"}:
                self.navigation.state = "navigating"
                self.navigation.message = f"3D 控制器: {state} / {reason}"
            elif state == "goal_rejected":
                self._active_pose_goal = None
                self._active_pose_goal_started_at = None
                self.navigation.state = "failed"
                self.navigation.message = f"3D 位姿目标被控制器拒绝: {reason}"
            elif state == "goal_timeout":
                self._active_pose_goal = None
                self._active_pose_goal_started_at = None
                self.navigation.state = "failed"
                self.navigation.message = f"3D 位姿目标超时: {reason}"
            elif state == "goal_reached":
                active_goal_distance = None
                if (
                    self._active_pose_goal is not None
                    and self.pose.available
                    and self.pose.x is not None
                    and self.pose.y is not None
                ):
                    active_goal_distance = math.hypot(
                        float(self.pose.x) - self._active_pose_goal.x,
                        float(self.pose.y) - self._active_pose_goal.y,
                    )
                reached_guard_m = min(float(self.config.navigation.pose_goal_tolerance_m), 0.20)
                if active_goal_distance is not None and active_goal_distance > reached_guard_m:
                    feedback["active_goal_distance"] = active_goal_distance
                    feedback["ignored_controller_status"] = status.raw
                    self.navigation.state = "navigating"
                    self.navigation.message = (
                        f"忽略过期到达状态: active_goal_distance={active_goal_distance:.3f}m"
                    )
                    self.navigation.feedback = feedback
                    self.navigation.updated_at = now_iso()
                    self._publish("navigation", dump_model(self.navigation))
                    return
                self._active_pose_goal = None
                self._active_pose_goal_started_at = None
                self.navigation.state = "succeeded"
                self.navigation.message = f"3D 位姿目标已到达: {reason}"
            elif state == "idle" and self._active_pose_goal is None:
                self.navigation.state = "idle"
                self.navigation.message = "3D 控制器等待目标"
        self._publish("navigation", dump_model(self.navigation))

    def _on_sdk_status(self, msg: String) -> None:
        with self._lock:
            self.status.sdk_status = self._status_from_string(msg.data)
        self._publish("status", dump_model(self.status))

    def _on_raw_state(self, msg: RobotState) -> None:
        raw_state = RawStateSummary(
            source_mode=msg.source_mode,
            frame_id=msg.frame_id,
            connected=msg.connected,
            imu_valid=msg.imu_valid,
            odom_valid=msg.odom_valid,
            position=list(msg.position),
            velocity=list(msg.velocity),
            rpy=list(msg.rpy),
            linear_acceleration=list(msg.linear_acceleration),
            angular_velocity=list(msg.angular_velocity),
            body_height=msg.body_height,
            yaw_speed=msg.yaw_speed,
            motion_mode=int(msg.motion_mode),
            gait_type=int(msg.gait_type),
            progress=msg.progress,
        )
        with self._lock:
            self.status.raw_state = raw_state
        self._publish("status", dump_model(self.status))

    def _on_battery(self, msg: BatteryState) -> None:
        stamp = now_iso()
        try:
            stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            stamp = datetime.fromtimestamp(stamp_sec).isoformat()
        except Exception:
            stamp = now_iso()

        available = bool(getattr(msg, "present", False))
        pct: float | None = None
        voltage: float | None = None
        charging: bool | None = None
        health: int | None = None
        if available:
            pct = _battery_percent_0_100(getattr(msg, "percentage", None))
            raw_voltage = float(getattr(msg, "voltage", float("nan")))
            if math.isfinite(raw_voltage) and raw_voltage > 0.0:
                voltage = raw_voltage
            try:
                charging = msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
            except Exception:
                charging = None
        try:
            health = int(getattr(msg, "power_supply_health", 0))
        except Exception:
            health = None
        with self._lock:
            self.battery.available = available
            self.battery.percentage = pct
            self.battery.voltage = voltage
            self.battery.charging = charging
            self.battery.health = health
            self.battery.stamp = stamp
            self.battery.stale = False
            if available and pct is not None:
                self._last_battery_monotonic = time.monotonic()
            elif time.monotonic() - self._last_battery_warn_monotonic > 5.0:
                self._last_battery_warn_monotonic = time.monotonic()
                self.get_logger().warning(
                    f"battery update incomplete: available={str(available).lower()} percentage={str(pct)} voltage={str(voltage)} charging={str(charging)} raw_percentage={str(getattr(msg, 'percentage', None))} raw_voltage={str(getattr(msg, 'voltage', None))} status={str(int(getattr(msg, 'power_supply_status', 0) or 0))} topic={str(self.config.ros.battery_topic)}"
                )
        self._publish("battery", dump_model(self.battery))

    def _on_scan_mission_status(self, msg: String) -> None:
        """Parse recovery_* fields from scan mission status."""
        _parsed, fields = parse_status_string(msg.data)
        rec = RecoveryStatus()
        rec.raw = msg.data
        rec.active = fields.get("state", "") == "recovery_active"
        rec.step = fields.get("reason", None)
        if rec.step and rec.step.startswith("step="):
            rec.step = rec.step.split("=", 1)[1] if "=" in rec.step else rec.step
        with self._lock:
            if rec.active:
                if rec.step and rec.step not in self.recovery_status.sequence:
                    self.recovery_status.sequence.append(rec.step)
                self.recovery_status.active = True
                self.recovery_status.step = rec.step
            elif self.recovery_status.active:
                # Recovery just ended
                self.recovery_status.active = False
            self.recovery_status.raw = rec.raw
        self._publish("recovery", dump_model(self.recovery_status))

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        if not self._camera_throttle_ready():
            return
        image_format = (msg.format or "jpeg").lower()
        mime = "image/png" if "png" in image_format else "image/jpeg"
        data_url = f"data:{mime};base64,{base64.b64encode(bytes(msg.data)).decode('ascii')}"
        frame = CameraFrame(
            available=True,
            topic=self.config.ros.camera_compressed_topic,
            frame_id=msg.header.frame_id or None,
            stamp=now_iso(),
            encoding="compressed",
            format=image_format,
            data_url=data_url,
            stale=False,
        )
        with self._lock:
            self.camera = frame
            self.health.camera_received = True
            self.health.last_camera_update = frame.stamp
        self._publish("camera", dump_model(frame))
        self._publish("health", self.get_health_dict())

    def _on_image(self, msg: Image) -> None:
        if self.config.camera.prefer_compressed and self.camera.available:
            return
        if not self._camera_throttle_ready():
            return
        if PilImage is None:
            self._set_health_error("Pillow 未安装，无法把 raw camera image 转成 JPEG")
            return
        mode = None
        raw = bytes(msg.data)
        encoding = msg.encoding.lower()
        if encoding == "rgb8":
            if msg.step != msg.width * 3:
                self._set_health_error("暂不支持带行填充的 rgb8 camera image")
                return
            mode = "RGB"
        elif encoding == "bgr8":
            if msg.step != msg.width * 3:
                self._set_health_error("暂不支持带行填充的 bgr8 camera image")
                return
            mode = "RGB"
            converted = bytearray(len(raw))
            for index in range(0, len(raw), 3):
                converted[index] = raw[index + 2]
                converted[index + 1] = raw[index + 1]
                converted[index + 2] = raw[index]
            raw = bytes(converted)
        elif encoding == "rgba8":
            if msg.step != msg.width * 4:
                self._set_health_error("暂不支持带行填充的 rgba8 camera image")
                return
            mode = "RGBA"
        elif encoding == "mono8":
            if msg.step != msg.width:
                self._set_health_error("暂不支持带行填充的 mono8 camera image")
                return
            mode = "L"
        else:
            self._set_health_error(f"暂不支持 raw camera encoding: {msg.encoding}")
            return
        try:
            image = PilImage.frombytes(mode, (msg.width, msg.height), raw)
            if image.mode != "RGB":
                image = image.convert("RGB")
            output = io.BytesIO()
            image.save(output, format="JPEG", quality=int(self.config.camera.jpeg_quality))
        except Exception as exc:
            self._set_health_error(f"raw camera image 转换失败: {exc}")
            return
        data_url = f"data:image/jpeg;base64,{base64.b64encode(output.getvalue()).decode('ascii')}"
        frame = CameraFrame(
            available=True,
            topic=self.config.ros.camera_image_topic,
            frame_id=msg.header.frame_id or None,
            stamp=now_iso(),
            width=int(msg.width),
            height=int(msg.height),
            encoding=msg.encoding,
            format="jpeg",
            data_url=data_url,
            stale=False,
        )
        with self._lock:
            self.camera = frame
            self.health.camera_received = True
            self.health.last_camera_update = frame.stamp
        self._publish("camera", dump_model(frame))
        self._publish("health", self.get_health_dict())

    def _publish_health(self) -> None:
        self._update_pose_topic_goal()
        with self._lock:
            if self.config.navigation.backend == "pose_topic_3d":
                self.health.action_server_ready = True
                self.navigation.backend = "pose_topic_3d"
                self.navigation.action_server_ready = True
            else:
                self.health.action_server_ready = bool(self.action_client and self.action_client.server_is_ready())
                self.navigation.backend = "nav2"
                self.navigation.action_server_ready = self.health.action_server_ready
        self._publish("health", self.get_health_dict())
        self._publish("navigation", dump_model(self.navigation))

    def _update_pose_topic_goal(self) -> None:
        with self._lock:
            active_goal = deep_copy_model(self._active_pose_goal)
            started_at = self._active_pose_goal_started_at
            pose = deep_copy_model(self.pose)
        if self.config.navigation.backend != "pose_topic_3d" or active_goal is None:
            return
        if started_at is None:
            started_at = time.monotonic()
        if not pose.available or pose.x is None or pose.y is None:
            with self._lock:
                self.navigation.feedback = {"reason": "pose_unavailable"}
                self.navigation.updated_at = now_iso()
            return

        dx = float(pose.x) - active_goal.x
        dy = float(pose.y) - active_goal.y
        distance = math.hypot(dx, dy)
        yaw_error = None
        if pose.yaw is not None:
            yaw_error = abs(math.atan2(math.sin(float(pose.yaw) - active_goal.yaw), math.cos(float(pose.yaw) - active_goal.yaw)))
        elapsed = time.monotonic() - started_at
        feedback = {
            "distance_remaining": distance,
            "yaw_error_rad": yaw_error,
            "elapsed_sec": elapsed,
            "backend": "pose_topic_3d",
        }
        timed_out = elapsed > self.config.navigation.goal_timeout_sec
        with self._lock:
            self.navigation.feedback = feedback
            self.navigation.updated_at = now_iso()
            if timed_out:
                self._publish_pose_topic_stop("goal_timeout")
                self.navigation.state = "failed"
                self.navigation.message = "3D 位姿目标超时"
                self._active_pose_goal = None
                self._active_pose_goal_started_at = None

    def get_health_dict(self) -> dict[str, Any]:
        with self._lock:
            self.health.websocket_clients = self.ws_manager.client_count
            return dump_model(self.health)

    def _snap_pose(
        self,
        pose: NavigationGoal,
        *,
        clearance_m: float,
        max_radius_m: float,
    ) -> tuple[NavigationGoal, bool]:
        with self._lock:
            map_snapshot = deep_copy_model(self.map_snapshot)
        return _snap_pose_to_free_cell(
            map_snapshot,
            pose,
            clearance_m=clearance_m,
            max_radius_m=max_radius_m,
            occupancy_block_threshold=self.config.navigation.occupancy_block_threshold,
        )

    def build_snapshot(self, *, ros_thread_alive: bool | None = None) -> DashboardSnapshot:
        with self._lock:
            pose = deep_copy_model(self.pose)
            status = deep_copy_model(self.status)
            navigation = deep_copy_model(self.navigation)
            health = deep_copy_model(self.health)
            battery = deep_copy_model(self.battery)
            if ros_thread_alive is not None:
                health.ros_thread_alive = ros_thread_alive
            if not health.ros_thread_alive:
                health.ros_connected = False
                health.action_server_ready = False
                if not health.last_error:
                    health.last_error = "ROS 线程未运行，页面状态可能已过期"
                status.system_ready = False
                status.localization_ok = False
                pose.stale = True
                battery.stale = True
            elif pose.stamp is not None:
                pose_age = time.monotonic() - self._last_pose_monotonic if self._last_pose_monotonic > 0.0 else math.inf
                pose.stale = pose_age > self.config.health.pose_stale_sec
            if health.ros_thread_alive:
                battery_age = time.monotonic() - self._last_battery_monotonic if self._last_battery_monotonic > 0.0 else math.inf
                battery.stale = battery_age > float(self.config.health.battery_stale_sec)
            return DashboardSnapshot(
                map=deep_copy_model(self.map_snapshot),
                pointcloud=deep_copy_model(self.pointcloud_snapshot),
                pose=pose,
                status=status,
                navigation=navigation,
                camera=deep_copy_model(self.camera),
                health=health,
                battery=battery,
                recovery=deep_copy_model(self.recovery_status),
            )

    def get_map_snapshot(self) -> MapSnapshot:
        with self._lock:
            return deep_copy_model(self.map_snapshot)

    def _pointcloud_snapshot_from_msg(self, msg: PointCloud2, source_topic: str) -> PointCloudSnapshot:
        x_field = next((field for field in msg.fields if field.name == "x"), None)
        y_field = next((field for field in msg.fields if field.name == "y"), None)
        z_field = next((field for field in msg.fields if field.name == "z"), None)
        if x_field is None or y_field is None or z_field is None:
            raise RosBridgeError("pointcloud missing x/y/z fields")
        if x_field.datatype != 7 or y_field.datatype != 7 or z_field.datatype != 7:
            raise RosBridgeError("only FLOAT32 x/y/z pointclouds are supported")

        total_points = int(msg.width) * int(msg.height)
        if total_points <= 0:
            return PointCloudSnapshot(
                loaded=False,
                source_topic=source_topic,
                frame_id=msg.header.frame_id or None,
                stamp=now_iso(),
                points=[],
                points_total=0,
                points_sampled=0,
                sample_stride=1,
            )

        max_points = max(1000, int(self.config.ros.pointcloud_preview_max_points))
        sample_stride = max(1, int(math.ceil(total_points / max_points)))
        endian = ">" if msg.is_bigendian else "<"
        unpack_float = struct.Struct(f"{endian}f").unpack_from
        raw = memoryview(msg.data)
        points: list[list[float]] = []
        for point_index in range(0, total_points, sample_stride):
            base = point_index * msg.point_step
            x = unpack_float(raw, base + x_field.offset)[0]
            y = unpack_float(raw, base + y_field.offset)[0]
            z = unpack_float(raw, base + z_field.offset)[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            points.append([float(x), float(y), float(z)])

        return PointCloudSnapshot(
            loaded=bool(points),
            frame_id=msg.header.frame_id or None,
            stamp=now_iso(),
            source_topic=source_topic,
            points=points,
            points_total=total_points,
            points_sampled=len(points),
            sample_stride=sample_stride,
        )

    def send_navigation_goal(self, request: NavigationGoalRequest) -> NavigationTaskState:
        with self._navigation_lock:
            if not self.config.navigation.allow_send_goal:
                raise RosBridgeError("导航发送在配置中被禁用")
            if self.config.navigation.require_localization_ready and self.status.localization_ok is not True:
                raise RosBridgeError("定位未就绪，禁止发送导航目标")
            if self.config.navigation.require_map_for_goal and not self.map_snapshot.loaded:
                raise RosBridgeError("地图尚未加载完成")
            if self._active_goal_handle is not None or self._active_pose_goal is not None:
                raise RosBridgeError("已有导航任务正在执行")
            if self.config.navigation.backend == "pose_topic_3d":
                return self._send_pose_topic_goal(request.goal)
            if self.action_client is None or NavigateToPose is None:
                raise RosBridgeError("NavigateToPose action client 不可用")
            if not self.action_client.wait_for_server(timeout_sec=self.config.navigation.action_wait_timeout_sec):
                raise RosBridgeError("NavigateToPose action server 不可用")

            snapped_goal, goal_snapped = self._snap_pose(
                request.goal,
                clearance_m=self.config.navigation.goal_clearance_m,
                max_radius_m=self.config.navigation.goal_snap_radius_m,
            )
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = snapped_goal.frame_id
            # A zero stamp asks Nav2 to use the latest transform. Reusing a
            # wall-clock stamp from the web process can trigger past
            # extrapolation when map->odom is freshly published.
            goal.pose.header.stamp.sec = 0
            goal.pose.header.stamp.nanosec = 0
            goal.pose.pose.position.x = snapped_goal.x
            goal.pose.pose.position.y = snapped_goal.y
            goal.pose.pose.orientation.z = math.sin(snapped_goal.yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(snapped_goal.yaw / 2.0)

            wait_event = threading.Event()
            result_box: dict[str, Any] = {}

            def feedback_callback(feedback_msg: NavigateToPose.FeedbackMessage) -> None:
                feedback = feedback_msg.feedback
                feedback_payload = {
                    "distance_remaining": getattr(feedback, "distance_remaining", None),
                    "estimated_time_remaining_sec": getattr(
                        getattr(feedback, "estimated_time_remaining", None), "sec", None
                    ),
                    "navigation_time_sec": getattr(getattr(feedback, "navigation_time", None), "sec", None),
                    "number_of_recoveries": getattr(feedback, "number_of_recoveries", None),
                }
                with self._lock:
                    self.navigation.feedback = feedback_payload
                    self.navigation.updated_at = now_iso()
                self._publish("navigation", dump_model(self.navigation))

            future = self.action_client.send_goal_async(goal, feedback_callback=feedback_callback)

            def goal_response_callback(goal_future: Any) -> None:
                try:
                    goal_handle = goal_future.result()
                    if goal_handle is None or not goal_handle.accepted:
                        result_box["error"] = "导航目标被 action server 拒绝"
                        return
                    self._active_goal_handle = goal_handle
                    accepted_message = "导航目标已接受"
                    if goal_snapped:
                        accepted_message = "导航目标已接受，已吸附到最近可行点"
                    with self._lock:
                        self.navigation = NavigationTaskState(
                            state="navigating",
                            message=accepted_message,
                            action_server_ready=True,
                            goal=snapped_goal,
                            feedback={},
                            updated_at=now_iso(),
                        )
                    self._publish("navigation", dump_model(self.navigation))
                    result_future = goal_handle.get_result_async()
                    result_future.add_done_callback(self._on_navigation_result)
                    result_box["ok"] = True
                except Exception as exc:  # pragma: no cover - defensive path
                    result_box["error"] = f"发送导航目标失败: {exc}"
                finally:
                    wait_event.set()

            future.add_done_callback(goal_response_callback)
            if not wait_event.wait(self.config.navigation.goal_response_timeout_sec):
                raise RosBridgeError("等待导航目标响应超时")
            if "error" in result_box:
                raise RosBridgeError(str(result_box["error"]))
            return deep_copy_model(self.navigation)

    def _send_pose_topic_goal(self, requested_goal: NavigationGoal) -> NavigationTaskState:
        goal = requested_goal.model_copy(update={"frame_id": self.config.navigation.goal_frame})
        msg = PoseStamped()
        msg.header.frame_id = self.config.navigation.goal_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = goal.x
        msg.pose.position.y = goal.y
        msg.pose.orientation.z = math.sin(goal.yaw / 2.0)
        msg.pose.orientation.w = math.cos(goal.yaw / 2.0)
        self.pose_goal_publisher.publish(msg)
        with self._lock:
            self._active_pose_goal = goal
            self._active_pose_goal_started_at = time.monotonic()
            self.navigation = NavigationTaskState(
                state="navigating",
                message=f"3D 位姿目标已发布到 {self.config.navigation.goal_topic}",
                backend="pose_topic_3d",
                action_server_ready=True,
                goal=goal,
                feedback={},
                updated_at=now_iso(),
            )
        self._publish("navigation", dump_model(self.navigation))
        return deep_copy_model(self.navigation)

    def _publish_pose_topic_stop(self, reason: str) -> None:
        if self.config.navigation.cancel_retarget_current_pose:
            pose = deep_copy_model(self.pose)
            if pose.available and pose.x is not None and pose.y is not None:
                stop_goal = PoseStamped()
                stop_goal.header.frame_id = self.config.navigation.goal_frame
                stop_goal.header.stamp = self.get_clock().now().to_msg()
                stop_goal.pose.position.x = float(pose.x)
                stop_goal.pose.position.y = float(pose.y)
                yaw = float(pose.yaw or 0.0)
                stop_goal.pose.orientation.z = math.sin(yaw / 2.0)
                stop_goal.pose.orientation.w = math.cos(yaw / 2.0)
                self.pose_goal_publisher.publish(stop_goal)
        stop = Twist()
        burst_count = max(1, int(self.config.navigation.cancel_stop_burst_count))
        interval = max(0.0, float(self.config.navigation.cancel_stop_burst_interval_sec))
        for _ in range(burst_count):
            self.cancel_stop_publisher.publish(stop)
            if interval > 0.0:
                time.sleep(interval)
        self.get_logger().info(
            "Published 3D pose-topic stop. "
            f"reason={reason} goal_topic={self.config.navigation.goal_topic} "
            f"stop_topic={self.config.navigation.cancel_stop_topic} burst={burst_count}"
        )

    def set_initial_pose(self, request: InitialPoseRequest) -> dict[str, Any]:
        with self._navigation_lock:
            uses_pose_topic_3d = self.config.navigation.backend == "pose_topic_3d"
            if not uses_pose_topic_3d and not self.map_snapshot.loaded:
                raise RosBridgeError("地图尚未加载完成")

            if uses_pose_topic_3d:
                snapped_pose = request.pose.model_copy(update={"frame_id": self.config.navigation.goal_frame})
                pose_snapped = False
            else:
                snapped_pose, pose_snapped = self._snap_pose(
                    request.pose,
                    clearance_m=self.config.navigation.initial_pose_clearance_m,
                    max_radius_m=self.config.navigation.initial_pose_snap_radius_m,
                )

            covariance = [0.0] * 36
            covariance[0] = float(self.config.navigation.initial_pose_covariance_xy)
            covariance[7] = float(self.config.navigation.initial_pose_covariance_xy)
            covariance[35] = float(self.config.navigation.initial_pose_covariance_yaw)

            previous_pose_stamp = self.pose.stamp
            deadline = time.monotonic() + self.config.navigation.initial_pose_wait_timeout_sec
            publish_interval = max(0.1, self.config.navigation.initial_pose_publish_interval_sec)
            attempts = 0

            while True:
                if attempts == 0 or not uses_pose_topic_3d:
                    msg = PoseWithCovarianceStamped()
                    msg.header.frame_id = snapped_pose.frame_id
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.pose.pose.position.x = snapped_pose.x
                    msg.pose.pose.position.y = snapped_pose.y
                    msg.pose.pose.orientation.z = math.sin(snapped_pose.yaw / 2.0)
                    msg.pose.pose.orientation.w = math.cos(snapped_pose.yaw / 2.0)
                    msg.pose.covariance = covariance
                    self.initial_pose_publisher.publish(msg)
                    attempts += 1

                ready, localization_status = self._initial_pose_ready(previous_pose_stamp)
                if ready:
                    break
                if time.monotonic() >= deadline:
                    detail = localization_status.reason or localization_status.state or "unknown"
                    raise RosBridgeError(
                        f"初始位姿已发送，但定位未进入 ready: {detail}"
                    )
                time.sleep(publish_interval)

            message = "初始位姿已发送，定位已就绪"
            if uses_pose_topic_3d:
                message = "3D 重定位初始位姿已发送，定位已就绪"
            elif pose_snapped:
                message = "初始位姿已发送，已吸附到最近可行点，定位已就绪"
            return {
                "pose": dump_model(snapped_pose),
                "snapped": pose_snapped,
                "attempts": attempts,
                "message": message,
            }

    def set_light(
        self,
        *,
        device_id: str,
        on: bool,
        intensity: int,
        color_mode: int,
        r: int,
        g: int,
        b: int,
        color_temperature_kelvin: int,
    ) -> None:
        if self.light_command_publisher is None or LightCommand is None:
            raise RosBridgeError("灯光控制未启用")
        msg = LightCommand()
        msg.device_id = device_id
        msg.on = bool(on)
        msg.intensity = max(0, min(255, int(intensity)))
        msg.color_mode = max(0, min(255, int(color_mode)))
        msg.r = max(0, min(255, int(r)))
        msg.g = max(0, min(255, int(g)))
        msg.b = max(0, min(255, int(b)))
        msg.color_temperature_kelvin = max(0, min(65535, int(color_temperature_kelvin)))
        msg.stamp = self.get_clock().now().to_msg()
        self.light_command_publisher.publish(msg)

    def _initial_pose_ready(self, previous_pose_stamp: str | None) -> tuple[bool, TextStatus]:
        with self._lock:
            pose = deep_copy_model(self.pose)
            localization_status = deep_copy_model(self.status.localization_status)
            localization_ok = bool(self.status.localization_ok)

        pose_updated = pose.available and pose.stamp is not None and pose.stamp != previous_pose_stamp
        ready = (localization_ok or localization_status.ready is True) and pose_updated
        return ready, localization_status

    def _on_navigation_result(self, result_future: Any) -> None:
        state = "failed"
        message = "未知导航结果"
        try:
            result = result_future.result()
            status = result.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                state = "succeeded"
                message = "导航成功到达目标点"
            elif status == GoalStatus.STATUS_CANCELED:
                state = "canceled"
                message = "导航任务已取消"
            elif status == GoalStatus.STATUS_ABORTED:
                state = "failed"
                message = "导航任务被中止"
            else:
                state = "failed"
                message = f"导航返回状态码 {status}"
        except Exception as exc:  # pragma: no cover - defensive path
            state = "failed"
            message = f"获取导航结果失败: {exc}"
        with self._lock:
            self.navigation.state = state
            self.navigation.message = message
            self.navigation.updated_at = now_iso()
            self._active_goal_handle = None
        self._publish("navigation", dump_model(self.navigation))

    def cancel_navigation(self) -> NavigationTaskState:
        with self._navigation_lock:
            if self.config.navigation.backend == "pose_topic_3d":
                if self._active_pose_goal is None:
                    self._publish_pose_topic_stop("cancel_without_active_goal")
                    with self._lock:
                        self.navigation.state = "idle"
                        self.navigation.message = "当前没有活动 3D 位姿目标，已发布停止信号"
                        self.navigation.updated_at = now_iso()
                    self._publish("navigation", dump_model(self.navigation))
                    return deep_copy_model(self.navigation)
                self._publish_pose_topic_stop("cancel_requested")
                with self._lock:
                    self._active_pose_goal = None
                    self._active_pose_goal_started_at = None
                    self.navigation.state = "canceled"
                    self.navigation.message = "3D 位姿目标已取消，并已发布停止信号"
                    self.navigation.updated_at = now_iso()
                self._publish("navigation", dump_model(self.navigation))
                return deep_copy_model(self.navigation)
            if self._active_goal_handle is None:
                with self._lock:
                    self.navigation.state = "idle"
                    self.navigation.message = "当前没有活动导航任务"
                    self.navigation.updated_at = now_iso()
                self._publish("navigation", dump_model(self.navigation))
                return deep_copy_model(self.navigation)

            wait_event = threading.Event()
            result_box: dict[str, Any] = {}
            cancel_future = self._active_goal_handle.cancel_goal_async()

            def cancel_callback(future: Any) -> None:
                try:
                    cancel_response = future.result()
                    if getattr(cancel_response, "goals_canceling", []):
                        with self._lock:
                            self.navigation.state = "canceled"
                            self.navigation.message = "已请求取消导航任务"
                            self.navigation.updated_at = now_iso()
                        result_box["ok"] = True
                    else:
                        result_box["error"] = "导航取消请求未被接受"
                except Exception as exc:  # pragma: no cover - defensive path
                    result_box["error"] = f"取消导航失败: {exc}"
                finally:
                    wait_event.set()

            cancel_future.add_done_callback(cancel_callback)
            if not wait_event.wait(self.config.navigation.cancel_timeout_sec):
                raise RosBridgeError("等待取消导航响应超时")
            if "error" in result_box:
                raise RosBridgeError(str(result_box["error"]))
            self._publish("navigation", dump_model(self.navigation))
            return deep_copy_model(self.navigation)


class RosRuntime:
    def __init__(self, config: AppConfig, ws_manager: WebSocketManager) -> None:
        self.config = config
        self.ws_manager = ws_manager
        self.node: RosBridgeNode | None = None
        self.executor: MultiThreadedExecutor | None = None
        self.thread: threading.Thread | None = None
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        rclpy.init(args=None)
        self.node = RosBridgeNode(self.config, self.ws_manager)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)

        def run_executor() -> None:
            failure_message: str | None = None
            try:
                assert self.executor is not None
                self.executor.spin()
            except Exception as exc:  # pragma: no cover - defensive runtime guard
                failure_message = f"ROS 线程异常退出: {exc}"
            finally:
                if self.node is not None:
                    with self.node._lock:
                        self.node.health.ros_thread_alive = False
                        self.node.health.ros_connected = False
                        self.node.health.action_server_ready = False
                        if failure_message:
                            self.node.health.last_error = failure_message
                    self.node._publish("health", self.node.get_health_dict())

        self.thread = threading.Thread(target=run_executor, name="ros2-web-console", daemon=True)
        self.thread.start()
        with self.node._lock:
            self.node.health.ros_thread_alive = True
        self._started = True

    def stop(self) -> None:
        if not self._started:
            return
        if self.node is not None:
            with self.node._lock:
                self.node.health.ros_thread_alive = False
        if self.executor is not None:
            self.executor.shutdown()
        if self.node is not None:
            self.node.destroy_node()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        rclpy.shutdown()
        self._started = False
