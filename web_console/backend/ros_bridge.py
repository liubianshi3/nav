from __future__ import annotations

import threading
import math
import time
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

try:
    from a2_interfaces.msg import RobotState
except ImportError:  # pragma: no cover - runtime environment fallback
    RobotState = None

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover - runtime environment fallback
    NavigateToPose = None

from .config import AppConfig
from .models import (
    DashboardSnapshot,
    InitialPoseRequest,
    MapSnapshot,
    NavigationGoal,
    NavigationGoalRequest,
    NavigationTaskState,
    Pose2D,
    RawStateSummary,
    RobotPose,
    RobotStatus,
    SystemHealth,
    TextStatus,
)
from .utils import deep_copy_model, dump_model, now_iso, parse_optional_bool, parse_status_string, quaternion_to_yaw
from .ws import WebSocketManager


class RosBridgeError(RuntimeError):
    """Raised when the backend cannot execute a ROS-side command."""


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
        self.pose = RobotPose()
        self.status = RobotStatus()
        self.navigation = NavigationTaskState(updated_at=now_iso())
        self.health = SystemHealth(ros_connected=True)
        self._last_tf_frame: str | None = None

        self._setup_subscriptions()
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.config.navigation.initial_pose_topic,
            10,
        )
        self.action_client = (
            ActionClient(self, NavigateToPose, self.config.navigation.action_name) if NavigateToPose is not None else None
        )
        if self.action_client is None:
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
        self.create_subscription(PoseWithCovarianceStamped, ros.amcl_pose_topic, self._on_amcl_pose, latched_qos)
        self.create_subscription(PoseWithCovarianceStamped, ros.amcl_pose_topic, self._on_amcl_pose, 10)
        self.create_subscription(Odometry, ros.odom_topic, self._on_odom, 20)
        self.create_subscription(TFMessage, ros.tf_topic, self._on_tf, 20)
        self.create_subscription(String, ros.real_report_topic, self._on_real_report, 10)
        self.create_subscription(String, ros.lidar_status_topic, self._on_lidar_status, 10)
        self.create_subscription(Bool, ros.localization_ok_topic, self._on_localization_ok, 10)
        self.create_subscription(String, ros.localization_status_topic, self._on_localization_status, 10)
        self.create_subscription(String, ros.map_manager_status_topic, self._on_map_manager_status, 10)
        self.create_subscription(String, ros.map_manager_active_map_topic, self._on_active_map, 10)
        self.create_subscription(String, ros.sdk_status_topic, self._on_sdk_status, 10)
        if RobotState is not None:
            self.create_subscription(RobotState, ros.raw_state_topic, self._on_raw_state, 10)
        else:
            self.get_logger().warning("a2_interfaces.msg.RobotState is unavailable. /a2/raw_state will be skipped.")

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

    def _set_health_error(self, message: str) -> None:
        with self._lock:
            self.health.last_error = message
        self._publish("health", self.get_health_dict())

    def _on_map(self, msg: OccupancyGrid) -> None:
        orientation = msg.info.origin.orientation
        map_snapshot = MapSnapshot(
            loaded=True,
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

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        robot_pose = RobotPose(
            available=True,
            source="amcl_pose",
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

    def _on_localization_ok(self, msg: Bool) -> None:
        with self._lock:
            self.status.localization_ok = msg.data
        self._publish("status", dump_model(self.status))

    def _on_localization_status(self, msg: String) -> None:
        with self._lock:
            self.status.localization_status = self._status_from_string(msg.data)
        self._publish("status", dump_model(self.status))

    def _on_map_manager_status(self, msg: String) -> None:
        with self._lock:
            self.status.map_manager_status = self._status_from_string(msg.data)
        self._publish("status", dump_model(self.status))

    def _on_active_map(self, msg: String) -> None:
        with self._lock:
            self.status.active_map = msg.data or None
        self._publish("status", dump_model(self.status))

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

    def _publish_health(self) -> None:
        with self._lock:
            self.health.action_server_ready = bool(self.action_client and self.action_client.server_is_ready())
            self.navigation.action_server_ready = self.health.action_server_ready
        self._publish("health", self.get_health_dict())
        self._publish("navigation", dump_model(self.navigation))

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

    def build_snapshot(self) -> DashboardSnapshot:
        with self._lock:
            pose = deep_copy_model(self.pose)
            if pose.stamp is not None:
                # The frontend treats stale pose conservatively when no fresh update arrives.
                pose.stale = False
            return DashboardSnapshot(
                map=deep_copy_model(self.map_snapshot),
                pose=pose,
                status=deep_copy_model(self.status),
                navigation=deep_copy_model(self.navigation),
                health=deep_copy_model(self.health),
            )

    def get_map_snapshot(self) -> MapSnapshot:
        with self._lock:
            return deep_copy_model(self.map_snapshot)

    def send_navigation_goal(self, request: NavigationGoalRequest) -> NavigationTaskState:
        with self._navigation_lock:
            if not self.config.navigation.allow_send_goal:
                raise RosBridgeError("导航发送在配置中被禁用")
            if self.status.localization_ok is not True:
                raise RosBridgeError("定位未就绪，禁止发送导航目标")
            if not self.map_snapshot.loaded:
                raise RosBridgeError("地图尚未加载完成")
            if self._active_goal_handle is not None:
                raise RosBridgeError("已有导航任务正在执行")
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
            goal.pose.header.stamp = self.get_clock().now().to_msg()
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

    def set_initial_pose(self, request: InitialPoseRequest) -> dict[str, Any]:
        with self._navigation_lock:
            if not self.map_snapshot.loaded:
                raise RosBridgeError("地图尚未加载完成")

            snapped_pose, pose_snapped = self._snap_pose(
                request.pose,
                clearance_m=self.config.navigation.initial_pose_clearance_m,
                max_radius_m=self.config.navigation.initial_pose_snap_radius_m,
            )

            covariance = [0.0] * 36
            covariance[0] = 0.25
            covariance[7] = 0.25
            covariance[35] = 0.068

            for _ in range(3):
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = snapped_pose.frame_id
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.pose.position.x = snapped_pose.x
                msg.pose.pose.position.y = snapped_pose.y
                msg.pose.pose.orientation.z = math.sin(snapped_pose.yaw / 2.0)
                msg.pose.pose.orientation.w = math.cos(snapped_pose.yaw / 2.0)
                msg.pose.covariance = covariance
                self.initial_pose_publisher.publish(msg)
                time.sleep(0.1)

            message = "初始位姿已发送"
            if pose_snapped:
                message = "初始位姿已发送，已吸附到最近可行点"
            return {
                "pose": dump_model(snapped_pose),
                "snapped": pose_snapped,
                "message": message,
            }

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
        self.thread = threading.Thread(target=self.executor.spin, name="ros2-web-console", daemon=True)
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
