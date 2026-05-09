#!/usr/bin/env python3

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        v = value.strip().lower()
        return v in ("1", "true", "t", "yes", "y", "on")
    if isinstance(value, (int, float)):
        return value != 0
    return bool(value)


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class PoseGoalController3D(Node):
    """Conservative short-range pose servo for the JT128 3D stack.

    This is intentionally not a full 3D planner. It closes the first real
    control loop by converting a nearby map-frame pose goal into `/cmd_vel`.
    Global planning, obstacle-aware 3D planning, and coverage exploration stay
    separate layers above this verified local servo.
    """

    def __init__(self) -> None:
        super().__init__("pose_goal_controller_3d")
        self.goal_topic = self.declare_parameter("goal_topic", "/a2/nav3/goal_pose").value
        self.legacy_goal_topic = self.declare_parameter("legacy_goal_topic", "/goal_pose_").value
        self.pose_topic = self.declare_parameter("pose_topic", "/a2/relocalization/pose").value
        self.cmd_topic = self.declare_parameter("cmd_topic", "/cmd_vel").value
        self.status_topic = self.declare_parameter("status_topic", "/a2/nav3/status").value
        self.localization_ok_topic = self.declare_parameter(
            "localization_ok_topic", "/a2/localization_ok"
        ).value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.dry_run = as_bool(self.declare_parameter("dry_run", True).value)
        self.require_localization_ok = as_bool(
            self.declare_parameter("require_localization_ok", True).value
        )
        self.require_obstacle_cloud = as_bool(
            self.declare_parameter("require_obstacle_cloud", True).value
        )
        self.obstacle_cloud_topic = self.declare_parameter(
            "obstacle_cloud_topic", "/jt128/front/points"
        ).value
        self.obstacle_cloud_timeout_sec = float(
            self.declare_parameter("obstacle_cloud_timeout_sec", 1.0).value
        )
        self.control_hz = max(1.0, float(self.declare_parameter("control_hz", 10.0).value))
        self.pose_timeout_sec = float(self.declare_parameter("pose_timeout_sec", 0.5).value)
        self.goal_timeout_sec = float(self.declare_parameter("goal_timeout_sec", 60.0).value)
        self.max_goal_distance_from_current = float(
            self.declare_parameter("max_goal_distance_from_current", 1.5).value
        )
        self.goal_tolerance_xy = float(self.declare_parameter("goal_tolerance_xy", 0.15).value)
        self.goal_tolerance_yaw = float(self.declare_parameter("goal_tolerance_yaw", 0.18).value)
        self.linear_gain = float(self.declare_parameter("linear_gain", 0.45).value)
        self.yaw_gain = float(self.declare_parameter("yaw_gain", 0.9).value)
        self.max_linear_x = float(self.declare_parameter("max_linear_x", 0.18).value)
        self.max_linear_y = float(self.declare_parameter("max_linear_y", 0.12).value)
        self.max_yaw_rate = float(self.declare_parameter("max_yaw_rate", 0.3).value)

        self.pose: PoseWithCovarianceStamped | None = None
        self.pose_time = None
        self.localization_ok = False
        self.obstacle_cloud_time = None
        self.obstacle_cloud_points = 0
        self.goal: PoseStamped | None = None
        self.goal_start_time = None
        self.last_status = ""
        self.last_logged_state = ""
        self.last_log_time = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self.on_goal, 10)
        if self.legacy_goal_topic and self.legacy_goal_topic != self.goal_topic:
            self.create_subscription(PoseStamped, self.legacy_goal_topic, self.on_goal, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.on_pose, 20)
        self.create_subscription(Bool, self.localization_ok_topic, self.on_localization_ok, 10)
        if self.require_obstacle_cloud:
            self.create_subscription(PointCloud2, self.obstacle_cloud_topic, self.on_obstacle_cloud, 10)
        self.create_timer(1.0 / self.control_hz, self.tick)
        self.publish_status(False, "idle", "waiting_goal")

    def on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.pose = msg
        self.pose_time = self.get_clock().now()

    def on_localization_ok(self, msg: Bool) -> None:
        self.localization_ok = bool(msg.data)

    def on_obstacle_cloud(self, msg: PointCloud2) -> None:
        self.obstacle_cloud_time = self.get_clock().now()
        self.obstacle_cloud_points = int(msg.width) * int(msg.height)

    def on_goal(self, msg: PoseStamped) -> None:
        frame = msg.header.frame_id or self.map_frame
        if frame != self.map_frame:
            self.reject_goal(f"bad_frame:{frame}")
            return
        if self.pose is None:
            self.reject_goal("no_current_pose")
            return
        if not self.obstacle_cloud_is_fresh():
            self.reject_goal("obstacle_cloud_stale")
            return
        gx = float(msg.pose.position.x)
        gy = float(msg.pose.position.y)
        if not math.isfinite(gx) or not math.isfinite(gy):
            self.reject_goal("nonfinite_goal")
            return
        px = float(self.pose.pose.pose.position.x)
        py = float(self.pose.pose.pose.position.y)
        distance = math.hypot(gx - px, gy - py)
        if distance > self.max_goal_distance_from_current:
            self.reject_goal(
                f"goal_too_far:distance={distance:.2f},limit={self.max_goal_distance_from_current:.2f}"
            )
            return
        self.goal = msg
        self.goal.header.frame_id = self.map_frame
        self.goal_start_time = self.get_clock().now()
        self.publish_status(True, "goal_active", f"accepted;distance={distance:.2f};dry_run={self.dry_run}")

    def reject_goal(self, reason: str) -> None:
        self.goal = None
        self.goal_start_time = None
        self.publish_zero()
        self.publish_status(False, "goal_rejected", reason)

    def pose_is_fresh(self) -> bool:
        if self.pose is None or self.pose_time is None:
            return False
        age = (self.get_clock().now() - self.pose_time).nanoseconds * 1e-9
        return age <= self.pose_timeout_sec

    def obstacle_cloud_is_fresh(self) -> bool:
        if not self.require_obstacle_cloud:
            return True
        if self.obstacle_cloud_time is None or self.obstacle_cloud_points <= 0:
            return False
        age = (self.get_clock().now() - self.obstacle_cloud_time).nanoseconds * 1e-9
        return age <= self.obstacle_cloud_timeout_sec

    def tick(self) -> None:
        if self.goal is None:
            return
        if not self.pose_is_fresh():
            self.publish_zero()
            self.publish_status(False, "blocked", "pose_stale")
            return
        if self.require_localization_ok and not self.localization_ok:
            self.publish_zero()
            self.publish_status(False, "blocked", "localization_not_ready")
            return
        if not self.obstacle_cloud_is_fresh():
            self.publish_zero()
            self.publish_status(False, "blocked", "obstacle_cloud_stale")
            return
        if self.goal_start_time is not None:
            age = (self.get_clock().now() - self.goal_start_time).nanoseconds * 1e-9
            if age > self.goal_timeout_sec:
                self.goal = None
                self.goal_start_time = None
                self.publish_zero()
                self.publish_status(False, "goal_timeout", f"age={age:.1f}")
                return

        pose = self.pose.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        goal_yaw = yaw_from_quaternion(self.goal.pose.orientation)
        dx = float(self.goal.pose.position.x) - float(pose.position.x)
        dy = float(self.goal.pose.position.y) - float(pose.position.y)
        distance = math.hypot(dx, dy)
        yaw_error = normalize_angle(goal_yaw - yaw)

        if distance <= self.goal_tolerance_xy and abs(yaw_error) <= self.goal_tolerance_yaw:
            self.goal = None
            self.goal_start_time = None
            self.publish_zero()
            self.publish_status(True, "goal_reached", f"distance={distance:.3f};yaw_error={yaw_error:.3f}")
            return

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        forward_error = cos_yaw * dx + sin_yaw * dy
        lateral_error = -sin_yaw * dx + cos_yaw * dy

        cmd = Twist()
        cmd.linear.x = clamp(self.linear_gain * forward_error, self.max_linear_x)
        cmd.linear.y = clamp(self.linear_gain * lateral_error, self.max_linear_y)
        cmd.angular.z = clamp(self.yaw_gain * yaw_error, self.max_yaw_rate)
        if not self.dry_run:
            self.cmd_pub.publish(cmd)
        self.publish_status(
            True,
            "running",
            (
                f"distance={distance:.3f};yaw_error={yaw_error:.3f};"
                f"vx={cmd.linear.x:.3f};vy={cmd.linear.y:.3f};wz={cmd.angular.z:.3f};dry_run={self.dry_run}"
            ),
        )

    def publish_zero(self) -> None:
        if not self.dry_run:
            self.cmd_pub.publish(Twist())

    def publish_status(self, ready: bool, state: str, reason: str) -> None:
        status = (
            f"state={state};ready={str(bool(ready)).lower()};reason={reason};"
            f"goal_topic={self.goal_topic};pose_topic={self.pose_topic};cmd_topic={self.cmd_topic};"
            f"require_obstacle_cloud={str(bool(self.require_obstacle_cloud)).lower()};"
            f"obstacle_cloud_topic={self.obstacle_cloud_topic};obstacle_cloud_points={self.obstacle_cloud_points}"
        )
        self.status_pub.publish(String(data=status))
        now = self.get_clock().now()
        log_age = (now - self.last_log_time).nanoseconds * 1e-9
        should_log = state != self.last_logged_state or log_age >= 5.0
        if should_log:
            self.get_logger().info(f"3D pose controller status changed: {status}")
            self.last_logged_state = state
            self.last_log_time = now
        self.last_status = status


def main() -> None:
    rclpy.init()
    node = PoseGoalController3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
