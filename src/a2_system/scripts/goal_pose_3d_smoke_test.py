#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


@dataclass
class PoseSample:
    x: float
    y: float
    yaw: float
    frame_id: str
    stamp_monotonic: float


class GoalPose3DSmokeNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("goal_pose_3d_smoke_test")
        self.args = args
        self.latest_pose: PoseSample | None = None
        self.localization_ok: bool | None = None
        self.pointcloud_seen = False
        self.goal_pub = self.create_publisher(PoseStamped, args.goal_topic, 10)
        self.stop_pub = self.create_publisher(Twist, args.stop_topic, 10)
        self.create_subscription(Odometry, args.pose_topic, self._on_odom, 20)
        self.create_subscription(Bool, args.localization_ok_topic, self._on_localization_ok, 10)
        self.create_subscription(PointCloud2, args.pointcloud_topic, self._on_pointcloud, 10)

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.latest_pose = PoseSample(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w)),
            frame_id=msg.header.frame_id or "odom",
            stamp_monotonic=time.monotonic(),
        )

    def _on_localization_ok(self, msg: Bool) -> None:
        self.localization_ok = bool(msg.data)

    def _on_pointcloud(self, _msg: PointCloud2) -> None:
        self.pointcloud_seen = True

    def wait_ready(self) -> PoseSample:
        deadline = time.monotonic() + self.args.preflight_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_pose is None:
                continue
            pose_age = time.monotonic() - self.latest_pose.stamp_monotonic
            if pose_age > self.args.max_pose_age_sec:
                continue
            if self.args.require_localization_ok and self.localization_ok is not True:
                continue
            if self.args.require_pointcloud and not self.pointcloud_seen:
                continue
            return self.latest_pose
        raise RuntimeError(
            "3D preflight failed: "
            f"pose={self.latest_pose is not None}, "
            f"localization_ok={self.localization_ok}, "
            f"pointcloud={self.pointcloud_seen}"
        )

    def make_goal(self, pose: PoseSample) -> PoseStamped:
        dx = self.args.relative_x * math.cos(pose.yaw) - self.args.relative_y * math.sin(pose.yaw)
        dy = self.args.relative_x * math.sin(pose.yaw) + self.args.relative_y * math.cos(pose.yaw)
        yaw = normalize_angle(pose.yaw + self.args.relative_yaw)
        goal = PoseStamped()
        goal.header.frame_id = self.args.goal_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = pose.x + dx
        goal.pose.position.y = pose.y + dy
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def publish_goal(self, goal: PoseStamped) -> None:
        for _ in range(max(1, self.args.goal_burst_count)):
            goal.header.stamp = self.get_clock().now().to_msg()
            self.goal_pub.publish(goal)
            rclpy.spin_once(self, timeout_sec=0.05)

    def publish_stop(self) -> None:
        stop = Twist()
        for _ in range(max(1, self.args.stop_burst_count)):
            self.stop_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_result(self, goal: PoseStamped) -> int:
        deadline = time.monotonic() + self.args.goal_timeout_sec
        last_print = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_pose is None:
                continue
            dx = float(goal.pose.position.x) - self.latest_pose.x
            dy = float(goal.pose.position.y) - self.latest_pose.y
            distance = math.hypot(dx, dy)
            goal_yaw = yaw_from_quaternion(
                float(goal.pose.orientation.x),
                float(goal.pose.orientation.y),
                float(goal.pose.orientation.z),
                float(goal.pose.orientation.w),
            )
            yaw_error = abs(normalize_angle(goal_yaw - self.latest_pose.yaw))
            now = time.monotonic()
            if now - last_print >= self.args.progress_print_sec:
                print(
                    "progress "
                    f"distance={distance:.3f}m yaw_error={yaw_error:.3f}rad "
                    f"pose=({self.latest_pose.x:.3f},{self.latest_pose.y:.3f},{self.latest_pose.yaw:.3f})"
                )
                last_print = now
            if distance <= self.args.position_tolerance_m and yaw_error <= self.args.yaw_tolerance_rad:
                self.publish_stop()
                print(
                    "PASS: 3D goal reached "
                    f"topic={self.args.goal_topic} distance={distance:.3f}m yaw_error={yaw_error:.3f}rad"
                )
                return 0
        self.publish_stop()
        print(f"FAIL: 3D goal timed out on {self.args.goal_topic}; stop signal published", file=sys.stderr)
        return 1


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A2 3D local-goal closed-loop smoke test")
    parser.add_argument("--pose-topic", default="/odom")
    parser.add_argument("--localization-ok-topic", default="/a2/localization_ok")
    parser.add_argument("--pointcloud-topic", default="/jt128/front/points")
    parser.add_argument("--goal-topic", default="/a2/nav3/goal_pose")
    parser.add_argument("--goal-frame", default="map")
    parser.add_argument("--stop-topic", default="/cmd_vel")
    parser.add_argument("--relative-x", type=float, default=0.25)
    parser.add_argument("--relative-y", type=float, default=0.0)
    parser.add_argument("--relative-yaw", type=float, default=0.0)
    parser.add_argument("--position-tolerance-m", type=float, default=0.35)
    parser.add_argument("--yaw-tolerance-rad", type=float, default=0.35)
    parser.add_argument("--preflight-timeout-sec", type=float, default=10.0)
    parser.add_argument("--goal-timeout-sec", type=float, default=25.0)
    parser.add_argument("--max-pose-age-sec", type=float, default=2.0)
    parser.add_argument("--progress-print-sec", type=float, default=1.0)
    parser.add_argument("--goal-burst-count", type=int, default=3)
    parser.add_argument("--stop-burst-count", type=int, default=8)
    parser.add_argument("--require-localization-ok", action="store_true", default=True)
    parser.add_argument("--no-require-localization-ok", dest="require_localization_ok", action="store_false")
    parser.add_argument("--require-pointcloud", action="store_true", default=True)
    parser.add_argument("--no-require-pointcloud", dest="require_pointcloud", action="store_false")
    parser.add_argument("--execute", action="store_true", help="Actually publish the goal; otherwise only preflight and print")
    parser.add_argument(
        "--i-understand-robot-will-move",
        action="store_true",
        help="Required with --execute because the robot may move",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    if args.execute and not args.i_understand_robot_will_move:
        print("--execute requires --i-understand-robot-will-move", file=sys.stderr)
        return 2

    rclpy.init()
    node = GoalPose3DSmokeNode(args)
    try:
        pose = node.wait_ready()
        goal = node.make_goal(pose)
        print(
            "READY: "
            f"pose_topic={args.pose_topic} pose_frame={pose.frame_id} "
            f"goal_topic={args.goal_topic} goal_frame={goal.header.frame_id} "
            f"goal=({goal.pose.position.x:.3f},{goal.pose.position.y:.3f}) "
            f"relative=({args.relative_x:.3f},{args.relative_y:.3f},{args.relative_yaw:.3f})"
        )
        if pose.frame_id != goal.header.frame_id:
            print(
                "WARN: pose frame and goal frame differ; result check assumes frames are aligned "
                f"pose_frame={pose.frame_id} goal_frame={goal.header.frame_id}",
                file=sys.stderr,
            )
        if not args.execute:
            print("DRY-RUN: goal was not published. Add --execute --i-understand-robot-will-move to move.")
            return 0
        node.publish_goal(goal)
        return node.wait_result(goal)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
