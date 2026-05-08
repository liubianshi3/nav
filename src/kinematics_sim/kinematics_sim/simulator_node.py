#!/usr/bin/env python3
"""
Lightweight kinematics simulator for offline A2 testing.

Replaces the real LiDAR driver + DLIO with:
  - Virtual lidar scans sampled from a pre-built PCD map
  - Kinematic odometry from /cmd_vel integration
  - Kidnap simulation via /initialpose

Enables full pipeline testing (ground_seg → relocalizer → Nav2 → diagnostics)
without hardware, GPU, or physics engine.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TransformStamped,
    Twist,
    Vector3,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import TransformBroadcaster


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Return (x, y, z, w) quaternion for a rotation around Z by yaw."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw from quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _build_transform_4x4(xyz: list[float], rot_9: list[float]) -> np.ndarray:
    """Build a 4x4 homogeneous transform from translation + 9-element rotation matrix."""
    m = np.eye(4, dtype=np.float64)
    m[:3, :3] = np.array(rot_9, dtype=np.float64).reshape(3, 3)
    m[:3, 3] = np.array(xyz, dtype=np.float64)
    return m


def _transform_points_inv(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Transform points by the inverse of a 4x4 matrix: pts_new = inv(T) @ pts.

    Equivalent to: (pts - t) @ R  (since inv(T) = [R.T, -R.T @ t]).
    """
    R = transform[:3, :3]
    t = transform[:3, 3]
    return (points - t) @ R


class KinematicsSimulator(Node):
    """Lightweight kinematics simulator node."""

    def __init__(self) -> None:
        super().__init__("simulator_node")

        # ── parameters ──────────────────────────────────────────────
        pcd_raw = self.declare_parameter(
            "pcd_map_path", "${A2_WORKSPACE}/runtime/maps/current/pointcloud_map_3d.pcd"
        ).value
        self.pcd_map_path = Path(os.path.expandvars(os.path.expanduser(pcd_raw)))

        self.lidar_range = float(self.declare_parameter("lidar_range_m", 80.0).value)
        self.lidar_points_per_scan = int(self.declare_parameter("lidar_points_per_scan", 5000).value)
        self.lidar_noise_stddev = float(self.declare_parameter("lidar_noise_stddev_m", 0.02).value)
        self.lidar_rate = float(self.declare_parameter("lidar_rate_hz", 10.0).value)
        self.odom_rate = float(self.declare_parameter("odom_rate_hz", 200.0).value)
        self.base_height = float(self.declare_parameter("base_height_m", 0.28).value)
        self.pos_variance = float(self.declare_parameter("odom_pos_variance", 0.01).value)
        self.rot_variance = float(self.declare_parameter("odom_rot_variance", 0.005).value)

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.lidar_frame = self.declare_parameter("lidar_frame", "jt128_front_link").value

        self.base_to_lidar_xyz = list(
            self.declare_parameter(
                "base_to_lidar_xyz", [0.33767, 0.0, 0.08134]
            ).value
        )
        self.base_to_lidar_rot = list(
            self.declare_parameter(
                "base_to_lidar_rotation_matrix",
                [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            ).value
        )

        # ── state ────────────────────────────────────────────────────
        self.x = float(self.declare_parameter("initial_x", 0.0).value)
        self.y = float(self.declare_parameter("initial_y", 0.0).value)
        self.yaw = float(self.declare_parameter("initial_yaw", 0.0).value)
        self._last_cmd = Twist()
        self._last_cmd_time = self.get_clock().now()
        self._have_cmd = False

        # ── load PCD map ─────────────────────────────────────────────
        self.map_points = self._load_pcd()
        self.get_logger().info(
            f"Loaded PCD map: {len(self.map_points)} points from {self.pcd_map_path}"
        )

        # ── build static base→lidar transform ────────────────────────
        self.base_to_lidar = _build_transform_4x4(
            self.base_to_lidar_xyz, self.base_to_lidar_rot
        )

        # ── publishers ───────────────────────────────────────────────
        self._lidar_pub = self.create_publisher(
            PointCloud2, "/jt128/front/points", 10
        )
        self._odom_pub = self.create_publisher(Odometry, "/jt128/dlio/odom", 10)
        self._status_pub = self.create_publisher(String, "/a2/sim/status", 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel_safe").value

        # ── subscribers ──────────────────────────────────────────────
        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._on_initial_pose, 10
        )

        # ── timers ───────────────────────────────────────────────────
        self._odom_timer = self.create_timer(1.0 / self.odom_rate, self._odom_tick)
        self._lidar_timer = self.create_timer(1.0 / self.lidar_rate, self._lidar_tick)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"Kinematics simulator ready: "
            f"lidar={self.lidar_rate}Hz {self.lidar_points_per_scan}pts "
            f"odom={self.odom_rate}Hz range={self.lidar_range}m"
        )

    # ── PCD loader ───────────────────────────────────────────────────
    def _load_pcd(self) -> np.ndarray:
        path = self.pcd_map_path
        if not path.exists():
            self.get_logger().error(f"PCD map not found: {path}")
            return np.zeros((0, 3), dtype=np.float64)

        pts: list[tuple[float, float, float]] = []
        fields: list[str] = []
        data_started = False

        with path.open("r", encoding="ascii", errors="strict") as f:
            for raw_line in f:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if data_started:
                    parts = line.split()
                    if len(parts) < 3:
                        continue
                    try:
                        if fields and {"x", "y", "z"}.issubset(set(fields)):
                            pts.append((
                                float(parts[fields.index("x")]),
                                float(parts[fields.index("y")]),
                                float(parts[fields.index("z")]),
                            ))
                        else:
                            pts.append((
                                float(parts[0]), float(parts[1]), float(parts[2]),
                            ))
                    except (ValueError, IndexError):
                        continue
                    continue
                key, _, value = line.partition(" ")
                if key.upper() == "FIELDS":
                    fields = value.split()
                if key.upper() == "DATA" and value.strip().lower() != "ascii":
                    self.get_logger().error("Simulator only supports ASCII PCD")
                    return np.zeros((0, 3), dtype=np.float64)
                if key.upper() == "DATA":
                    data_started = True

        return np.array(pts, dtype=np.float64)

    # ── command input ────────────────────────────────────────────────
    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()
        self._have_cmd = True

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Kidnap simulation: reset robot pose to an arbitrary location."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = float(p.x)
        self.y = float(p.y)
        self.yaw = _yaw_from_quaternion(
            float(q.x), float(q.y), float(q.z), float(q.w)
        )
        self._have_cmd = False
        self._last_cmd = Twist()
        self.get_logger().warn(
            f"KIDNAP: robot reset to x={self.x:.2f} y={self.y:.2f} yaw={math.degrees(self.yaw):.1f}°"
        )

    # ── odometry tick ────────────────────────────────────────────────
    def _odom_tick(self) -> None:
        """Integrate /cmd_vel into pose, publish TF and odometry."""
        now = self.get_clock().now()
        dt = 1.0 / self.odom_rate

        # Check for command timeout (0.5s)
        cmd_age = (now - self._last_cmd_time).nanoseconds * 1e-9
        if cmd_age > 0.5:
            self._last_cmd = Twist()
            self._have_cmd = False

        cmd = self._last_cmd
        vx = float(cmd.linear.x)
        vy = float(cmd.linear.y)
        wz = float(cmd.angular.z)

        # Kinematic integration (2D planar)
        self.x += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        self.y += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        self.yaw += wz * dt
        # Normalize yaw to [-π, π]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Publish odom → base_link TF
        qx, qy, qz, qw = _quaternion_from_yaw(self.yaw)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = self.base_height
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf_msg)

        # Publish DLIO-compatible odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.base_height
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        # Covariance
        odom.pose.covariance[0] = self.pos_variance
        odom.pose.covariance[7] = self.pos_variance
        odom.pose.covariance[14] = self.pos_variance * 2.0
        odom.pose.covariance[21] = self.rot_variance
        odom.pose.covariance[28] = self.rot_variance
        odom.pose.covariance[35] = self.rot_variance
        self._odom_pub.publish(odom)

    # ── lidar tick ───────────────────────────────────────────────────
    def _lidar_tick(self) -> None:
        """Sample PCD map around current pose, add noise, publish."""
        if len(self.map_points) == 0:
            return

        # Build map→base and map→lidar transforms
        qx, qy, qz, qw = _quaternion_from_yaw(self.yaw)
        map_to_base = np.eye(4, dtype=np.float64)
        # Rotation around Z
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        map_to_base[0, 0] = cy
        map_to_base[0, 1] = -sy
        map_to_base[1, 0] = sy
        map_to_base[1, 1] = cy
        map_to_base[:3, 3] = [self.x, self.y, self.base_height]

        map_to_lidar = map_to_base @ self.base_to_lidar

        # Select map points within range of the lidar
        lidar_pos = map_to_lidar[:3, 3]
        dists = np.linalg.norm(self.map_points - lidar_pos, axis=1)
        nearby = self.map_points[dists <= self.lidar_range]

        if len(nearby) == 0:
            return

        # Transform to lidar frame
        pts_lidar = _transform_points_inv(map_to_lidar, nearby)

        # Add Gaussian noise
        if self.lidar_noise_stddev > 0.0:
            pts_lidar += np.random.randn(*pts_lidar.shape).astype(np.float64) * self.lidar_noise_stddev

        # Downsample to lidar_points_per_scan
        n = len(pts_lidar)
        if n > self.lidar_points_per_scan:
            idx = np.random.choice(n, self.lidar_points_per_scan, replace=False)
            pts_lidar = pts_lidar[idx]
        elif n < self.lidar_points_per_scan and n > 0:
            # Repeat some points with small jitter to fill the count
            repeats = self.lidar_points_per_scan // n
            remainder = self.lidar_points_per_scan % n
            pts_lidar = np.concatenate([
                np.tile(pts_lidar, (repeats, 1)),
                pts_lidar[:remainder] + np.random.randn(remainder, 3).astype(np.float64) * 0.01,
            ])

        # Build PointCloud2 message
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = self.lidar_frame

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = point_cloud2.create_cloud(h, fields, pts_lidar.astype(np.float32))
        self._lidar_pub.publish(cloud)

    # ── status ───────────────────────────────────────────────────────
    def _publish_status(self) -> None:
        status = (
            f"state=simulating;ready=true;reason=ok;"
            f"x={self.x:.2f};y={self.y:.2f};yaw={math.degrees(self.yaw):.1f};"
            f"points={len(self.map_points)};range={self.lidar_range}m"
        )
        self._status_pub.publish(String(data=status))


def main() -> None:
    rclpy.init()
    node = KinematicsSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
