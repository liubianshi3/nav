#!/usr/bin/env python3

from __future__ import annotations

import math
import struct
import threading
from collections import OrderedDict

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


class PointCloudAccumulator(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_accumulator")
        self.pointcloud_topic = self.declare_parameter(
            "pointcloud_topic", "/jt128/front/points"
        ).value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/a2/pointcloud_map_3d"
        ).value
        self.status_topic = self.declare_parameter(
            "status_topic", "/a2/pointcloud_map/status"
        ).value
        self.output_frame = self.declare_parameter("output_frame", "odom").value
        self.publish_rate_hz = max(
            0.2, float(self.declare_parameter("publish_rate_hz", 2.0).value)
        )
        self.voxel_size = max(
            0.01, float(self.declare_parameter("voxel_size", 0.08).value)
        )
        self.max_voxels = max(
            1000, int(self.declare_parameter("max_voxels", 250000).value)
        )
        self.max_points_per_scan = max(
            1000, int(self.declare_parameter("max_points_per_scan", 18000).value)
        )
        self.min_translation_delta_m = max(
            0.0,
            float(self.declare_parameter("min_translation_delta_m", 0.10).value),
        )
        self.min_yaw_delta_rad = max(
            0.0, float(self.declare_parameter("min_yaw_delta_rad", 0.08).value)
        )
        self.min_range_m = max(
            0.0, float(self.declare_parameter("min_range_m", 0.25).value)
        )
        self.max_range_m = max(
            self.min_range_m + 0.1,
            float(self.declare_parameter("max_range_m", 8.0).value),
        )
        self.z_min_m = float(self.declare_parameter("z_min_m", -1.0).value)
        self.z_max_m = float(self.declare_parameter("z_max_m", 2.2).value)
        self.lidar_offset_xyz = [
            float(value)
            for value in self.declare_parameter(
                "lidar_offset_xyz", [0.32, 0.0, 0.24]
            ).value
        ]
        self.lidar_offset_rpy = [
            float(value)
            for value in self.declare_parameter(
                "lidar_offset_rpy", [0.0, 0.0, 0.0]
            ).value
        ]
        legacy_lidar_offset_yaw = float(
            self.declare_parameter("lidar_offset_yaw", self.lidar_offset_rpy[2]).value
        )
        self.lidar_offset_rpy[2] = legacy_lidar_offset_yaw
        self.lidar_offset_quat = _quat_from_rpy(*self.lidar_offset_rpy)
        self.body_exclusion_x = [
            float(value)
            for value in self.declare_parameter(
                "body_exclusion_x", [-0.45, 0.50]
            ).value
        ]
        self.body_exclusion_y = [
            float(value)
            for value in self.declare_parameter(
                "body_exclusion_y", [-0.30, 0.30]
            ).value
        ]

        self._lock = threading.Lock()
        self._pose: tuple[float, float, float, float, float, float, float, float] | None = None
        self._last_integrated_pose: tuple[float, float, float, float, float, float, float, float] | None = None
        self._voxels: OrderedDict[tuple[int, int, int], tuple[float, float, float]] = OrderedDict()
        self._last_status = ""

        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.status_publisher = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 20)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self._on_cloud, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._publish_map)
        self._publish_status("waiting_odom", "startup")

    def _on_odom(self, msg: Odometry) -> None:
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        with self._lock:
            self._pose = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                float(msg.pose.pose.position.z),
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
                float(yaw),
            )

    def _should_integrate_pose(
        self,
        pose: tuple[float, float, float, float, float, float, float, float],
    ) -> bool:
        if self._last_integrated_pose is None:
            return True
        dx = pose[0] - self._last_integrated_pose[0]
        dy = pose[1] - self._last_integrated_pose[1]
        dz = pose[2] - self._last_integrated_pose[2]
        dyaw = math.atan2(
            math.sin(pose[7] - self._last_integrated_pose[7]),
            math.cos(pose[7] - self._last_integrated_pose[7]),
        )
        return (
            math.sqrt(dx * dx + dy * dy + dz * dz) >= self.min_translation_delta_m
            or abs(dyaw) >= self.min_yaw_delta_rad
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        with self._lock:
            pose = self._pose
        if pose is None:
            self._publish_status("waiting_odom", "pointcloud_without_odom")
            return
        if not self._should_integrate_pose(pose):
            return

        x_field = next((field for field in msg.fields if field.name == "x"), None)
        y_field = next((field for field in msg.fields if field.name == "y"), None)
        z_field = next((field for field in msg.fields if field.name == "z"), None)
        if x_field is None or y_field is None or z_field is None:
            self._publish_status("error", "pointcloud_missing_xyz")
            return
        if x_field.datatype != 7 or y_field.datatype != 7 or z_field.datatype != 7:
            self._publish_status("error", "pointcloud_non_float32")
            return

        total_points = int(msg.width) * int(msg.height)
        if total_points <= 0:
            return

        scan_stride = max(1, int(math.ceil(total_points / self.max_points_per_scan)))
        endian = ">" if msg.is_bigendian else "<"
        unpack_float = struct.Struct(f"{endian}f").unpack_from
        raw = memoryview(msg.data)
        min_range_sq = self.min_range_m * self.min_range_m
        max_range_sq = self.max_range_m * self.max_range_m
        world_quat = _normalize_quat((pose[3], pose[4], pose[5], pose[6]))

        added_points = 0
        updated_points = 0

        with self._lock:
            for point_index in range(0, total_points, scan_stride):
                base = point_index * msg.point_step
                px = unpack_float(raw, base + x_field.offset)[0]
                py = unpack_float(raw, base + y_field.offset)[0]
                pz = unpack_float(raw, base + z_field.offset)[0]
                if not (
                    math.isfinite(px)
                    and math.isfinite(py)
                    and math.isfinite(pz)
                ):
                    continue
                range_sq = px * px + py * py + pz * pz
                if range_sq < min_range_sq or range_sq > max_range_sq:
                    continue

                rotated_lidar_point = _quat_rotate(
                    self.lidar_offset_quat,
                    (px, py, pz),
                )
                lidar_x = rotated_lidar_point[0] + self.lidar_offset_xyz[0]
                lidar_y = rotated_lidar_point[1] + self.lidar_offset_xyz[1]
                lidar_z = rotated_lidar_point[2] + self.lidar_offset_xyz[2]

                if (
                    self.body_exclusion_x[0] <= lidar_x <= self.body_exclusion_x[1]
                    and self.body_exclusion_y[0] <= lidar_y <= self.body_exclusion_y[1]
                ):
                    continue
                if lidar_z < self.z_min_m or lidar_z > self.z_max_m:
                    continue

                rotated_world_point = _quat_rotate(
                    world_quat,
                    (lidar_x, lidar_y, lidar_z),
                )
                world_x = pose[0] + rotated_world_point[0]
                world_y = pose[1] + rotated_world_point[1]
                world_z = pose[2] + rotated_world_point[2]
                voxel = (
                    int(math.floor(world_x / self.voxel_size)),
                    int(math.floor(world_y / self.voxel_size)),
                    int(math.floor(world_z / self.voxel_size)),
                )
                if voxel in self._voxels:
                    updated_points += 1
                    self._voxels.move_to_end(voxel)
                else:
                    added_points += 1
                self._voxels[voxel] = (world_x, world_y, world_z)
                while len(self._voxels) > self.max_voxels:
                    self._voxels.popitem(last=False)

            self._last_integrated_pose = pose

        reason = f"points={len(self._voxels)};added={added_points};updated={updated_points}"
        self._publish_status("accumulating", reason)

    def _publish_map(self) -> None:
        with self._lock:
            points = list(self._voxels.values())
        if not points:
            return

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.output_frame
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        payload = bytearray(msg.row_step)
        pack_into = struct.Struct("<fff").pack_into
        for index, point in enumerate(points):
            pack_into(payload, index * msg.point_step, point[0], point[1], point[2])
        msg.data = bytes(payload)
        self.publisher.publish(msg)
        self._publish_status("ready", f"points={len(points)};frame={self.output_frame}")

    def _publish_status(self, state: str, reason: str) -> None:
        status = (
            f"mode=real;state={state};ready={str(bool(self._voxels)).lower()};"
            f"reason={reason};source_topic={self.pointcloud_topic};output_topic={self.output_topic};"
            f"pose_source=odom_quaternion"
        )
        if status == self._last_status:
            return
        self.status_publisher.publish(String(data=status))
        self._last_status = status
        self.get_logger().info(status)


def main() -> None:
    rclpy.init()
    node = PointCloudAccumulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return _normalize_quat(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


def _normalize_quat(quat: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = quat
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def _quat_rotate(
    quat: tuple[float, float, float, float],
    point: tuple[float, float, float],
) -> tuple[float, float, float]:
    qx, qy, qz, qw = quat
    px, py, pz = point
    tx = 2.0 * (qy * pz - qz * py)
    ty = 2.0 * (qz * px - qx * pz)
    tz = 2.0 * (qx * py - qy * px)
    return (
        px + qw * tx + qy * tz - qz * ty,
        py + qw * ty + qz * tx - qx * tz,
        pz + qw * tz + qx * ty - qy * tx,
    )


if __name__ == "__main__":
    main()
