#!/usr/bin/env python3
"""DLIO-synchronized cloud gate and OctoMap saver.

octomap_server owns the OcTree insertion. This node keeps the A2-specific policy:
only point clouds with a nearby DLIO odometry timestamp are forwarded, and the
running OctoMap is periodically persisted through octomap_saver_node.
"""

from __future__ import annotations

import os
import ast
import math
import subprocess
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque

import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def _stamp_to_sec(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


@dataclass(frozen=True)
class FilterStats:
    input_points: int
    kept_points: int
    self_points: int
    range_points: int
    invalid_points: int


def _float_list(value: object, *, expected_len: int, fallback: list[float]) -> list[float]:
    if isinstance(value, str):
        try:
            value = ast.literal_eval(value)
        except (SyntaxError, ValueError):
            return fallback
    try:
        result = [float(item) for item in value]  # type: ignore[arg-type]
    except TypeError:
        return fallback
    if len(result) != expected_len:
        return fallback
    return result


def _bool_value(value: object) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "t", "yes", "y", "on"}
    return bool(value)


def _field_offset(msg: PointCloud2, name: str) -> int:
    field = next((item for item in msg.fields if item.name == name), None)
    if field is None:
        raise ValueError(f"pointcloud_missing_{name}")
    if field.datatype != 7:
        raise ValueError(f"pointcloud_{name}_not_float32")
    return int(field.offset)


def _transform_lidar_to_base(
    x: float,
    y: float,
    z: float,
    translation: list[float],
    rotation: list[float],
) -> tuple[float, float, float]:
    return (
        rotation[0] * x + rotation[1] * y + rotation[2] * z + translation[0],
        rotation[3] * x + rotation[4] * y + rotation[5] * z + translation[1],
        rotation[6] * x + rotation[7] * y + rotation[8] * z + translation[2],
    )


def _in_box(
    x: float,
    y: float,
    z: float,
    box: tuple[float, float, float, float, float, float],
) -> bool:
    return (
        box[0] <= x <= box[1]
        and box[2] <= y <= box[3]
        and box[4] <= z <= box[5]
    )


def filter_octomap_cloud(
    msg: PointCloud2,
    lidar_to_base_translation: list[float],
    lidar_to_base_rotation: list[float],
    self_filter_box: tuple[float, float, float, float, float, float],
    min_range_m: float,
    max_range_m: float,
    self_filter_enabled: bool,
) -> tuple[PointCloud2, FilterStats]:
    x_offset = _field_offset(msg, "x")
    y_offset = _field_offset(msg, "y")
    z_offset = _field_offset(msg, "z")

    if len(lidar_to_base_translation) != 3 or len(lidar_to_base_rotation) != 9:
        raise ValueError("invalid_lidar_to_base_transform")
    if msg.point_step <= 0:
        raise ValueError("invalid_point_step")

    min_range_sq = min_range_m * min_range_m if min_range_m > 0.0 else None
    max_range_sq = max_range_m * max_range_m if max_range_m > 0.0 else None
    endian = ">" if msg.is_bigendian else "<"
    unpack_float = struct.Struct(f"{endian}f").unpack_from
    raw = memoryview(msg.data)
    kept = bytearray()

    input_points = int(msg.width) * int(msg.height)
    self_points = 0
    range_points = 0
    invalid_points = 0

    for row in range(int(msg.height)):
        row_base = row * int(msg.row_step)
        for col in range(int(msg.width)):
            point_base = row_base + col * int(msg.point_step)
            point_end = point_base + int(msg.point_step)
            if point_end > len(raw):
                invalid_points += 1
                continue

            x = unpack_float(raw, point_base + x_offset)[0]
            y = unpack_float(raw, point_base + y_offset)[0]
            z = unpack_float(raw, point_base + z_offset)[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                invalid_points += 1
                continue

            range_sq = x * x + y * y + z * z
            if (min_range_sq is not None and range_sq < min_range_sq) or (
                max_range_sq is not None and range_sq > max_range_sq
            ):
                range_points += 1
                continue

            base_x, base_y, base_z = _transform_lidar_to_base(
                x,
                y,
                z,
                lidar_to_base_translation,
                lidar_to_base_rotation,
            )
            if self_filter_enabled and _in_box(base_x, base_y, base_z, self_filter_box):
                self_points += 1
                continue

            kept.extend(raw[point_base:point_end])

    filtered = PointCloud2()
    filtered.header = msg.header
    filtered.height = 1
    filtered.width = len(kept) // int(msg.point_step)
    filtered.fields = msg.fields
    filtered.is_bigendian = msg.is_bigendian
    filtered.point_step = msg.point_step
    filtered.row_step = filtered.point_step * filtered.width
    filtered.data = bytes(kept)
    filtered.is_dense = msg.is_dense and invalid_points == 0

    return filtered, FilterStats(
        input_points=input_points,
        kept_points=int(filtered.width),
        self_points=self_points,
        range_points=range_points,
        invalid_points=invalid_points,
    )


class OctomapMappingNode(Node):
    def __init__(self) -> None:
        super().__init__("octomap_mapping_node")

        self.odom_topic = str(self.declare_parameter("odom_topic", "/jt128/dlio/odom").value)
        self.cloud_topic = str(self.declare_parameter("cloud_topic", "/jt128/front/points").value)
        self.filtered_cloud_topic = str(
            self.declare_parameter("filtered_cloud_topic", "/a2/octomap/cloud_in").value
        )
        self.max_stamp_delta_sec = float(self.declare_parameter("max_stamp_delta_sec", 0.010).value)
        self.odom_cache_sec = float(self.declare_parameter("odom_cache_sec", 2.0).value)
        self.save_path = str(self.declare_parameter("save_path", "").value)
        self.save_period_sec = float(self.declare_parameter("save_period_sec", 30.0).value)
        self.save_on_shutdown = bool(self.declare_parameter("save_on_shutdown", True).value)
        self.self_filter_enabled = _bool_value(
            self.declare_parameter("self_filter_enabled", True).value
        )
        self.lidar_to_base_translation = _float_list(
            self.declare_parameter(
                "lidar_to_base_translation", [0.33767, 0.0, 0.08134]
            ).value,
            expected_len=3,
            fallback=[0.33767, 0.0, 0.08134],
        )
        self.lidar_to_base_rotation = _float_list(
            self.declare_parameter(
                "lidar_to_base_rotation",
                [
                    0.0,
                    0.0,
                    1.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                ],
            ).value,
            expected_len=9,
            fallback=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.self_filter_min_x = float(self.declare_parameter("self_filter_min_x", -0.70).value)
        self.self_filter_max_x = float(self.declare_parameter("self_filter_max_x", 0.70).value)
        self.self_filter_min_y = float(self.declare_parameter("self_filter_min_y", -0.45).value)
        self.self_filter_max_y = float(self.declare_parameter("self_filter_max_y", 0.45).value)
        self.self_filter_min_z = float(self.declare_parameter("self_filter_min_z", -0.30).value)
        self.self_filter_max_z = float(self.declare_parameter("self_filter_max_z", 0.80).value)
        self.min_range_m = float(self.declare_parameter("min_range_m", 0.20).value)
        self.max_range_m = float(self.declare_parameter("max_range_m", 12.0).value)

        qos = QoSProfile(
            depth=20,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.odom_stamps: Deque[float] = deque()
        self.forwarded_clouds = 0
        self.dropped_clouds = 0
        self.forwarded_points = 0
        self.filtered_self_points = 0
        self.filtered_range_points = 0
        self.filtered_invalid_points = 0
        self.last_save_time = 0.0
        self._saving = False
        self._lock = threading.Lock()

        self.cloud_pub = self.create_publisher(PointCloud2, self.filtered_cloud_topic, qos)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, qos)
        self.create_subscription(PointCloud2, self.cloud_topic, self._on_cloud, qos)

        if self.save_path and self.save_period_sec > 0.0:
            self.create_timer(self.save_period_sec, self._save_timer)

        self.create_timer(5.0, self._status_timer)
        self.get_logger().info(
            "OctoMap cloud gate: cloud=%s odom=%s out=%s max_delta=%.3fs "
            "save=%s self_filter=%s box=[%.2f,%.2f]x[%.2f,%.2f]x[%.2f,%.2f]"
            % (
                self.cloud_topic,
                self.odom_topic,
                self.filtered_cloud_topic,
                self.max_stamp_delta_sec,
                self.save_path or "disabled",
                self.self_filter_enabled,
                self.self_filter_min_x,
                self.self_filter_max_x,
                self.self_filter_min_y,
                self.self_filter_max_y,
                self.self_filter_min_z,
                self.self_filter_max_z,
            )
        )

    def _on_odom(self, msg: Odometry) -> None:
        stamp = _stamp_to_sec(msg.header.stamp)
        self.odom_stamps.append(stamp)
        cutoff = stamp - self.odom_cache_sec
        while self.odom_stamps and self.odom_stamps[0] < cutoff:
            self.odom_stamps.popleft()

    def _nearest_odom_delta(self, stamp: float) -> float | None:
        if not self.odom_stamps:
            return None
        return min(abs(stamp - odom_stamp) for odom_stamp in self.odom_stamps)

    def _on_cloud(self, msg: PointCloud2) -> None:
        stamp = _stamp_to_sec(msg.header.stamp)
        delta = self._nearest_odom_delta(stamp)
        if delta is None or delta > self.max_stamp_delta_sec:
            self.dropped_clouds += 1
            if self.dropped_clouds <= 5 or self.dropped_clouds % 100 == 0:
                reason = "no_odom" if delta is None else f"delta={delta:.4f}s"
                self.get_logger().warn(f"Dropping OctoMap cloud: {reason}")
            return
        filtered_msg, stats = self._filter_cloud(msg)
        self.filtered_self_points += stats.self_points
        self.filtered_range_points += stats.range_points
        self.filtered_invalid_points += stats.invalid_points
        self.forwarded_points += stats.kept_points
        self.cloud_pub.publish(filtered_msg)
        self.forwarded_clouds += 1

    def _filter_cloud(self, msg: PointCloud2) -> tuple[PointCloud2, "FilterStats"]:
        if not self.self_filter_enabled and self.min_range_m <= 0.0 and self.max_range_m <= 0.0:
            total_points = int(msg.width) * int(msg.height)
            return msg, FilterStats(total_points, total_points, 0, 0, 0)

        try:
            return filter_octomap_cloud(
                msg,
                self.lidar_to_base_translation,
                self.lidar_to_base_rotation,
                (
                    self.self_filter_min_x,
                    self.self_filter_max_x,
                    self.self_filter_min_y,
                    self.self_filter_max_y,
                    self.self_filter_min_z,
                    self.self_filter_max_z,
                ),
                self.min_range_m,
                self.max_range_m,
                self.self_filter_enabled,
            )
        except ValueError as exc:
            self.get_logger().warn(f"Skipping OctoMap self filter: {exc}")
            total_points = int(msg.width) * int(msg.height)
            return msg, FilterStats(total_points, total_points, 0, 0, 0)

    def _status_timer(self) -> None:
        self.get_logger().info(
            "OctoMap gate stats: clouds_forwarded=%d clouds_dropped=%d odom_cache=%d "
            "points_forwarded=%d filtered_self=%d filtered_range=%d filtered_invalid=%d"
            % (
                self.forwarded_clouds,
                self.dropped_clouds,
                len(self.odom_stamps),
                self.forwarded_points,
                self.filtered_self_points,
                self.filtered_range_points,
                self.filtered_invalid_points,
            )
        )

    def _save_timer(self) -> None:
        self.save_octomap_async()

    def save_octomap_async(self) -> None:
        if not self.save_path:
            return
        with self._lock:
            if self._saving:
                return
            self._saving = True
        thread = threading.Thread(target=self._save_octomap, daemon=True)
        thread.start()

    def _save_octomap(self) -> None:
        try:
            save_path = Path(self.save_path)
            save_path.parent.mkdir(parents=True, exist_ok=True)
            tmp_path = save_path.with_name(save_path.stem + ".tmp" + save_path.suffix)
            cmd = [
                "ros2",
                "run",
                "octomap_server",
                "octomap_saver_node",
                "--ros-args",
                "-p",
                f"octomap_path:={tmp_path}",
            ]
            start = time.monotonic()
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=20)
            if result.returncode != 0:
                self.get_logger().warn(
                    "octomap_saver_node failed rc=%d output=%s"
                    % (result.returncode, result.stdout.strip())
                )
                return
            if not tmp_path.exists():
                self.get_logger().warn(
                    "octomap_saver_node did not create %s output=%s"
                    % (tmp_path, result.stdout.strip())
                )
                return
            os.replace(tmp_path, save_path)
            self.last_save_time = time.monotonic()
            self.get_logger().info(
                "Saved OctoMap to %s in %.1fs" % (save_path, time.monotonic() - start)
            )
        except subprocess.TimeoutExpired:
            self.get_logger().warn("Timed out while saving OctoMap")
        except Exception as exc:
            self.get_logger().warn(f"Failed to save OctoMap: {exc}")
        finally:
            with self._lock:
                self._saving = False

    def destroy_node(self) -> bool:
        if self.save_on_shutdown and self.save_path:
            self._save_octomap()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = OctomapMappingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
