#!/usr/bin/env python3

from __future__ import annotations

import os
import struct
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


class PointCloudMapLoader(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_map_loader")
        raw_map_root = self.declare_parameter(
            "map_root", "${A2_WORKSPACE}/runtime/maps"
        ).value
        self.map_root = Path(os.path.expandvars(os.path.expanduser(raw_map_root)))
        self.map_id = self.declare_parameter("map_id", "").value
        self.pcd_path = self.declare_parameter("pcd_path", "").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/a2/map/pointcloud_3d"
        ).value
        self.status_topic = self.declare_parameter(
            "status_topic", "/a2/map_loader/status"
        ).value
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.publish_rate_hz = max(
            0.1, float(self.declare_parameter("publish_rate_hz", 1.0).value)
        )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, qos)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.pointcloud_msg: PointCloud2 | None = None
        self._last_status = ""

        try:
            self.pointcloud_msg = self._load_pcd()
            self._publish_status(
                "ready",
                f"points={self.pointcloud_msg.width};topic={self.output_topic}",
            )
        except Exception as exc:
            self._publish_status("error", f"load_failed:{exc}")
            self.get_logger().error(f"Failed to load 3D PCD map: {exc}")

        self.create_timer(1.0 / self.publish_rate_hz, self._publish)

    def _resolve_pcd_path(self) -> Path:
        if self.pcd_path:
            return Path(os.path.expandvars(os.path.expanduser(self.pcd_path)))
        if not self.map_id:
            current_file = self.map_root / "current_map.txt"
            if current_file.exists():
                self.map_id = current_file.read_text(encoding="utf-8").strip()
        if not self.map_id:
            raise RuntimeError("map_id or pcd_path is required")
        return self.map_root / self.map_id / "pointcloud_map_3d.pcd"

    def _load_pcd(self) -> PointCloud2:
        path = self._resolve_pcd_path()
        if not path.exists():
            raise RuntimeError(f"PCD not found: {path}")

        header: dict[str, str] = {}
        points: list[tuple[float, float, float]] = []
        with path.open("r", encoding="ascii", errors="strict") as handle:
            data_started = False
            fields: list[str] = []
            for raw_line in handle:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if data_started:
                    parts = line.split()
                    if len(parts) < 3:
                        continue
                    try:
                        if fields and {"x", "y", "z"}.issubset(set(fields)):
                            x = float(parts[fields.index("x")])
                            y = float(parts[fields.index("y")])
                            z = float(parts[fields.index("z")])
                        else:
                            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    except (ValueError, IndexError):
                        continue
                    points.append((x, y, z))
                    continue

                key, _, value = line.partition(" ")
                header[key.upper()] = value.strip()
                if key.upper() == "FIELDS":
                    fields = value.split()
                if key.upper() == "DATA":
                    if value.strip().lower() != "ascii":
                        raise RuntimeError("only ASCII PCD is supported by pointcloud_map_loader")
                    data_started = True

        if not points:
            raise RuntimeError(f"PCD has no readable XYZ points: {path}")

        msg = PointCloud2()
        msg.header.frame_id = self.frame_id
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
            pack_into(payload, index * msg.point_step, *point)
        msg.data = bytes(payload)
        self.get_logger().info(
            f"Loaded 3D PCD map {path} points={len(points)} frame={self.frame_id}"
        )
        return msg

    def _publish(self) -> None:
        if self.pointcloud_msg is None:
            return
        self.pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.pointcloud_msg)

    def _publish_status(self, state: str, reason: str) -> None:
        status = (
            f"state={state};ready={str(state == 'ready').lower()};reason={reason};"
            f"map_id={self.map_id or 'none'};output_topic={self.output_topic}"
        )
        if status == self._last_status:
            return
        self.status_pub.publish(String(data=status))
        self._last_status = status


def main() -> None:
    rclpy.init()
    node = PointCloudMapLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
