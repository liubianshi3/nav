#!/usr/bin/env python3

import math
import os
import struct
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from a2_interfaces.srv import ManageMap, SetMode
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class MapManagerNode(Node):
    def __init__(self):
        super().__init__("map_manager")
        self.use_mock = bool(self.declare_parameter("use_mock", True).value)
        self.runtime_mode = self.declare_parameter(
            "runtime_mode", "mock" if self.use_mock else "real"
        ).value
        raw_map_root = self.declare_parameter("map_root", "/tmp/a2_maps").value
        self.map_root = Path(os.path.expandvars(os.path.expanduser(raw_map_root)))
        self.occupancy_topic = self.declare_parameter("occupancy_topic", "/map").value
        self.map_representation = self.declare_parameter(
            "map_representation", "occupancy_grid_2d"
        ).value
        self.pointcloud_topic_3d = self.declare_parameter(
            "pointcloud_topic_3d", "/unitree/slam_lidar/points1"
        ).value
        self.pointcloud_snapshot_enabled = bool(
            self.declare_parameter("pointcloud_snapshot_enabled", True).value
        )
        self.pointcloud_max_points = int(
            self.declare_parameter("pointcloud_max_points", 200000).value
        )
        self.active_map_topic = self.declare_parameter(
            "active_map_topic", "/a2/map_manager/active_map"
        ).value
        self.mode_topic = self.declare_parameter("mode_topic", "/a2/system_mode").value
        self.status_topic = self.declare_parameter(
            "status_topic", "/a2/map_manager/status"
        ).value
        self.current_mode = self.declare_parameter("default_mode", "mapping").value
        self.map_transient_local = bool(
            self.declare_parameter("map_transient_local", False).value
        )
        self.latest_map = None
        self.latest_pointcloud = None
        self.active_map_id = ""
        self.last_status = ""

        self.map_root.mkdir(parents=True, exist_ok=True)
        self.active_pub = self.create_publisher(String, self.active_map_topic, 10)
        self.mode_pub = self.create_publisher(String, self.mode_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        map_qos = (
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            if self.map_transient_local
            else 10
        )
        self.create_subscription(
            OccupancyGrid, self.occupancy_topic, self.on_map, map_qos
        )
        if self.pointcloud_snapshot_enabled:
            self.create_subscription(
                PointCloud2, self.pointcloud_topic_3d, self.on_pointcloud, 10
            )
        self.create_service(
            ManageMap, "/map_manager/manage_map", self.handle_manage_map
        )
        self.create_service(SetMode, "/map_manager/set_mode", self.handle_set_mode)
        self.publish_mode()
        self.publish_status("idle", "startup")

    def on_map(self, msg):
        first_map = self.latest_map is None
        self.latest_map = msg
        if first_map:
            self.publish_status("ready", "map_received")

    def on_pointcloud(self, msg):
        first_cloud = self.latest_pointcloud is None
        self.latest_pointcloud = msg
        if first_cloud and self.latest_map is None:
            self.publish_status("ready", "pointcloud_received")

    def publish_active(self):
        self.active_pub.publish(String(data=self.active_map_id))

    def publish_mode(self):
        self.mode_pub.publish(String(data=self.current_mode))

    def list_maps(self):
        return sorted([item.name for item in self.map_root.iterdir() if item.is_dir()])

    def handle_set_mode(self, request, response):
        allowed = {"mapping", "localization", "navigation", "idle"}
        if request.mode not in allowed:
            response.success = False
            response.message = f"unsupported mode: {request.mode}"
            self.publish_status("error", f"unsupported_mode:{request.mode}")
            return response
        self.current_mode = request.mode
        self.publish_mode()
        self.publish_status("mode_changed", f"mode={self.current_mode}")
        response.success = True
        response.message = f"mode set to {self.current_mode}"
        return response

    def handle_manage_map(self, request, response):
        command = request.command.lower().strip()
        if command == "list":
            response.success = True
            response.message = "listed maps"
            response.map_ids = self.list_maps()
            self.publish_status("listed", f"count={len(response.map_ids)}")
            return response
        if command == "save":
            if self.latest_map is None and self.latest_pointcloud is None:
                response.success = False
                response.message = "no map or pointcloud received yet"
                self.publish_status("error", "no_map_or_pointcloud")
                return response
            try:
                map_id = self.save_map_bundle(request.map_id)
            except Exception as exc:
                response.success = False
                response.message = f"save failed: {exc}"
                self.publish_status("error", "save_failed")
                self.get_logger().error(f"Failed to save map bundle: {exc}")
                return response
            self.active_map_id = map_id
            self.publish_active()
            self.publish_status("saved", f"map_id={map_id}")
            response.success = True
            response.message = f"saved map {map_id}"
            response.map_ids = self.list_maps()
            return response
        if command == "load":
            map_id = request.map_id
            if not map_id or not (self.map_root / map_id).exists():
                response.success = False
                response.message = f"map not found: {map_id}"
                self.publish_status("error", f"map_not_found:{map_id}")
                return response
            self.active_map_id = map_id
            self.publish_active()
            self.publish_status("loaded", f"map_id={map_id}")
            response.success = True
            response.message = f"loaded map {map_id}"
            response.map_ids = self.list_maps()
            return response
        if command == "promote":
            map_id = request.map_id
            if not map_id or not (self.map_root / map_id).exists():
                response.success = False
                response.message = f"map not found: {map_id}"
                self.publish_status("error", f"map_not_found:{map_id}")
                return response
            with (self.map_root / "current_map.txt").open("w", encoding="utf-8") as handle:
                handle.write(map_id + "\n")
            self.active_map_id = map_id
            self.publish_active()
            self.publish_status("promoted", f"map_id={map_id}")
            response.success = True
            response.message = f"promoted map {map_id}"
            response.map_ids = self.list_maps()
            return response
        response.success = False
        response.message = f"unsupported command: {request.command}"
        self.publish_status("error", f"unsupported_command:{request.command}")
        return response

    def save_map_bundle(self, requested_map_id: str) -> str:
        map_id = requested_map_id or datetime.now().strftime("map_%Y%m%d_%H%M%S")
        map_dir = self.map_root / map_id
        map_dir.mkdir(parents=True, exist_ok=True)

        artifacts = []
        if self.latest_map is not None:
            self.write_nav2_map(self.latest_map, map_dir)
            artifacts.append(
                {
                    "kind": "occupancy_grid_2d",
                    "topic": self.occupancy_topic,
                    "path": "map.yaml",
                    "resolution": float(self.latest_map.info.resolution),
                }
            )
        if self.pointcloud_snapshot_enabled and self.latest_pointcloud is not None:
            artifacts.append(
                self.write_pointcloud_snapshot(self.latest_pointcloud, map_dir)
            )

        metadata = {
            "created_at": datetime.now().isoformat(),
            "mode": self.current_mode,
            "representation": self.map_representation,
            "source_topic": self.occupancy_topic,
            "width": self.latest_map.info.width if self.latest_map is not None else None,
            "height": self.latest_map.info.height if self.latest_map is not None else None,
            "resolution": self.latest_map.info.resolution
            if self.latest_map is not None
            else None,
            "pointcloud_topic_3d": self.pointcloud_topic_3d
            if self.latest_pointcloud is not None
            else None,
            "artifacts": artifacts,
        }
        with (map_dir / "metadata.yaml").open("w", encoding="utf-8") as handle:
            yaml.safe_dump(metadata, handle, sort_keys=False)
        return map_id

    def publish_status(self, state, reason):
        mode = self.runtime_mode
        ready = self.latest_map is not None or self.latest_pointcloud is not None
        status = (
            f"mode={mode};state={state};ready={str(bool(ready)).lower()};reason={reason};"
            f"system_mode={self.current_mode};active_map={self.active_map_id or 'none'};"
            f"representation={self.map_representation};source_topic={self.occupancy_topic};"
            f"pointcloud_topic_3d={self.pointcloud_topic_3d}"
        )
        self.status_pub.publish(String(data=status))
        if status != self.last_status:
            self.get_logger().info(f"Map manager status changed: {status}")
            self.last_status = status

    def write_nav2_map(self, msg, map_dir: Path):
        width = msg.info.width
        height = msg.info.height
        data = list(msg.data)
        image_path = map_dir / "map.pgm"
        yaml_path = map_dir / "map.yaml"

        with image_path.open("wb") as handle:
            handle.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
            for row in range(height - 1, -1, -1):
                for col in range(width):
                    value = data[row * width + col]
                    if value < 0:
                        pixel = 205
                    elif value >= 65:
                        pixel = 0
                    else:
                        pixel = 254
                    handle.write(bytes([pixel]))

        yaml_data = {
            "image": "map.pgm",
            "resolution": float(msg.info.resolution),
            "origin": [
                float(msg.info.origin.position.x),
                float(msg.info.origin.position.y),
                0.0,
            ],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25,
            "mode": "trinary",
        }
        with yaml_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(yaml_data, handle, sort_keys=False)

    def write_pointcloud_snapshot(self, msg: PointCloud2, map_dir: Path) -> dict:
        pcd_path = map_dir / "front_lidar_snapshot.pcd"
        x_field = next((field for field in msg.fields if field.name == "x"), None)
        y_field = next((field for field in msg.fields if field.name == "y"), None)
        z_field = next((field for field in msg.fields if field.name == "z"), None)
        if x_field is None or y_field is None or z_field is None:
            raise RuntimeError("pointcloud missing x/y/z fields")
        if x_field.datatype != 7 or y_field.datatype != 7 or z_field.datatype != 7:
            raise RuntimeError(
                "only FLOAT32 x/y/z pointclouds are supported for snapshot export"
            )

        total_points = int(msg.width) * int(msg.height)
        if total_points <= 0:
            raise RuntimeError("pointcloud contains no points")

        sample_stride = max(
            1, int(math.ceil(total_points / max(1, self.pointcloud_max_points)))
        )
        endian = ">" if msg.is_bigendian else "<"
        unpack_float = struct.Struct(f"{endian}f").unpack_from
        valid_points = []
        raw = memoryview(msg.data)

        for point_index in range(0, total_points, sample_stride):
            base = point_index * msg.point_step
            x = unpack_float(raw, base + x_field.offset)[0]
            y = unpack_float(raw, base + y_field.offset)[0]
            z = unpack_float(raw, base + z_field.offset)[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            valid_points.append((x, y, z))

        with pcd_path.open("w", encoding="ascii") as handle:
            handle.write("# .PCD v0.7 - Point Cloud Data file format\n")
            handle.write("VERSION 0.7\n")
            handle.write("FIELDS x y z\n")
            handle.write("SIZE 4 4 4\n")
            handle.write("TYPE F F F\n")
            handle.write("COUNT 1 1 1\n")
            handle.write(f"WIDTH {len(valid_points)}\n")
            handle.write("HEIGHT 1\n")
            handle.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            handle.write(f"POINTS {len(valid_points)}\n")
            handle.write("DATA ascii\n")
            for x, y, z in valid_points:
                handle.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

        stamp = msg.header.stamp
        return {
            "kind": "pointcloud_snapshot_3d",
            "topic": self.pointcloud_topic_3d,
            "path": pcd_path.name,
            "frame_id": msg.header.frame_id,
            "stamp_sec": int(stamp.sec),
            "stamp_nanosec": int(stamp.nanosec),
            "points_total": total_points,
            "points_saved": len(valid_points),
            "sample_stride": sample_stride,
        }


def main():
    rclpy.init()
    node = MapManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
