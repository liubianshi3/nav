#!/usr/bin/env python3
"""
Convert /a2/traversability (OccupancyGrid) to PointCloud2 for Nav2 obstacle_layer.

The ground segmentation node publishes a traversability grid (V1 binary 0/100,
or V2 graded 0-100). Cells with value >= lethal_threshold represent
non-traversable terrain. Unknown cells default to NOT generating obstacle points.

Published topics:
  /a2/traversability/obstacle_points (sensor_msgs/PointCloud2, frame base_link)

V2 changes:
  - unknown_policy: "ignore" (skip -1 cells), "lethal" (treat as obstacle), "soft_cost"
  - lethal_threshold: cells >= this value become obstacle points
  - publish_unknown_as_obstacle: legacy parameter, kept for backward compat
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def _make_pointcloud2(
    points: np.ndarray,
    stamp: rclpy.time.Time,
    frame_id: str,
) -> PointCloud2:
    """Create a PointCloud2 message from an Nx3 numpy array."""
    if points.size == 0:
        msg = PointCloud2()
        msg.header = Header(stamp=stamp, frame_id=frame_id)
        msg.height = 1
        msg.width = 0
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 0
        msg.is_dense = True
        return msg

    return point_cloud2.create_cloud_xyz32(
        Header(stamp=stamp, frame_id=frame_id),
        points.tolist(),
    )


class TraversabilityToObstacleCloud(Node):
    """Convert traversability OccupancyGrid to obstacle PointCloud2."""

    def __init__(self) -> None:
        super().__init__("traversability_to_obstacle_cloud")

        self._obstacle_threshold = int(
            self.declare_parameter("traversability_obstacle_threshold", 90).value
        )
        self._lethal_threshold = int(
            self.declare_parameter("lethal_threshold", 70).value
        )
        self._publish_hz = float(
            self.declare_parameter("publish_hz", 2.0).value
        )
        self._obstacle_z = float(
            self.declare_parameter("obstacle_z", 0.15).value
        )
        self._traversability_topic = str(
            self.declare_parameter("traversability_topic", "/a2/traversability").value
        )
        self._output_topic = str(
            self.declare_parameter("output_topic", "/a2/traversability/obstacle_points").value
        )
        self._output_frame = str(
            self.declare_parameter("output_frame", "base_link").value
        )
        self._treat_unknown_as_obstacle = bool(
            self.declare_parameter("treat_unknown_as_obstacle", False).value
        )
        self._unknown_policy = str(
            self.declare_parameter("unknown_policy", "ignore").value
        )
        self._publish_unknown_as_obstacle = bool(
            self.declare_parameter("publish_unknown_as_obstacle", False).value
        )
        self._transform_timeout_sec = float(
            self.declare_parameter("transform_timeout_sec", 0.2).value
        )
        self._local_window_enabled = bool(
            self.declare_parameter("local_window_enabled", True).value
        )
        self._local_min_x = float(self.declare_parameter("local_min_x", -1.0).value)
        self._local_max_x = float(self.declare_parameter("local_max_x", 6.0).value)
        self._local_min_y = float(self.declare_parameter("local_min_y", -4.0).value)
        self._local_max_y = float(self.declare_parameter("local_max_y", 4.0).value)
        self._self_filter_enabled = bool(
            self.declare_parameter("self_filter_enabled", True).value
        )
        self._self_filter_min_x = float(
            self.declare_parameter("self_filter_min_x", -0.70).value
        )
        self._self_filter_max_x = float(
            self.declare_parameter("self_filter_max_x", 0.95).value
        )
        self._self_filter_min_y = float(
            self.declare_parameter("self_filter_min_y", -0.55).value
        )
        self._self_filter_max_y = float(
            self.declare_parameter("self_filter_max_y", 0.55).value
        )
        self._self_filter_min_z = float(
            self.declare_parameter("self_filter_min_z", -0.30).value
        )
        self._self_filter_max_z = float(
            self.declare_parameter("self_filter_max_z", 0.90).value
        )
        self._max_output_points = int(
            self.declare_parameter("max_output_points", 20000).value
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._last_grid: OccupancyGrid | None = None

        self._sub = self.create_subscription(
            OccupancyGrid, self._traversability_topic, self._on_grid, 10
        )
        self._pub = self.create_publisher(PointCloud2, self._output_topic, 10)
        self._timer = self.create_timer(1.0 / max(self._publish_hz, 0.1), self._publish)

        self.get_logger().info(
            "TraversabilityToObstacleCloud started: "
            f"lethal_threshold={self._lethal_threshold} (legacy={self._obstacle_threshold}) "
            f"z={self._obstacle_z:.2f} hz={self._publish_hz:.1f} "
            f"input={self._traversability_topic} output={self._output_topic} "
            f"output_frame={self._output_frame} unknown_policy={self._unknown_policy} "
            f"self_filter={str(self._self_filter_enabled).lower()}"
        )

    def _on_grid(self, msg: OccupancyGrid) -> None:
        self._last_grid = msg

    def _publish_empty(self) -> None:
        empty = _make_pointcloud2(
            np.zeros((0, 3), dtype=np.float32),
            self.get_clock().now().to_msg(),
            self._output_frame,
        )
        self._pub.publish(empty)

    @staticmethod
    def _rotation_matrix_from_quaternion(x: float, y: float, z: float, w: float) -> np.ndarray:
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 0.0:
            return np.eye(3, dtype=np.float64)
        x /= norm
        y /= norm
        z /= norm
        w /= norm
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z
        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float64,
        )

    def _transform_points(self, points: np.ndarray, source_frame: str) -> np.ndarray | None:
        if source_frame == self._output_frame:
            return points
        try:
            transform = self._tf_buffer.lookup_transform(
                self._output_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self._transform_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Traversability transform unavailable: "
                f"{source_frame}->{self._output_frame}: {exc}"
            )
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        rot = self._rotation_matrix_from_quaternion(q.x, q.y, q.z, q.w)
        trans = np.array([t.x, t.y, t.z], dtype=np.float64)
        return (points.astype(np.float64) @ rot.T + trans).astype(np.float32)

    def _apply_local_window(self, points: np.ndarray) -> np.ndarray:
        if not self._local_window_enabled or points.size == 0:
            return points
        mask = (
            (points[:, 0] >= self._local_min_x)
            & (points[:, 0] <= self._local_max_x)
            & (points[:, 1] >= self._local_min_y)
            & (points[:, 1] <= self._local_max_y)
        )
        return points[mask]

    def _apply_self_filter(self, points: np.ndarray) -> np.ndarray:
        if not self._self_filter_enabled or points.size == 0:
            return points
        inside_body = (
            (points[:, 0] >= self._self_filter_min_x)
            & (points[:, 0] <= self._self_filter_max_x)
            & (points[:, 1] >= self._self_filter_min_y)
            & (points[:, 1] <= self._self_filter_max_y)
            & (points[:, 2] >= self._self_filter_min_z)
            & (points[:, 2] <= self._self_filter_max_z)
        )
        return points[~inside_body]

    def _publish(self) -> None:
        if self._last_grid is None:
            return

        grid = self._last_grid
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin = grid.info.origin

        if width == 0 or height == 0:
            return

        data = np.array(grid.data, dtype=np.int8).reshape((height, width))

        # Determine effective threshold: use lethal_threshold for V2 graded grids,
        # fall back to obstacle_threshold for V1 binary grids.
        threshold = self._lethal_threshold if self._lethal_threshold > 0 else self._obstacle_threshold

        # Lethal obstacle mask: cells with value >= threshold.
        obstacle_mask = data >= threshold
        mask = obstacle_mask

        # Unknown cells (data == -1): handled per unknown_policy.
        if data.dtype == np.int8:
            unknown_mask = data == -1
        else:
            unknown_mask = np.zeros_like(data, dtype=bool)

        # Legacy compat: treat_unknown_as_obstacle=True overrides policy.
        if self._treat_unknown_as_obstacle or self._publish_unknown_as_obstacle:
            mask = obstacle_mask | unknown_mask
        elif self._unknown_policy == "lethal":
            mask = obstacle_mask | unknown_mask
        elif self._unknown_policy == "soft_cost":
            # Soft cost: exclude unknown from obstacle output, but don't hard-block.
            # Unknown cells stay out of the obstacle cloud.
            mask = obstacle_mask
        else:
            # Default: "ignore" — unknown cells do not become obstacles.
            mask = obstacle_mask

        ys, xs = np.where(mask)
        if len(xs) == 0:
            self._publish_empty()
            return

        # Convert grid indices to map-frame coordinates
        x_coords = origin.position.x + (xs.astype(np.float64) + 0.5) * resolution
        y_coords = origin.position.y + (ys.astype(np.float64) + 0.5) * resolution
        z_coords = np.full_like(x_coords, self._obstacle_z)

        points_in_grid_frame = np.column_stack((x_coords, y_coords, z_coords)).astype(np.float32)
        points = self._transform_points(points_in_grid_frame, grid.header.frame_id)
        if points is None:
            self._publish_empty()
            return

        points = self._apply_self_filter(points)
        if points.size == 0:
            self._publish_empty()
            return

        points = self._apply_local_window(points)
        if points.size == 0:
            self._publish_empty()
            return

        if self._max_output_points > 0 and len(points) > self._max_output_points:
            stride = int(math.ceil(len(points) / self._max_output_points))
            points = points[::stride]

        cloud = _make_pointcloud2(points, self.get_clock().now().to_msg(), self._output_frame)
        self._pub.publish(cloud)


def main() -> None:
    rclpy.init()
    node = TraversabilityToObstacleCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
