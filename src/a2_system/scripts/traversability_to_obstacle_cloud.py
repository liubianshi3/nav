#!/usr/bin/env python3
"""
Convert /a2/traversability (OccupancyGrid) to PointCloud2 for Nav2 obstacle_layer.

The ground segmentation node publishes a 2D traversability grid in the "map"
frame. Cells with value >= threshold (default 90) represent steep/non-traversable
terrain. This node converts those cells to 3D obstacle points so the Nav2
costmap obstacle_layer can consume them alongside the live pointcloud.

Published topics:
  /a2/traversability/obstacle_points (sensor_msgs/PointCloud2, frame "map")
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2


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
            self.declare_parameter("output_frame", "map").value
        )
        self._treat_unknown_as_obstacle = bool(
            self.declare_parameter("treat_unknown_as_obstacle", False).value
        )

        self._last_grid: OccupancyGrid | None = None

        self._sub = self.create_subscription(
            OccupancyGrid, self._traversability_topic, self._on_grid, 10
        )
        self._pub = self.create_publisher(PointCloud2, self._output_topic, 10)
        self._timer = self.create_timer(1.0 / max(self._publish_hz, 0.1), self._publish)

        self.get_logger().info(
            "TraversabilityToObstacleCloud started: "
            "threshold=%d z=%.2f hz=%.1f input=%s output=%s",
            self._obstacle_threshold, self._obstacle_z, self._publish_hz,
            self._traversability_topic, self._output_topic,
        )

    def _on_grid(self, msg: OccupancyGrid) -> None:
        self._last_grid = msg

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

        # Cells that are obstacle (>= threshold). Unknown cells are only
        # merged when treat_unknown_as_obstacle is explicitly enabled,
        # keeping unmapped areas from permanently polluting obstacle_points.
        obstacle_mask = data >= self._obstacle_threshold
        if self._treat_unknown_as_obstacle:
            unknown_mask = data == -1
            mask = obstacle_mask | unknown_mask
        else:
            mask = obstacle_mask

        ys, xs = np.where(mask)
        if len(xs) == 0:
            # Publish empty cloud to clear previous obstacles
            empty = _make_pointcloud2(
                np.zeros((0, 3), dtype=np.float32),
                self.get_clock().now(),
                self._output_frame,
            )
            self._pub.publish(empty)
            return

        # Convert grid indices to map-frame coordinates
        x_coords = origin.position.x + (xs.astype(np.float64) + 0.5) * resolution
        y_coords = origin.position.y + (ys.astype(np.float64) + 0.5) * resolution
        z_coords = np.full_like(x_coords, self._obstacle_z)

        points = np.column_stack((x_coords, y_coords, z_coords)).astype(np.float32)

        cloud = _make_pointcloud2(points, grid.header.stamp, self._output_frame)
        self._pub.publish(cloud)


def main() -> None:
    rclpy.init()
    node = TraversabilityToObstacleCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
