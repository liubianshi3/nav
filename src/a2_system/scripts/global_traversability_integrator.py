#!/usr/bin/env python3
"""Stable global traversability integrator for global planner feedback.

Subscribes to /a2/traversability (OccupancyGrid, frame map) and applies
temporal cooldown filtering before publishing verified obstacles as a
PointCloud2 layer for Nav2 global_costmap.

Design:
  - Unknown cells (value == -1) are ignored by default (unknown_policy=ignore).
  - Single-frame high-cost cells are NOT forwarded — only cells that have been
    observed as high-cost across *min_observations* frames with confidence >=
    *min_confidence* are emitted.
  - Stale cells are cleared after *stale_clear_sec* without observation.
  - Confidence decays over time for cells not currently observed as high-cost.
  - local_update_window limits processing to a radius around the robot pose.
"""

from __future__ import annotations

import math
import time
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class CellState:
    x: float
    y: float
    observation_count: int = 0
    high_cost_count: int = 0
    confidence: float = 0.0
    last_seen_time: float = 0.0
    last_high_cost_time: float = 0.0
    stable_cost: int = 0


@dataclass
class IntegratorStats:
    known_cells: int = 0
    stable_obstacle_cells: int = 0
    stale_cells: int = 0
    dropped_unknown_cells: int = 0
    output_points: int = 0
    input_age_sec: float = 0.0
    input_frame: str = ""
    output_frame: str = ""
    last_publish_age_sec: float = 0.0
    state: str = "idle"
    ready: bool = False
    reason: str = "startup"

    def status_string(self) -> str:
        return (
            f"state={self.state};ready={str(self.ready).lower()};reason={self.reason};"
            f"input_age_sec={self.input_age_sec:.2f};input_frame={self.input_frame};"
            f"output_frame={self.output_frame};known_cells={self.known_cells};"
            f"stable_obstacle_cells={self.stable_obstacle_cells};stale_cells={self.stale_cells};"
            f"dropped_unknown_cells={self.dropped_unknown_cells};output_points={self.output_points};"
            f"last_publish_age_sec={self.last_publish_age_sec:.2f}"
        )


class GlobalTraversabilityMemory:
    """Pure-Python temporal cooldown and stability logic.  Testable without ROS."""

    def __init__(
        self,
        high_cost_threshold: int = 90,
        lethal_threshold: int = 70,
        min_observations: int = 3,
        min_confidence: float = 0.6,
        confidence_increment: float = 0.25,
        confidence_decay: float = 0.10,
        observation_decay_sec: float = 20.0,
        stale_clear_sec: float = 60.0,
        unknown_policy: str = "ignore",
        local_update_window_enabled: bool = True,
        local_update_radius_m: float = 8.0,
        max_points: int = 50000,
    ) -> None:
        self.high_cost_threshold = high_cost_threshold
        self.lethal_threshold = lethal_threshold
        self.min_observations = min_observations
        self.min_confidence = min_confidence
        self.confidence_increment = confidence_increment
        self.confidence_decay = confidence_decay
        self.observation_decay_sec = observation_decay_sec
        self.stale_clear_sec = stale_clear_sec
        self.unknown_policy = unknown_policy
        self.local_update_window_enabled = local_update_window_enabled
        self.local_update_radius_m = local_update_radius_m
        self.max_points = max_points

        self._cells: Dict[Tuple[int, int], CellState] = OrderedDict()
        self._last_grid: Optional[OccupancyGrid] = None
        self._last_grid_time: float = 0.0
        self._stats = IntegratorStats()

    @property
    def stats(self) -> IntegratorStats:
        return self._stats

    def update(self, grid: OccupancyGrid, robot_x: float = 0.0, robot_y: float = 0.0) -> None:
        now = time.monotonic()
        self._last_grid = grid
        self._last_grid_time = now
        self._stats.input_age_sec = 0.0
        self._stats.input_frame = grid.header.frame_id

        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        if width == 0 or height == 0:
            return

        data = np.array(grid.data, dtype=np.int8).reshape((height, width))
        dropped_unknown = 0

        for row in range(height):
            for col in range(width):
                value = int(data[row, col])
                if value == -1:
                    if self.unknown_policy == "ignore":
                        dropped_unknown += 1
                        continue
                    # unknown_policy != "ignore" → treat as potential obstacle
                    value = 99

                cell_x = origin_x + (col + 0.5) * resolution
                cell_y = origin_y + (row + 0.5) * resolution

                if self.local_update_window_enabled:
                    dist_sq = (cell_x - robot_x) ** 2 + (cell_y - robot_y) ** 2
                    if dist_sq > self.local_update_radius_m ** 2:
                        continue

                key = (col, row)
                cell = self._cells.get(key)
                if cell is None:
                    cell = CellState(x=cell_x, y=cell_y)
                    self._cells[key] = cell

                cell.observation_count += 1
                cell.last_seen_time = now

                if value >= self.high_cost_threshold:
                    cell.high_cost_count += 1
                    cell.last_high_cost_time = now
                    cell.confidence = min(1.0, cell.confidence + self.confidence_increment)
                    cell.stable_cost = max(cell.stable_cost, min(100, value))
                elif value >= self.lethal_threshold:
                    cell.observation_count = max(cell.observation_count, 1)
                    cell.confidence = max(0.0, cell.confidence - self.confidence_decay * 0.5)
                else:
                    cell.confidence = max(0.0, cell.confidence - self.confidence_decay)

        self._stats.dropped_unknown_cells += dropped_unknown

    def apply_decay(self) -> None:
        now = time.monotonic()
        stale_keys = []

        for key, cell in list(self._cells.items()):
            age = now - cell.last_seen_time
            if age > self.stale_clear_sec:
                stale_keys.append(key)
                continue
            if age > self.observation_decay_sec:
                cell.confidence = max(0.0, cell.confidence - self.confidence_decay)

        for key in stale_keys:
            self._cells.pop(key, None)
        self._stats.stale_cells = len(stale_keys)
        self._stats.known_cells = len(self._cells)

    def get_stable_obstacle_points(self) -> np.ndarray:
        now = time.monotonic()
        points = []
        stable_count = 0

        for key, cell in self._cells.items():
            stale = (now - cell.last_seen_time) > self.stale_clear_sec
            if stale:
                continue

            is_stable = (
                cell.high_cost_count >= self.min_observations
                and cell.confidence >= self.min_confidence
                and cell.high_cost_count > 0
            )

            if is_stable:
                points.append((cell.x, cell.y, 0.15))
                stable_count += 1

        self._stats.stable_obstacle_cells = stable_count

        result = np.array(points, dtype=np.float32)
        if self.max_points > 0 and len(result) > self.max_points:
            stride = max(1, int(math.ceil(len(result) / self.max_points)))
            result = result[::stride]

        self._stats.output_points = len(result)
        return result

    def set_ready(self) -> None:
        self._stats.ready = True
        if self._stats.state == "idle":
            self._stats.state = "active"


def _make_empty_pointcloud(stamp: rclpy.time.Time, frame_id: str) -> PointCloud2:
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


class GlobalTraversabilityIntegrator(Node):
    """ROS node wrapper around GlobalTraversabilityMemory."""

    def __init__(self) -> None:
        super().__init__("global_traversability_integrator")

        self._input_topic = str(
            self.declare_parameter("input_topic", "/a2/traversability").value
        )
        self._output_obstacle_topic = str(
            self.declare_parameter("output_obstacle_topic", "/a2/global_traversability/obstacle_points").value
        )
        self._output_costmap_topic = str(
            self.declare_parameter("output_costmap_topic", "/a2/global_traversability/costmap").value
        )
        self._status_topic = str(
            self.declare_parameter("status_topic", "/a2/global_traversability/status").value
        )
        self._frame_id = str(self.declare_parameter("frame_id", "map").value)
        self._publish_hz = float(self.declare_parameter("publish_hz", 1.0).value)
        self._transform_timeout_sec = float(
            self.declare_parameter("transform_timeout_sec", 0.2).value
        )
        self._odom_topic = str(self.declare_parameter("odom_topic", "/odometry/local").value)

        self._memory = GlobalTraversabilityMemory(
            high_cost_threshold=int(self.declare_parameter("high_cost_threshold", 90).value),
            lethal_threshold=int(self.declare_parameter("lethal_threshold", 70).value),
            min_observations=int(self.declare_parameter("min_observations", 3).value),
            min_confidence=float(self.declare_parameter("min_confidence", 0.6).value),
            confidence_increment=float(self.declare_parameter("confidence_increment", 0.25).value),
            confidence_decay=float(self.declare_parameter("confidence_decay", 0.10).value),
            observation_decay_sec=float(self.declare_parameter("observation_decay_sec", 20.0).value),
            stale_clear_sec=float(self.declare_parameter("stale_clear_sec", 60.0).value),
            unknown_policy=str(self.declare_parameter("unknown_policy", "ignore").value),
            local_update_window_enabled=bool(
                self.declare_parameter("local_update_window_enabled", True).value
            ),
            local_update_radius_m=float(
                self.declare_parameter("local_update_radius_m", 8.0).value
            ),
            max_points=int(self.declare_parameter("max_points", 50000).value),
        )

        self._memory._stats.output_frame = self._frame_id
        self._memory._stats.state = "idle"
        self._memory._stats.reason = "startup"

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_robot_x: float = 0.0
        self._last_robot_y: float = 0.0
        self._tf_ok: bool = False
        self._tf_fail_reason: str = "waiting_tf"

        self._sub = self.create_subscription(
            OccupancyGrid, self._input_topic, self._on_grid, 10
        )
        self._sub_odom = self.create_subscription(
            Odometry, self._odom_topic, self._on_odom, 10
        )
        self._obstacle_pub = self.create_publisher(PointCloud2, self._output_obstacle_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._timer = self.create_timer(1.0 / max(self._publish_hz, 0.1), self._publish)

        self._last_publish_time: float = 0.0
        self._grid_count: int = 0

        self.get_logger().info(
            f"GlobalTraversabilityIntegrator started: "
            f"input={self._input_topic} output={self._output_obstacle_topic} "
            f"frame={self._frame_id} min_obs={self._memory.min_observations} "
            f"min_conf={self._memory.min_confidence:.2f} "
            f"stale_clear={self._memory.stale_clear_sec:.0f}s "
            f"unknown_policy={self._memory.unknown_policy}"
        )

    def _lookup_robot_pose(self) -> Tuple[float, float, bool]:
        try:
            transform: TransformStamped = self._tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=self._transform_timeout_sec),
            )
            self._tf_ok = True
            self._tf_fail_reason = ""
            return (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                True,
            )
        except TransformException as exc:
            if self._tf_fail_reason != str(exc):
                self._tf_fail_reason = str(exc)
                self.get_logger().warn(
                    f"TF lookup map→base_link failed: {self._tf_fail_reason}",
                    throttle_duration_sec=10.0,
                )
            self._tf_ok = False
            return self._last_robot_x, self._last_robot_y, False

    def _on_odom(self, msg: Odometry) -> None:
        pass  # Reserved for future use; primary pose source is TF.

    def _on_grid(self, msg: OccupancyGrid) -> None:
        rx, ry, tf_ok = self._lookup_robot_pose()
        if tf_ok:
            self._last_robot_x = rx
            self._last_robot_y = ry

        self._memory.update(msg, self._last_robot_x, self._last_robot_y)
        self._grid_count += 1

        if self._grid_count == 1:
            self._memory.set_ready()
            self._memory._stats.reason = "first_grid"

    def _publish(self) -> None:
        now = time.monotonic()

        # Update stats
        if self._memory._last_grid_time > 0:
            self._memory._stats.input_age_sec = now - self._memory._last_grid_time
        self._memory._stats.last_publish_age_sec = now - self._last_publish_time if self._last_publish_time > 0 else 0.0

        if not self._tf_ok:
            self._memory._stats.state = "tf_error"
            self._memory._stats.reason = self._tf_fail_reason

        self._memory.apply_decay()
        points = self._memory.get_stable_obstacle_points()

        stamp = self.get_clock().now().to_msg()

        # Publish obstacle pointcloud
        if points.size == 0:
            cloud = _make_empty_pointcloud(stamp, self._frame_id)
        else:
            cloud = point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id=self._frame_id), points.tolist()
            )
        self._obstacle_pub.publish(cloud)

        # Publish status
        self._status_pub.publish(String(data=self._memory._stats.status_string()))

        self._last_publish_time = now


def main() -> None:
    rclpy.init()
    node = GlobalTraversabilityIntegrator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
