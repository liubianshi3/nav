#!/usr/bin/env python3
"""Stable global traversability integrator for global planner feedback.

Subscribes to /a2/traversability (OccupancyGrid, frame map) and applies
temporal cooldown filtering before publishing verified obstacles as a
PointCloud2 layer for Nav2 global_costmap.

Field hardening:
  - TF unavailable → skip memory update, publish empty cloud, status tf_error.
  - Frame mismatch → skip memory update, status frame_error.
  - Costmap output published for debugging on /a2/global_traversability/costmap.
"""

from __future__ import annotations

import math
import time
from collections import OrderedDict
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class CellState:
    col: int
    row: int
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


def validate_frame(input_frame_id: str, expected_frame_id: str) -> Tuple[bool, str]:
    """Pure function: check that input frame matches expected frame.

    Returns (ok, reason).  ok=False means the grid must not be consumed.
    """
    inp = (input_frame_id or "").strip()
    exp = (expected_frame_id or "").strip()
    if not inp:
        return False, "empty_input_frame"
    if inp != exp:
        return False, f"frame_mismatch:{inp}!={exp}"
    return True, "ok"


def should_update_with_tf(local_window_enabled: bool, tf_ok: bool) -> Tuple[bool, str]:
    """Pure function: decide whether to update memory given TF state.

    Returns (should_update, reason).
    """
    if not local_window_enabled:
        return True, "ok"
    if tf_ok:
        return True, "ok"
    return False, "waiting_tf"


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

    @property
    def last_grid(self) -> Optional[OccupancyGrid]:
        return self._last_grid

    @property
    def last_grid_time(self) -> float:
        return self._last_grid_time

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
                    cell = CellState(col=col, row=row, x=cell_x, y=cell_y)
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

    def get_stable_obstacle_cells(self) -> List[CellState]:
        """Return cells that meet stability criteria."""
        now = time.monotonic()
        result: List[CellState] = []
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
                result.append(cell)
        self._stats.stable_obstacle_cells = len(result)
        return result

    def get_stable_obstacle_points(self) -> np.ndarray:
        cells = self.get_stable_obstacle_cells()
        points = [(cell.x, cell.y, 0.15) for cell in cells]

        result = np.array(points, dtype=np.float32) if points else np.zeros((0, 3), dtype=np.float32)
        if self.max_points > 0 and len(result) > self.max_points:
            stride = max(1, int(math.ceil(len(result) / self.max_points)))
            result = result[::stride]

        self._stats.output_points = len(result)
        return result

    def build_stable_costmap(self) -> Optional[OccupancyGrid]:
        """Build an OccupancyGrid with stable obstacles set to 100, others 0."""
        if self._last_grid is None:
            return None

        info = self._last_grid.info
        width = info.width
        height = info.height
        if width == 0 or height == 0:
            return None

        data = [0] * (width * height)
        for cell in self.get_stable_obstacle_cells():
            if 0 <= cell.row < height and 0 <= cell.col < width:
                data[cell.row * width + cell.col] = 100

        grid = OccupancyGrid()
        grid.header = Header(frame_id=self._stats.output_frame or "map")
        grid.info = info
        grid.data = data
        return grid

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


def _make_empty_costmap(frame_id: str) -> OccupancyGrid:
    grid = OccupancyGrid()
    grid.header = Header(frame_id=frame_id)
    grid.info = MapMetaData(resolution=0.1, width=1, height=1)
    grid.info.origin.orientation.w = 1.0
    grid.data = [0]
    return grid


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
        self._local_update_window_enabled = bool(
            self.declare_parameter("local_update_window_enabled", True).value
        )

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
            local_update_window_enabled=self._local_update_window_enabled,
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
        self._costmap_pub = self.create_publisher(OccupancyGrid, self._output_costmap_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._timer = self.create_timer(1.0 / max(self._publish_hz, 0.1), self._publish)

        self._last_publish_time: float = 0.0
        self._grid_count: int = 0

        self.get_logger().info(
            f"GlobalTraversabilityIntegrator started: "
            f"input={self._input_topic} output={self._output_obstacle_topic} "
            f"costmap={self._output_costmap_topic} frame={self._frame_id} "
            f"min_obs={self._memory.min_observations} "
            f"min_conf={self._memory.min_confidence:.2f} "
            f"stale_clear={self._memory.stale_clear_sec:.0f}s "
            f"unknown_policy={self._memory.unknown_policy} "
            f"local_window={self._local_update_window_enabled}"
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
            err_str = str(exc)
            if self._tf_fail_reason != err_str:
                self._tf_fail_reason = err_str
                self.get_logger().warn(
                    f"TF lookup map→base_link failed: {self._tf_fail_reason}",
                    throttle_duration_sec=10.0,
                )
            self._tf_ok = False
            return self._last_robot_x, self._last_robot_y, False

    def _validate_frame(self, msg: OccupancyGrid) -> bool:
        input_frame = (msg.header.frame_id or "").strip()
        expected = self._frame_id.strip()
        if not input_frame:
            self._memory._stats.state = "frame_error"
            self._memory._stats.ready = False
            self._memory._stats.reason = "empty_input_frame"
            return False
        if input_frame != expected:
            self._memory._stats.state = "frame_error"
            self._memory._stats.ready = False
            self._memory._stats.reason = f"frame_mismatch:{input_frame}!={expected}"
            self.get_logger().warn(
                f"Frame mismatch: got '{input_frame}', expected '{expected}'. "
                f"Grid will not be consumed.",
                throttle_duration_sec=10.0,
            )
            return False
        return True

    def _on_odom(self, msg: Odometry) -> None:
        pass  # Reserved for future use; primary pose source is TF.

    def _on_grid(self, msg: OccupancyGrid) -> None:
        if not self._validate_frame(msg):
            return

        rx, ry, tf_ok = self._lookup_robot_pose()

        if tf_ok:
            self._last_robot_x = rx
            self._last_robot_y = ry

        should_update, reason = should_update_with_tf(
            self._local_update_window_enabled, tf_ok
        )
        if not should_update:
            self._memory._stats.state = "tf_error"
            self._memory._stats.ready = False
            self._memory._stats.reason = self._tf_fail_reason or reason
            self._memory._stats.input_frame = msg.header.frame_id
            self._memory._stats.output_frame = self._frame_id
            return

        self._memory.update(msg, self._last_robot_x, self._last_robot_y)
        self._grid_count += 1

        # Reset state on every successful update so we recover from
        # transient tf_error / frame_error without needing a restart.
        if self._grid_count == 1:
            self._memory.set_ready()
        self._memory._stats.state = "active"
        self._memory._stats.ready = True
        self._memory._stats.reason = "ok"

    def _publish(self) -> None:
        now = time.monotonic()

        if self._memory._last_grid_time > 0:
            self._memory._stats.input_age_sec = now - self._memory._last_grid_time
        self._memory._stats.last_publish_age_sec = (
            now - self._last_publish_time if self._last_publish_time > 0 else 0.0
        )

        # TF error or frame error → publish empty cloud / empty costmap so
        # downstream global_costmap stops consuming stale stable points.
        # Still apply decay on tf_error so points fade if TF stays bad.
        tf_blocked = not self._tf_ok and self._local_update_window_enabled
        frame_blocked = self._memory._stats.state == "frame_error"
        if tf_blocked or frame_blocked:
            if tf_blocked and not frame_blocked:
                self._memory._stats.state = "tf_error"
                self._memory._stats.ready = False
                self._memory._stats.reason = self._tf_fail_reason or "waiting_tf"
                self._memory.apply_decay()
            stamp = self.get_clock().now().to_msg()
            self._obstacle_pub.publish(_make_empty_pointcloud(stamp, self._frame_id))
            empty_costmap = _make_empty_costmap(self._frame_id)
            empty_costmap.header.stamp = stamp
            self._costmap_pub.publish(empty_costmap)
            self._status_pub.publish(String(data=self._memory._stats.status_string()))
            self._last_publish_time = now
            return

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

        # Publish costmap for debugging
        costmap = self._memory.build_stable_costmap()
        if costmap is not None:
            costmap.header.stamp = stamp
            self._costmap_pub.publish(costmap)
        else:
            self._costmap_pub.publish(_make_empty_costmap(self._frame_id))

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
