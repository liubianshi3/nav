#!/usr/bin/env python3
"""
Ray-based ground segmentation for 3D point clouds.

Algorithm adapted from Autoware's ray_ground_filter, implemented in pure
Python/NumPy for zero-compilation integration.  The core idea:

1. Convert Cartesian (x,y,z) → cylindrical (radius, theta, z).
2. Divide 360° into angular sectors.
3. Sort points within each sector by radius (near → far).
4. Sweep each sector: a point is ground if its z-height stays within a
   slope-defined envelope relative to the previous ground point.
5. Also build a 2.5D traversability grid from the ground points.

This node publishes three outputs:
  /a2/obstacle/points  — non-ground points (feed to occupancy mapper)
  /a2/ground/points    — ground points (feed to traversability analysis)
  /a2/traversability   — 2D grid encoding height, slope, and roughness
"""

from __future__ import annotations

import math
import struct
from typing import Tuple

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def _parse_pointcloud2(msg: PointCloud2) -> np.ndarray:
    """Extract (x, y, z) as an (N,3) float64 array.  Returns empty on failure."""
    pts = []
    for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            pts.append((x, y, z))
    if not pts:
        return np.empty((0, 3), dtype=np.float64)
    return np.array(pts, dtype=np.float64)


def _build_pointcloud2(points: np.ndarray, header: Header, frame_id: str) -> PointCloud2:
    """Pack an (N,3) float64 array into a PointCloud2 message."""
    if points.size == 0:
        points = np.empty((0, 3), dtype=np.float64)
    buf = points.astype(np.float32).tobytes()
    pc = PointCloud2()
    pc.header = header
    pc.header.frame_id = frame_id
    pc.height = 1
    pc.width = points.shape[0]
    pc.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    pc.point_step = 12
    pc.row_step = pc.point_step * pc.width
    pc.is_bigendian = False
    pc.is_dense = True
    pc.data = buf
    return pc


def _build_occupancy_grid(
    data: np.ndarray,
    resolution: float,
    origin_x: float,
    origin_y: float,
    header: Header,
    frame_id: str,
) -> OccupancyGrid:
    grid = OccupancyGrid()
    grid.header = header
    grid.header.frame_id = frame_id
    grid.info.resolution = float(resolution)
    grid.info.width = data.shape[1]
    grid.info.height = data.shape[0]
    grid.info.origin.position.x = float(origin_x)
    grid.info.origin.position.y = float(origin_y)
    grid.info.origin.orientation.w = 1.0
    grid.data = [int(v) for v in data.ravel()]
    return grid


class GroundSegmentationNode(Node):
    """Ray-based ground / obstacle separation + 2.5D traversability."""

    def __init__(self) -> None:
        super().__init__("ground_segmentation")

        # -- input -----------------------------------------------------------
        self.input_topic = self.declare_parameter(
            "input_topic", "/jt128/front/points"
        ).value

        # -- output topics ---------------------------------------------------
        self.ground_topic = self.declare_parameter(
            "ground_topic", "/a2/ground/points"
        ).value
        self.obstacle_topic = self.declare_parameter(
            "obstacle_topic", "/a2/obstacle/points"
        ).value
        self.traversability_topic = self.declare_parameter(
            "traversability_topic", "/a2/traversability"
        ).value

        # -- ray ground filter parameters ------------------------------------
        self.radial_divider_angle = float(
            self.declare_parameter("radial_divider_angle", 1.0).value
        )
        self.general_max_slope_deg = float(
            self.declare_parameter("general_max_slope_deg", 8.0).value
        )
        self.local_max_slope_deg = float(
            self.declare_parameter("local_max_slope_deg", 6.0).value
        )
        self.initial_max_slope_deg = float(
            self.declare_parameter("initial_max_slope_deg", 3.0).value
        )
        self.min_height_threshold = float(
            self.declare_parameter("min_height_threshold", 0.15).value
        )
        self.concentric_divider_distance = float(
            self.declare_parameter("concentric_divider_distance", 0.01).value
        )
        self.reclass_distance_threshold = float(
            self.declare_parameter("reclass_distance_threshold", 0.1).value
        )

        # -- traversability grid parameters ----------------------------------
        self.traversability_resolution = float(
            self.declare_parameter("traversability_resolution", 0.1).value
        )
        self.traversability_width = int(
            self.declare_parameter("traversability_width", 400).value
        )
        self.traversability_height = int(
            self.declare_parameter("traversability_height", 400).value
        )
        self.traversability_origin_x = float(
            self.declare_parameter("traversability_origin_x", -20.0).value
        )
        self.traversability_origin_y = float(
            self.declare_parameter("traversability_origin_y", -20.0).value
        )
        self.max_traversable_slope_deg = float(
            self.declare_parameter("max_traversable_slope_deg", 20.0).value
        )
        self.traversability_publish_hz = float(
            self.declare_parameter("traversability_publish_hz", 1.0).value
        )

        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.process_every_n = int(
            self.declare_parameter("process_every_n", 1).value
        )

        # internal state
        self._skip_count = 0
        self._ground_height_map: np.ndarray | None = None
        self._ground_var_map: np.ndarray | None = None
        self._ground_count_map: np.ndarray | None = None

        # Pre-compute trig constants
        self._tan_general = math.tan(math.radians(self.general_max_slope_deg))
        self._tan_local = math.tan(math.radians(self.local_max_slope_deg))
        self._tan_initial = math.tan(math.radians(self.initial_max_slope_deg))
        self._num_sectors = int(math.ceil(360.0 / max(0.1, self.radial_divider_angle)))

        # Publishers
        self._ground_pub = self.create_publisher(PointCloud2, self.ground_topic, 10)
        self._obstacle_pub = self.create_publisher(PointCloud2, self.obstacle_topic, 10)
        self._traversability_pub = self.create_publisher(
            OccupancyGrid, self.traversability_topic, 10
        )

        # Subscription
        self.create_subscription(PointCloud2, self.input_topic, self._on_cloud, 10)

        # Traversability timer
        period = 1.0 / max(0.1, self.traversability_publish_hz)
        self.create_timer(period, self._publish_traversability)

        self.get_logger().info(
            f"Ground segmentation active: input={self.input_topic}, "
            f"sectors={self._num_sectors}, "
            f"general_slope={self.general_max_slope_deg}°, "
            f"local_slope={self.local_max_slope_deg}°"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._skip_count += 1
        if self._skip_count % max(1, self.process_every_n) != 0:
            return

        points = _parse_pointcloud2(msg)
        if points.shape[0] < 10:
            return

        ground_mask = self._classify_ground(points)
        ground = points[ground_mask]
        obstacle = points[~ground_mask]
        header = msg.header

        self._ground_pub.publish(_build_pointcloud2(ground, header, self.frame_id))
        self._obstacle_pub.publish(_build_pointcloud2(obstacle, header, self.frame_id))
        self._update_traversability(ground)

    # ------------------------------------------------------------------
    #  Ground classification
    # ------------------------------------------------------------------

    def _classify_ground(self, points: np.ndarray) -> np.ndarray:
        """Return boolean mask: True = ground."""
        N = points.shape[0]
        ground = np.zeros(N, dtype=bool)

        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        radius = np.sqrt(x * x + y * y)
        theta = np.degrees(np.arctan2(y, x))
        theta = np.where(theta < 0, theta + 360.0, theta)
        theta = np.where(theta >= 360.0, theta - 360.0, theta)

        sector_idx = np.floor(theta / max(0.1, self.radial_divider_angle)).astype(np.int32)
        sector_idx = np.clip(sector_idx, 0, self._num_sectors - 1)

        for s in range(self._num_sectors):
            mask = sector_idx == s
            if np.count_nonzero(mask) < 2:
                continue
            sector_indices = np.where(mask)[0]
            order = np.argsort(radius[sector_indices])
            sector_indices = sector_indices[order]

            prev_radius = 0.0
            prev_height = 0.0
            prev_ground = False

            for j, idx in enumerate(sector_indices):
                r = float(radius[idx])
                h = float(z[idx])

                if j == 0:
                    # First point in sector – use initial (stricter) slope
                    h_thresh = self._tan_initial * r
                    h_thresh = max(h_thresh, self.min_height_threshold)
                    general_thresh = self._tan_general * r
                    if abs(h) <= h_thresh:
                        ground[idx] = True
                        prev_ground = True
                    elif abs(h) <= general_thresh:
                        ground[idx] = True
                        prev_ground = True
                    else:
                        prev_ground = False
                    prev_radius = r
                    prev_height = h
                    continue

                dr = r - prev_radius
                if dr < self.concentric_divider_distance:
                    ground[idx] = prev_ground
                    continue

                local_thresh = self._tan_local * dr
                local_thresh = max(local_thresh, self.min_height_threshold)
                general_thresh = self._tan_general * r

                dh = abs(h - prev_height)

                if dh <= local_thresh:
                    if prev_ground:
                        ground[idx] = True
                    else:
                        ground[idx] = abs(h) <= general_thresh
                else:
                    if dr > self.reclass_distance_threshold and abs(h) <= general_thresh:
                        ground[idx] = True
                    else:
                        ground[idx] = False

                if ground[idx]:
                    prev_ground = True
                else:
                    prev_ground = False

                prev_radius = r
                prev_height = h

        return ground

    # ------------------------------------------------------------------
    #  Traversability grid
    # ------------------------------------------------------------------

    def _update_traversability(self, ground_points: np.ndarray) -> None:
        if ground_points.shape[0] < 20:
            return

        x, y, z = ground_points[:, 0], ground_points[:, 1], ground_points[:, 2]

        cols = np.floor(
            (x - self.traversability_origin_x) / self.traversability_resolution
        ).astype(np.int32)
        rows = np.floor(
            (y - self.traversability_origin_y) / self.traversability_resolution
        ).astype(np.int32)

        valid = (
            (cols >= 0)
            & (cols < self.traversability_width)
            & (rows >= 0)
            & (rows < self.traversability_height)
        )
        cols, rows, z = cols[valid], rows[valid], z[valid]
        if cols.size < 20:
            return

        H = self.traversability_height
        W = self.traversability_width

        if self._ground_height_map is None:
            self._ground_height_map = np.full((H, W), np.nan, dtype=np.float32)
            self._ground_var_map = np.zeros((H, W), dtype=np.float32)
            self._ground_count_map = np.zeros((H, W), dtype=np.int32)

        # Exponential moving average for height
        alpha = 0.3
        for r, c, zv in zip(rows, cols, z):
            cnt = self._ground_count_map[r, c]
            old = self._ground_height_map[r, c]
            if cnt == 0 or np.isnan(old):
                self._ground_height_map[r, c] = float(zv)
            else:
                self._ground_height_map[r, c] = float(
                    old * (1.0 - alpha) + zv * alpha
                )
            self._ground_count_map[r, c] = cnt + 1

    def _publish_traversability(self) -> None:
        if self._ground_height_map is None:
            return

        H = self.traversability_height
        W = self.traversability_width
        res = self.traversability_resolution

        height = self._ground_height_map
        count = self._ground_count_map

        # Compute local slope magnitude from height differences
        slope = np.full((H, W), np.nan, dtype=np.float32)
        valid = ~np.isnan(height)
        for dr, dc in [(1, 0), (0, 1)]:
            shifted = np.roll(height, shift=(-dr, -dc), axis=(0, 1))
            mask = valid & ~np.isnan(shifted)
            dz = np.abs(height[mask] - shifted[mask])
            s = np.degrees(np.arctan(dz / (res * max(dr, dc))))
            if np.any(mask):
                existing = slope[mask]
                slope[mask] = np.fmax(
                    np.where(np.isnan(existing), 0.0, existing), s
                )

        # Cost: 0=free, 100=occupied, -1=unknown (OccupancyGrid convention)
        cost = np.full((H, W), -1, dtype=np.int8)

        max_slope = self.max_traversable_slope_deg
        too_steep = ~np.isnan(slope) & (slope > max_slope)
        known = count >= 3

        cost[known] = 0  # default traversable
        cost[known & too_steep] = 100  # steep → occupied

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        grid = _build_occupancy_grid(
            cost, res, self.traversability_origin_x,
            self.traversability_origin_y, header, self.frame_id,
        )
        self._traversability_pub.publish(grid)


def main() -> None:
    rclpy.init()
    node = GroundSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
