#!/usr/bin/env python3
"""One-shot Nav2 corridor gate for real-motion closed-loop tests.

The gate checks whether a short forward goal is safe enough to attempt before
switching from dry-run to live motion. It intentionally reports concrete map
and costmap numbers so a blocked run tells us where it failed.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass
class GridProbe:
    x: float
    y: float
    grid_x: int | None
    grid_y: int | None
    value: int | None
    state: str
    nearest_occupied_m: float | None = None


@dataclass
class GateReport:
    pass_gate: bool
    reason: str
    pose_topic: str
    map_topic: str
    costmap_topic: str
    start_x: float
    start_y: float
    yaw_rad: float
    distance_m: float
    robot_radius_m: float
    min_static_clearance_m: float | None
    max_global_cost: int | None
    static_unknown_samples: int
    costmap_unknown_samples: int
    static_blocked_samples: int
    costmap_blocked_samples: int
    best_direction_deg: float | None
    best_direction_clearance_m: float | None
    samples: list[GridProbe]


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def grid_origin_yaw(grid: OccupancyGrid) -> float:
    return yaw_from_quaternion(grid.info.origin.orientation)


def world_to_grid(grid: OccupancyGrid, x: float, y: float) -> tuple[int | None, int | None]:
    origin = grid.info.origin.position
    resolution = grid.info.resolution
    if resolution <= 0.0:
        return None, None
    dx = x - origin.x
    dy = y - origin.y
    yaw = grid_origin_yaw(grid)
    cos_yaw = math.cos(-yaw)
    sin_yaw = math.sin(-yaw)
    local_x = cos_yaw * dx - sin_yaw * dy
    local_y = sin_yaw * dx + cos_yaw * dy
    gx = int(math.floor(local_x / resolution))
    gy = int(math.floor(local_y / resolution))
    if gx < 0 or gy < 0 or gx >= grid.info.width or gy >= grid.info.height:
        return None, None
    return gx, gy


def grid_to_world(grid: OccupancyGrid, gx: int, gy: int) -> tuple[float, float]:
    origin = grid.info.origin.position
    resolution = grid.info.resolution
    yaw = grid_origin_yaw(grid)
    local_x = (gx + 0.5) * resolution
    local_y = (gy + 0.5) * resolution
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        origin.x + cos_yaw * local_x - sin_yaw * local_y,
        origin.y + sin_yaw * local_x + cos_yaw * local_y,
    )


def grid_value(grid: OccupancyGrid, gx: int | None, gy: int | None) -> int | None:
    if gx is None or gy is None:
        return None
    idx = gy * grid.info.width + gx
    if idx < 0 or idx >= len(grid.data):
        return None
    return int(grid.data[idx])


def classify_occupancy(value: int | None, occupied_threshold: int, allow_unknown: bool) -> str:
    if value is None:
        return "out_of_bounds"
    if value < 0:
        return "free" if allow_unknown else "unknown"
    if value >= occupied_threshold:
        return "occupied"
    return "free"


def corridor_points(x: float, y: float, yaw: float, distance: float, sample_count: int) -> Iterable[tuple[float, float]]:
    count = max(2, sample_count)
    for i in range(count):
        step = distance * i / (count - 1)
        yield x + math.cos(yaw) * step, y + math.sin(yaw) * step


def occupied_centers_near_corridor(
    grid: OccupancyGrid,
    points: list[tuple[float, float]],
    threshold: int,
    search_radius: float,
) -> list[tuple[float, float]]:
    min_x = min(p[0] for p in points) - search_radius
    max_x = max(p[0] for p in points) + search_radius
    min_y = min(p[1] for p in points) - search_radius
    max_y = max(p[1] for p in points) + search_radius
    centers: list[tuple[float, float]] = []
    for gy in range(grid.info.height):
        row = gy * grid.info.width
        for gx in range(grid.info.width):
            value = int(grid.data[row + gx])
            if value < threshold:
                continue
            wx, wy = grid_to_world(grid, gx, gy)
            if min_x <= wx <= max_x and min_y <= wy <= max_y:
                centers.append((wx, wy))
    return centers


def nearest_occupied_distance(point: tuple[float, float], centers: list[tuple[float, float]]) -> float | None:
    if not centers:
        return None
    px, py = point
    return min(math.hypot(px - ox, py - oy) for ox, oy in centers)


class CorridorGateNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("nav2_corridor_gate")
        self.args = args
        self.pose: PoseWithCovarianceStamped | None = None
        self.static_map: OccupancyGrid | None = None
        self.costmap: OccupancyGrid | None = None

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(PoseWithCovarianceStamped, args.pose_topic, self._on_pose, 10)
        self.create_subscription(OccupancyGrid, args.map_topic, self._on_map, transient_qos)
        self.create_subscription(OccupancyGrid, args.costmap_topic, self._on_costmap, 10)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.pose = msg

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.static_map = msg

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self.costmap = msg

    def wait_for_inputs(self) -> list[str]:
        deadline = time.monotonic() + self.args.timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None and self.static_map is not None and self.costmap is not None:
                return []
        missing = []
        if self.pose is None:
            missing.append(self.args.pose_topic)
        if self.static_map is None:
            missing.append(self.args.map_topic)
        if self.costmap is None:
            missing.append(self.args.costmap_topic)
        return missing


def build_report(args: argparse.Namespace, pose_msg: PoseWithCovarianceStamped, static_map: OccupancyGrid, costmap: OccupancyGrid) -> GateReport:
    pose = pose_msg.pose.pose
    start_x = float(pose.position.x)
    start_y = float(pose.position.y)
    yaw = yaw_from_quaternion(pose.orientation)
    points = list(corridor_points(start_x, start_y, yaw, args.distance, args.sample_count))
    search_radius = max(args.search_radius, args.robot_radius + static_map.info.resolution * 2.0)
    occupied_centers = occupied_centers_near_corridor(static_map, points, args.static_occupied_threshold, search_radius)

    samples: list[GridProbe] = []
    min_clearance: float | None = None
    max_cost: int | None = None
    static_unknown = 0
    costmap_unknown = 0
    static_blocked = 0
    costmap_blocked = 0

    for x, y in points:
        static_gx, static_gy = world_to_grid(static_map, x, y)
        static_value = grid_value(static_map, static_gx, static_gy)
        static_state = classify_occupancy(static_value, args.static_occupied_threshold, args.allow_static_unknown)
        clearance = nearest_occupied_distance((x, y), occupied_centers)
        if clearance is not None:
            min_clearance = clearance if min_clearance is None else min(min_clearance, clearance)
        if static_state == "unknown":
            static_unknown += 1
        if static_state in {"occupied", "out_of_bounds"}:
            static_blocked += 1

        cost_gx, cost_gy = world_to_grid(costmap, x, y)
        cost_value = grid_value(costmap, cost_gx, cost_gy)
        cost_state = classify_occupancy(cost_value, args.max_global_cost + 1, args.allow_costmap_unknown)
        if cost_value is not None and cost_value >= 0:
            max_cost = cost_value if max_cost is None else max(max_cost, cost_value)
        if cost_state == "unknown":
            costmap_unknown += 1
        if cost_state in {"occupied", "out_of_bounds"}:
            costmap_blocked += 1

        state = "free"
        if static_state != "free":
            state = f"static_{static_state}"
        elif clearance is not None and clearance < args.min_static_clearance:
            state = "static_clearance_low"
        elif cost_state != "free":
            state = f"costmap_{cost_state}"
        elif cost_value is not None and cost_value > args.max_global_cost:
            state = "costmap_cost_high"

        samples.append(
            GridProbe(
                x=x,
                y=y,
                grid_x=static_gx,
                grid_y=static_gy,
                value=cost_value,
                state=state,
                nearest_occupied_m=clearance,
            )
        )

    best_direction_deg: float | None = None
    best_direction_clearance: float | None = None
    if args.scan_directions:
        for idx in range(args.scan_direction_count):
            delta = -math.pi + 2.0 * math.pi * idx / args.scan_direction_count
            candidate_yaw = yaw + delta
            candidate_points = list(corridor_points(start_x, start_y, candidate_yaw, args.distance, args.sample_count))
            centers = occupied_centers_near_corridor(static_map, candidate_points, args.static_occupied_threshold, search_radius)
            clearances = [nearest_occupied_distance(point, centers) for point in candidate_points]
            numeric = [value for value in clearances if value is not None]
            candidate_clearance = min(numeric) if numeric else search_radius
            if best_direction_clearance is None or candidate_clearance > best_direction_clearance:
                best_direction_clearance = candidate_clearance
                best_direction_deg = math.degrees(delta)

    failed_states = [sample.state for sample in samples if sample.state != "free"]
    pass_gate = not failed_states
    reason = "pass"
    if not pass_gate:
        reason = failed_states[0]

    return GateReport(
        pass_gate=pass_gate,
        reason=reason,
        pose_topic=args.pose_topic,
        map_topic=args.map_topic,
        costmap_topic=args.costmap_topic,
        start_x=start_x,
        start_y=start_y,
        yaw_rad=yaw,
        distance_m=args.distance,
        robot_radius_m=args.robot_radius,
        min_static_clearance_m=min_clearance,
        max_global_cost=max_cost,
        static_unknown_samples=static_unknown,
        costmap_unknown_samples=costmap_unknown,
        static_blocked_samples=static_blocked,
        costmap_blocked_samples=costmap_blocked,
        best_direction_deg=best_direction_deg,
        best_direction_clearance_m=best_direction_clearance,
        samples=samples,
    )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose-topic", default="/a2/relocalization/pose")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--costmap-topic", default="/global_costmap/costmap")
    parser.add_argument("--distance", type=float, default=0.5)
    parser.add_argument("--robot-radius", type=float, default=0.35)
    parser.add_argument("--min-static-clearance", type=float, default=0.35)
    parser.add_argument("--max-global-cost", type=int, default=98)
    parser.add_argument("--static-occupied-threshold", type=int, default=65)
    parser.add_argument("--sample-count", type=int, default=11)
    parser.add_argument("--search-radius", type=float, default=2.0)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument("--allow-static-unknown", action="store_true")
    parser.add_argument("--allow-costmap-unknown", action="store_true")
    parser.add_argument("--scan-directions", action="store_true")
    parser.add_argument("--scan-direction-count", type=int, default=24)
    parser.add_argument("--output-json", default="")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    node = CorridorGateNode(args)
    try:
        missing = node.wait_for_inputs()
        if missing:
            report = {
                "pass_gate": False,
                "reason": "missing_inputs",
                "missing_topics": missing,
                "pose_topic": args.pose_topic,
                "map_topic": args.map_topic,
                "costmap_topic": args.costmap_topic,
            }
            if args.output_json:
                Path(args.output_json).parent.mkdir(parents=True, exist_ok=True)
                Path(args.output_json).write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            print(json.dumps(report, indent=2, sort_keys=True))
            return 2

        report = build_report(args, node.pose, node.static_map, node.costmap)
        payload = asdict(report)
        if args.output_json:
            Path(args.output_json).parent.mkdir(parents=True, exist_ok=True)
            Path(args.output_json).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(payload, indent=2, sort_keys=True))
        return 0 if report.pass_gate else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
