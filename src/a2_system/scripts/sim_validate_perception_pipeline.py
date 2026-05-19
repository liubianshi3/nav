#!/usr/bin/env python3
"""Synthetic ROS validation for the JT128 perception/traversability chain.

This script runs the real ROS nodes in an isolated domain and feeds deterministic
synthetic data:

* /sim/jt128/front/points contains near-zero JT128 returns, self/body returns,
  flat ground, and a real obstacle outside the STOP polygon.
* /sim/traversability_bridge/input contains a small OccupancyGrid with known
  obstacle, unknown, and out-of-window cells.

It validates the failure mode that caused false "front obstacle" reports:

* ground_segmentation publishes transformed map-frame obstacle points,
* near-zero/self returns do not leak into the STOP polygon,
* traversability bridge publishes base_link-frame local-window points,
* unknown/out-of-window traversability cells do not pollute the output.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Iterable

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from sensor_msgs_py import point_cloud2
from tf2_ros import StaticTransformBroadcaster


MAP_FRAME = "map"
BASE_FRAME = "base_link"
LIDAR_FRAME = "jt128_front_link"


@dataclass
class CloudStats:
    frame_id: str
    total: int
    stop_points: int
    nearest_stop_xy: float | None
    min_x: float | None
    max_x: float | None


def base_to_lidar(points_base: np.ndarray) -> np.ndarray:
    """Convert base_link points to JT128 lidar frame using the A2 extrinsic."""
    rotation_base_from_lidar = np.array(
        [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float32,
    )
    translation = np.array([0.33767, 0.0, 0.08134], dtype=np.float32)
    return (points_base - translation) @ rotation_base_from_lidar


def make_synthetic_jt128_cloud() -> np.ndarray:
    rng = np.random.default_rng(42)

    near_zero = rng.normal(0.0, 0.015, size=(600, 3)).astype(np.float32)

    self_base = np.column_stack(
        (
            rng.uniform(0.05, 0.42, 450),
            rng.uniform(-0.28, 0.28, 450),
            rng.uniform(-0.05, 0.32, 450),
        )
    ).astype(np.float32)
    self_lidar = base_to_lidar(self_base)

    xs = np.linspace(0.55, 4.0, 80, dtype=np.float32)
    ys = np.linspace(-1.5, 1.5, 41, dtype=np.float32)
    grid_x, grid_y = np.meshgrid(xs, ys)
    ground_base = np.column_stack(
        (
            grid_x.ravel(),
            grid_y.ravel(),
            np.full(grid_x.size, -0.28, dtype=np.float32),
        )
    )
    ground_base[:, 2] += rng.normal(0.0, 0.005, size=ground_base.shape[0]).astype(np.float32)
    ground_lidar = base_to_lidar(ground_base)

    obstacle_base = np.column_stack(
        (
            rng.uniform(1.2, 1.5, 120),
            rng.uniform(0.75, 1.05, 120),
            rng.uniform(0.15, 0.55, 120),
        )
    ).astype(np.float32)
    obstacle_lidar = base_to_lidar(obstacle_base)

    return np.vstack((near_zero, self_lidar, ground_lidar, obstacle_lidar)).astype(np.float32)


def make_transform(
    parent: str,
    child: str,
    xyz: tuple[float, float, float],
    xyzw: tuple[float, float, float, float],
) -> TransformStamped:
    msg = TransformStamped()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = xyz[0]
    msg.transform.translation.y = xyz[1]
    msg.transform.translation.z = xyz[2]
    msg.transform.rotation.x = xyzw[0]
    msg.transform.rotation.y = xyzw[1]
    msg.transform.rotation.z = xyzw[2]
    msg.transform.rotation.w = xyzw[3]
    return msg


def make_traversability_grid(node: Node) -> OccupancyGrid:
    width = 120
    height = 100
    resolution = 0.1
    origin_x = -2.0
    origin_y = -5.0
    data = np.zeros((height, width), dtype=np.int8)

    def set_cell(world_x: float, world_y: float, value: int) -> None:
        col = int(math.floor((world_x - origin_x) / resolution))
        row = int(math.floor((world_y - origin_y) / resolution))
        if 0 <= row < height and 0 <= col < width:
            data[row, col] = value

    set_cell(2.0, 0.5, 100)   # inside local window
    set_cell(3.0, -0.5, 100)  # inside local window
    set_cell(8.0, 0.0, 100)   # outside local window, should be dropped
    set_cell(1.0, 1.0, -1)    # unknown, should be ignored

    grid = OccupancyGrid()
    grid.header.stamp = node.get_clock().now().to_msg()
    grid.header.frame_id = MAP_FRAME
    grid.info.resolution = resolution
    grid.info.width = width
    grid.info.height = height
    grid.info.origin.position.x = origin_x
    grid.info.origin.position.y = origin_y
    grid.info.origin.orientation.w = 1.0
    grid.data = data.ravel().tolist()
    return grid


def iter_xyz(msg: PointCloud2) -> Iterable[tuple[float, float, float]]:
    for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        yield (float(point[0]), float(point[1]), float(point[2]))


def compute_cloud_stats(msg: PointCloud2) -> CloudStats:
    points = np.array(list(iter_xyz(msg)), dtype=np.float32)
    if points.size == 0:
        return CloudStats(msg.header.frame_id, 0, 0, None, None, None)

    in_stop = (
        (points[:, 0] >= -0.3)
        & (points[:, 0] <= 0.5)
        & (points[:, 1] >= -0.4)
        & (points[:, 1] <= 0.4)
        & (points[:, 2] >= 0.05)
        & (points[:, 2] <= 0.85)
    )
    stop_points = points[in_stop]
    nearest = None
    if len(stop_points):
        nearest = float(np.min(np.hypot(stop_points[:, 0], stop_points[:, 1])))

    return CloudStats(
        frame_id=msg.header.frame_id,
        total=int(len(points)),
        stop_points=int(len(stop_points)),
        nearest_stop_xy=nearest,
        min_x=float(np.min(points[:, 0])),
        max_x=float(np.max(points[:, 0])),
    )


def parse_status_fields(text: str) -> dict[str, str]:
    out: dict[str, str] = {}
    for part in text.split(";"):
        if "=" in part:
            key, value = part.split("=", 1)
            out[key.strip()] = value.strip()
    return out


class SimHarness(Node):
    def __init__(self) -> None:
        super().__init__("a2_perception_sim_harness")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.static_tf = StaticTransformBroadcaster(self)
        self.cloud_pub = self.create_publisher(PointCloud2, "/sim/jt128/front/points", sensor_qos)
        self.grid_pub = self.create_publisher(
            OccupancyGrid, "/sim/traversability_bridge/input", latched_qos
        )

        self.obstacle: PointCloud2 | None = None
        self.ground: PointCloud2 | None = None
        self.traversability: OccupancyGrid | None = None
        self.status_text: str | None = None
        self.bridge_cloud: PointCloud2 | None = None

        self.create_subscription(PointCloud2, "/sim/a2/obstacle/points", self._on_obstacle, 10)
        self.create_subscription(PointCloud2, "/sim/a2/ground/points", self._on_ground, 10)
        self.create_subscription(OccupancyGrid, "/sim/a2/traversability", self._on_trav, 10)
        self.create_subscription(
            PointCloud2,
            "/sim/traversability_bridge/obstacle_points",
            self._on_bridge_cloud,
            10,
        )
        self.create_subscription(
            String,
            "/sim/a2/perception/ground_segmentation/status",
            self._on_status,
            10,
        )

        self.synthetic_cloud = make_synthetic_jt128_cloud()
        self.grid = make_traversability_grid(self)
        self._publish_static_tf()

        self.create_timer(0.2, self._publish_inputs)

    def _publish_static_tf(self) -> None:
        transforms = [
            make_transform(MAP_FRAME, BASE_FRAME, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            make_transform(
                BASE_FRAME,
                LIDAR_FRAME,
                (0.33767, 0.0, 0.08134),
                (0.5, 0.5, 0.5, 0.5),
            ),
        ]
        now = self.get_clock().now().to_msg()
        for msg in transforms:
            msg.header.stamp = now
        self.static_tf.sendTransform(transforms)

    def _publish_inputs(self) -> None:
        self._publish_static_tf()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = LIDAR_FRAME
        cloud = point_cloud2.create_cloud_xyz32(header, self.synthetic_cloud.tolist())
        self.cloud_pub.publish(cloud)

        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.grid_pub.publish(self.grid)

    def _on_obstacle(self, msg: PointCloud2) -> None:
        self.obstacle = msg

    def _on_ground(self, msg: PointCloud2) -> None:
        self.ground = msg

    def _on_trav(self, msg: OccupancyGrid) -> None:
        self.traversability = msg

    def _on_status(self, msg) -> None:
        self.status_text = msg.data

    def _on_bridge_cloud(self, msg: PointCloud2) -> None:
        self.bridge_cloud = msg

    def ready(self) -> bool:
        return all(
            item is not None
            for item in (
                self.obstacle,
                self.ground,
                self.traversability,
                self.status_text,
                self.bridge_cloud,
            )
        )


def start_processes() -> list[subprocess.Popen]:
    common_env = os.environ.copy()
    processes = [
        subprocess.Popen(
            [
                "ros2",
                "run",
                "a2_ground_segmentation_cpp",
                "ground_segmentation_cpp_node",
                "--ros-args",
                "-r",
                "__node:=sim_ground_segmentation",
                "-p",
                "input_topic:=/sim/jt128/front/points",
                "-p",
                "ground_topic:=/sim/a2/ground/points",
                "-p",
                "obstacle_topic:=/sim/a2/obstacle/points",
                "-p",
                "traversability_topic:=/sim/a2/traversability",
                "-p",
                "status_topic:=/sim/a2/perception/ground_segmentation/status",
                "-p",
                "target_frame:=map",
                "-p",
                "input_min_range_m:=0.15",
                "-p",
                "self_filter_enabled:=true",
                "-p",
                "self_filter_frame:=base_link",
                "-p",
                "traversability_width:=2000",
                "-p",
                "traversability_height:=2000",
                "-p",
                "traversability_origin_x:=-100.0",
                "-p",
                "traversability_origin_y:=-100.0",
            ],
            env=common_env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        ),
        subprocess.Popen(
            [
                "ros2",
                "run",
                "a2_system",
                "traversability_to_obstacle_cloud.py",
                "--ros-args",
                "-r",
                "__node:=sim_traversability_to_obstacle_cloud",
                "-p",
                "traversability_topic:=/sim/traversability_bridge/input",
                "-p",
                "output_topic:=/sim/traversability_bridge/obstacle_points",
                "-p",
                "output_frame:=base_link",
                "-p",
                "treat_unknown_as_obstacle:=false",
                "-p",
                "local_window_enabled:=true",
                "-p",
                "local_min_x:=-1.0",
                "-p",
                "local_max_x:=6.0",
                "-p",
                "local_min_y:=-4.0",
                "-p",
                "local_max_y:=4.0",
                "-p",
                "max_output_points:=20000",
            ],
            env=common_env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        ),
    ]
    return processes


def stop_processes(processes: list[subprocess.Popen]) -> None:
    for proc in processes:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
    deadline = time.monotonic() + 4.0
    for proc in processes:
        while proc.poll() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if proc.poll() is None:
            proc.terminate()
    time.sleep(0.2)
    for proc in processes:
        if proc.poll() is None:
            proc.kill()


def collect_process_output(processes: list[subprocess.Popen]) -> str:
    chunks: list[str] = []
    for proc in processes:
        if proc.stdout is not None:
            try:
                chunks.append(proc.stdout.read() or "")
            except Exception:
                pass
    return "\n".join(chunks)


def validate(harness: SimHarness, max_stop_points: int) -> list[str]:
    failures: list[str] = []
    assert harness.obstacle is not None
    assert harness.ground is not None
    assert harness.traversability is not None
    assert harness.status_text is not None
    assert harness.bridge_cloud is not None

    obstacle_stats = compute_cloud_stats(harness.obstacle)
    ground_stats = compute_cloud_stats(harness.ground)
    bridge_stats = compute_cloud_stats(harness.bridge_cloud)
    status = parse_status_fields(harness.status_text)

    if obstacle_stats.frame_id != MAP_FRAME:
        failures.append(f"obstacle frame is {obstacle_stats.frame_id!r}, expected {MAP_FRAME!r}")
    if ground_stats.frame_id != MAP_FRAME:
        failures.append(f"ground frame is {ground_stats.frame_id!r}, expected {MAP_FRAME!r}")
    if obstacle_stats.stop_points > max_stop_points:
        failures.append(
            f"STOP polygon still contains {obstacle_stats.stop_points} obstacle points "
            f"(max {max_stop_points})"
        )
    if int(status.get("dropped_min_range", "0")) <= 0:
        failures.append("ground_segmentation did not report dropped_min_range > 0")
    if int(status.get("dropped_self_filter", "0")) <= 0:
        failures.append("ground_segmentation did not report dropped_self_filter > 0")
    if harness.traversability.header.frame_id != MAP_FRAME:
        failures.append(
            f"traversability frame is {harness.traversability.header.frame_id!r}, expected {MAP_FRAME!r}"
        )
    if harness.traversability.info.width != 2000 or harness.traversability.info.height != 2000:
        failures.append(
            "traversability grid params not applied: "
            f"{harness.traversability.info.width}x{harness.traversability.info.height}"
        )
    if bridge_stats.frame_id != BASE_FRAME:
        failures.append(f"bridge frame is {bridge_stats.frame_id!r}, expected {BASE_FRAME!r}")
    if bridge_stats.total != 2:
        failures.append(
            f"bridge emitted {bridge_stats.total} points, expected 2 local known obstacles"
        )
    if bridge_stats.min_x is not None and bridge_stats.max_x is not None:
        if bridge_stats.min_x < -1.05 or bridge_stats.max_x > 6.05:
            failures.append(
                f"bridge local window failed: x range [{bridge_stats.min_x}, {bridge_stats.max_x}]"
            )

    print("=== Synthetic Perception Validation ===")
    print(f"ground_frame={ground_stats.frame_id} ground_points={ground_stats.total}")
    print(
        "obstacle_frame="
        f"{obstacle_stats.frame_id} obstacle_points={obstacle_stats.total} "
        f"stop_points={obstacle_stats.stop_points}"
    )
    print(
        "status="
        f"ready={status.get('ready')} reason={status.get('reason')} "
        f"dropped_min_range={status.get('dropped_min_range')} "
        f"dropped_self_filter={status.get('dropped_self_filter')}"
    )
    print(
        "traversability="
        f"{harness.traversability.header.frame_id} "
        f"{harness.traversability.info.width}x{harness.traversability.info.height}"
    )
    print(
        "bridge="
        f"frame={bridge_stats.frame_id} points={bridge_stats.total} "
        f"x_range=[{bridge_stats.min_x}, {bridge_stats.max_x}]"
    )

    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-sec", type=float, default=12.0)
    parser.add_argument("--max-stop-points", type=int, default=3)
    args = parser.parse_args()

    processes = start_processes()
    rclpy.init()
    harness = SimHarness()
    try:
        deadline = time.monotonic() + args.timeout_sec
        while time.monotonic() < deadline and not harness.ready():
            rclpy.spin_once(harness, timeout_sec=0.1)
            for proc in processes:
                if proc.poll() not in (None, 0):
                    output = collect_process_output(processes)
                    print(output, file=sys.stderr)
                    print(f"subprocess exited early with code {proc.returncode}", file=sys.stderr)
                    return 2

        for _ in range(20):
            rclpy.spin_once(harness, timeout_sec=0.05)

        if not harness.ready():
            missing = [
                name
                for name, value in (
                    ("obstacle", harness.obstacle),
                    ("ground", harness.ground),
                    ("traversability", harness.traversability),
                    ("status", harness.status_text),
                    ("bridge_cloud", harness.bridge_cloud),
                )
                if value is None
            ]
            print(f"timed out waiting for: {', '.join(missing)}", file=sys.stderr)
            print(collect_process_output(processes), file=sys.stderr)
            return 3

        failures = validate(harness, args.max_stop_points)
        if failures:
            print("FAIL")
            for failure in failures:
                print(f"- {failure}")
            return 1

        print("PASS")
        return 0
    finally:
        harness.destroy_node()
        rclpy.shutdown()
        stop_processes(processes)


if __name__ == "__main__":
    raise SystemExit(main())
