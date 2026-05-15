#!/usr/bin/env python3

from __future__ import annotations

import argparse
import heapq
import json
import math
import os
import signal
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from a2_interfaces.srv import ManageMap
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def point_count(msg: PointCloud2 | None) -> int:
    if msg is None:
        return 0
    return int(msg.width) * int(msg.height)


def read_cloud_xyz(msg: PointCloud2, max_points: int = 20000) -> list[tuple[float, float, float]]:
    fields = {field.name: field for field in msg.fields}
    if not {"x", "y", "z"}.issubset(fields):
        return []
    total = int(msg.width) * int(msg.height)
    if total <= 0:
        return []
    stride = max(1, math.ceil(total / max_points))
    endian = ">" if msg.is_bigendian else "<"
    unpack_float = struct.Struct(f"{endian}f").unpack_from
    raw = memoryview(msg.data)
    points: list[tuple[float, float, float]] = []
    for index in range(0, total, stride):
        base = index * msg.point_step
        x = unpack_float(raw, base + fields["x"].offset)[0]
        y = unpack_float(raw, base + fields["y"].offset)[0]
        z = unpack_float(raw, base + fields["z"].offset)[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append((float(x), float(y), float(z)))
    return points


class ClosedLoopProbe(Node):
    def __init__(self, map_root: Path) -> None:
        super().__init__("industrial_3d_closed_loop_probe")
        self.map_root = map_root
        self.latest_odom: Odometry | None = None
        self.latest_live_cloud: PointCloud2 | None = None
        self.latest_accumulated_map: PointCloud2 | None = None
        self.latest_loaded_map: PointCloud2 | None = None
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self.create_subscription(Odometry, "/jt128/dlio/odom", self._on_odom, 20)
        self.create_subscription(PointCloud2, "/jt128/front/points", self._on_live_cloud, 10)
        self.create_subscription(PointCloud2, "/a2/pointcloud_map_3d", self._on_accumulated_map, 10)
        self.create_subscription(PointCloud2, "/a2/map/pointcloud_3d", self._on_loaded_map, 10)
        self.map_client = self.create_client(ManageMap, "/map_manager/manage_map")

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _on_live_cloud(self, msg: PointCloud2) -> None:
        self.latest_live_cloud = msg

    def _on_accumulated_map(self, msg: PointCloud2) -> None:
        self.latest_accumulated_map = msg

    def _on_loaded_map(self, msg: PointCloud2) -> None:
        self.latest_loaded_map = msg

    def pose(self) -> Pose2D | None:
        if self.latest_odom is None:
            return None
        p = self.latest_odom.pose.pose.position
        return Pose2D(float(p.x), float(p.y), yaw_from_odom(self.latest_odom))

    def spin_until(self, predicate, timeout_sec: float, label: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {label}")

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())
        rclpy.spin_once(self, timeout_sec=0.05)

    def drive(self, linear_x: float, linear_y: float, angular_z: float, duration_sec: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        end_time = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < end_time:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stop()

    def save_map(self, map_id: str, timeout_sec: float = 8.0) -> None:
        self.spin_until(lambda: self.map_client.wait_for_service(timeout_sec=0.1), timeout_sec, "map_manager service")
        request = ManageMap.Request()
        request.command = "save"
        request.map_id = map_id
        future = self.map_client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                result = future.result()
                if result is None or not result.success:
                    raise RuntimeError(f"map save failed: {getattr(result, 'message', 'no response')}")
                return
        raise RuntimeError("timeout saving map through map_manager")


def run_process(command: list[str], log_path: Path, env: dict[str, str]) -> subprocess.Popen:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("w", encoding="utf-8")
    return subprocess.Popen(
        command,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        cwd=Path.cwd(),
        env=env,
        preexec_fn=os.setsid,
    )


def stop_processes(processes: list[subprocess.Popen]) -> None:
    for process in reversed(processes):
        if process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
    time.sleep(1.0)
    for process in reversed(processes):
        if process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass


def write_ascii_pcd(path: Path, points: list[tuple[float, float, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as handle:
        handle.write("# .PCD v0.7 - Point Cloud Data file format\n")
        handle.write("VERSION 0.7\n")
        handle.write("FIELDS x y z\n")
        handle.write("SIZE 4 4 4\n")
        handle.write("TYPE F F F\n")
        handle.write("COUNT 1 1 1\n")
        handle.write(f"WIDTH {len(points)}\n")
        handle.write("HEIGHT 1\n")
        handle.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        handle.write(f"POINTS {len(points)}\n")
        handle.write("DATA ascii\n")
        for x, y, z in points:
            handle.write(f"{x:.4f} {y:.4f} {z:.4f}\n")


def generate_industrial_seed_map(path: Path) -> int:
    points: list[tuple[float, float, float]] = []
    # Ground grid.
    for ix in range(-60, 61):
        for iy in range(-40, 41):
            points.append((ix * 0.05, iy * 0.05, 0.0))
    # Perimeter walls.
    for x in [i * 0.05 for i in range(-60, 61)]:
        for z in [0.15 + k * 0.10 for k in range(16)]:
            points.append((x, -2.0, z))
            points.append((x, 2.0, z))
    for y in [i * 0.05 for i in range(-40, 41)]:
        for z in [0.15 + k * 0.10 for k in range(16)]:
            points.append((-3.0, y, z))
            points.append((3.0, y, z))
    # A small box obstacle offset from the planned corridor.
    for x in [0.85 + i * 0.05 for i in range(9)]:
        for y in [0.45 + i * 0.05 for i in range(9)]:
            for z in [0.10 + i * 0.08 for i in range(10)]:
                if x in (0.85, 1.25) or y in (0.45, 0.85):
                    points.append((x, y, z))
    write_ascii_pcd(path, points)
    return len(points)


def run_pcd_projection(pcd_path: Path, output_dir: Path, env: dict[str, str], log_path: Path) -> None:
    cmd = [
        "ros2",
        "run",
        "a2_system",
        "pcd_to_2d_map.py",
        str(pcd_path),
        "--output",
        str(output_dir),
        "--resolution",
        "0.05",
        "--ground-threshold",
        "0.08",
        "--ceiling-threshold",
        "1.8",
        "--min-obstacle-points",
        "1",
        "--min-ground-points",
        "1",
        "--dilate",
        "1",
    ]
    with log_path.open("w", encoding="utf-8") as log:
        result = subprocess.run(cmd, cwd=Path.cwd(), env=env, stdout=log, stderr=subprocess.STDOUT, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"pcd_to_2d_map failed; see {log_path}")


def read_pgm(path: Path) -> tuple[int, int, list[int]]:
    with path.open("rb") as handle:
        magic = handle.readline().strip()
        if magic != b"P5":
            raise RuntimeError(f"unsupported PGM magic: {magic!r}")
        line = handle.readline().strip()
        while line.startswith(b"#"):
            line = handle.readline().strip()
        width, height = [int(v) for v in line.split()]
        max_value = int(handle.readline().strip())
        if max_value != 255:
            raise RuntimeError("only 8-bit PGM maps are supported")
        data = list(handle.read())
    if len(data) != width * height:
        raise RuntimeError("PGM data size does not match header")
    return width, height, data


def world_to_grid(x: float, y: float, origin: list[float], resolution: float, height: int) -> tuple[int, int]:
    col = int((x - float(origin[0])) / resolution)
    row_from_bottom = int((y - float(origin[1])) / resolution)
    return col, height - 1 - row_from_bottom


def grid_to_world(col: int, row: int, origin: list[float], resolution: float, height: int) -> tuple[float, float]:
    x = float(origin[0]) + (col + 0.5) * resolution
    y = float(origin[1]) + (height - row - 0.5) * resolution
    return x, y


def plan_grid_path(map_yaml: Path, start: Pose2D, goal_xy: tuple[float, float]) -> list[tuple[float, float]]:
    meta = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    pgm_path = map_yaml.parent / meta["image"]
    width, height, data = read_pgm(pgm_path)
    resolution = float(meta["resolution"])
    origin = list(meta["origin"])
    start_cell = world_to_grid(start.x, start.y, origin, resolution, height)
    goal_cell = world_to_grid(goal_xy[0], goal_xy[1], origin, resolution, height)

    def passable(cell: tuple[int, int]) -> bool:
        col, row = cell
        if not (0 <= col < width and 0 <= row < height):
            return False
        return data[row * width + col] > 80

    def nearest_passable(cell: tuple[int, int]) -> tuple[int, int]:
        if passable(cell):
            return cell
        col0, row0 = cell
        for radius in range(1, 30):
            for dc in range(-radius, radius + 1):
                for dr in range(-radius, radius + 1):
                    candidate = (col0 + dc, row0 + dr)
                    if passable(candidate):
                        return candidate
        raise RuntimeError(f"no passable cell near {cell}")

    start_cell = nearest_passable(start_cell)
    goal_cell = nearest_passable(goal_cell)
    frontier: list[tuple[float, tuple[int, int]]] = [(0.0, start_cell)]
    came_from: dict[tuple[int, int], tuple[int, int] | None] = {start_cell: None}
    cost_so_far: dict[tuple[int, int], float] = {start_cell: 0.0}
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal_cell:
            break
        for dc, dr in neighbors:
            nxt = (current[0] + dc, current[1] + dr)
            if not passable(nxt):
                continue
            step_cost = math.sqrt(2.0) if dc and dr else 1.0
            new_cost = cost_so_far[current] + step_cost
            if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                cost_so_far[nxt] = new_cost
                heuristic = math.hypot(goal_cell[0] - nxt[0], goal_cell[1] - nxt[1])
                heapq.heappush(frontier, (new_cost + heuristic, nxt))
                came_from[nxt] = current

    if goal_cell not in came_from:
        raise RuntimeError("planner failed to connect start and goal on loaded map")

    cells = []
    current: tuple[int, int] | None = goal_cell
    while current is not None:
        cells.append(current)
        current = came_from[current]
    cells.reverse()
    return [grid_to_world(col, row, origin, resolution, height) for col, row in cells]


def map_usage_score(live_cloud: PointCloud2, loaded_map: PointCloud2) -> dict:
    live = read_cloud_xyz(live_cloud, max_points=2000)
    loaded = read_cloud_xyz(loaded_map, max_points=8000)
    if not live or not loaded:
        return {"matched": 0, "mean_xy_error_m": float("inf")}
    occupied = {(round(x / 0.10), round(y / 0.10)) for x, y, _ in loaded}
    errors = []
    for x, y, _ in live[:1000]:
        gx = round(x / 0.10)
        gy = round(y / 0.10)
        best = 999.0
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if (gx + dx, gy + dy) in occupied:
                    best = min(best, math.hypot(dx * 0.10, dy * 0.10))
        if best < 999.0:
            errors.append(best)
    return {
        "matched": len(errors),
        "mean_xy_error_m": float(sum(errors) / len(errors)) if errors else float("inf"),
    }


def execute_two_map_steps(node: ClosedLoopProbe, path: list[tuple[float, float]]) -> list[dict]:
    if len(path) < 4:
        raise RuntimeError("planned path is too short to execute two steps")
    samples = [path[min(len(path) - 1, max(3, len(path) // 3))], path[min(len(path) - 1, max(6, 2 * len(path) // 3))]]
    records: list[dict] = []
    for index, target in enumerate(samples, start=1):
        before = node.pose()
        if before is None:
            raise RuntimeError("no odom before executing step")
        desired_yaw = math.atan2(target[1] - before.y, target[0] - before.x)
        yaw_error = math.atan2(math.sin(desired_yaw - before.yaw), math.cos(desired_yaw - before.yaw))
        node.drive(0.0, 0.0, max(-0.35, min(0.35, yaw_error)), min(1.5, abs(yaw_error) / 0.25 + 0.2))
        node.drive(0.16, 0.0, 0.0, 1.2)
        after = node.pose()
        if after is None:
            raise RuntimeError("no odom after executing step")
        moved = math.hypot(after.x - before.x, after.y - before.y)
        records.append(
            {
                "step": index,
                "target_xy": [target[0], target[1]],
                "before": {"x": before.x, "y": before.y, "yaw": before.yaw},
                "after": {"x": after.x, "y": after.y, "yaw": after.yaw},
                "moved_m": moved,
            }
        )
    if any(record["moved_m"] < 0.05 for record in records):
        raise RuntimeError(f"movement verification failed: {records}")
    return records


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run an end-to-end industrial 3D navigation closed-loop check.")
    parser.add_argument("--run-root", default="runtime/closed_loop_runs")
    parser.add_argument("--map-root", default="runtime/maps")
    parser.add_argument("--ros-domain-id", default="")
    parser.add_argument("--map-id", default="")
    parser.add_argument("--keep-processes", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.run_root).expanduser().resolve() / f"industrial_3d_{timestamp}"
    map_root = Path(args.map_root).expanduser().resolve()
    map_id = args.map_id or f"industrial_3d_{timestamp}"
    logs_dir = run_dir / "logs"
    run_dir.mkdir(parents=True, exist_ok=True)
    logs_dir.mkdir(parents=True, exist_ok=True)

    env = os.environ.copy()
    if args.ros_domain_id:
        env["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    else:
        env["ROS_DOMAIN_ID"] = str(140 + int(time.time()) % 40)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    env["A2_WORKSPACE"] = str(Path.cwd())

    seed_pcd = run_dir / "seed_world_3d.pcd"
    seed_points = generate_industrial_seed_map(seed_pcd)
    processes: list[subprocess.Popen] = []

    report: dict = {
        "run_dir": str(run_dir),
        "ros_domain_id": env["ROS_DOMAIN_ID"],
        "seed_pcd": str(seed_pcd),
        "seed_points": seed_points,
        "map_id": map_id,
        "stages": [],
    }

    def stage(name: str, **data) -> None:
        record = {"name": name, "time": datetime.now().isoformat(), **data}
        report["stages"].append(record)
        print(json.dumps(record, ensure_ascii=False), flush=True)

    try:
        stage("stage1_chain", modules=[
            "kinematics_sim:/jt128/front/points,/jt128/dlio/odom",
            "map_manager.pointcloud_accumulator:/a2/pointcloud_map_3d",
            "map_manager.map_manager_node:/map_manager/manage_map",
            "map_manager.pointcloud_map_loader:/a2/map/pointcloud_3d",
            "pcd_to_2d_map:map.yaml/map.pgm",
            "grid planner + cmd_vel_safe controller",
        ], known_break="real_robot_dlio_heap_crash_so_simulator_provides_same_contract")

        processes.append(run_process([
            "ros2", "run", "kinematics_sim", "simulator_node",
            "--ros-args",
            "-p", f"pcd_map_path:={seed_pcd}",
            "-p", "lidar_points_per_scan:=6000",
            "-p", "lidar_rate_hz:=10.0",
            "-p", "odom_rate_hz:=80.0",
            "-p", "lidar_noise_stddev_m:=0.0",
        ], logs_dir / "simulator.log", env))
        processes.append(run_process([
            "ros2", "run", "map_manager", "pointcloud_accumulator",
            "--ros-args",
            "-p", "odom_topic:=/jt128/dlio/odom",
            "-p", "pointcloud_topic:=/jt128/front/points",
            "-p", "output_topic:=/a2/pointcloud_map_3d",
            "-p", "output_frame:=map",
            "-p", "min_translation_delta_m:=0.02",
            "-p", "min_yaw_delta_rad:=0.02",
            "-p", "max_points_per_scan:=12000",
            "-p", "max_range_m:=8.0",
            "-p", "voxel_size:=0.05",
        ], logs_dir / "pointcloud_accumulator.log", env))
        processes.append(run_process([
            "ros2", "run", "map_manager", "map_manager_node",
            "--ros-args",
            "-p", f"map_root:={map_root}",
            "-p", "map_representation:=pointcloud_map_3d",
            "-p", "pointcloud_topic_3d:=/a2/pointcloud_map_3d",
            "-p", "pointcloud_fallback_topic_3d:=/jt128/front/points",
            "-p", "pointcloud_primary_stale_sec:=10.0",
            "-p", "pointcloud_max_points:=250000",
        ], logs_dir / "map_manager.log", env))

        rclpy.init(args=None)
        node = ClosedLoopProbe(map_root)
        node.spin_until(lambda: node.latest_odom is not None and node.latest_live_cloud is not None, 10.0, "sim odom and live 3D cloud")
        stage("stage2_3d_input_ready", live_cloud_points=point_count(node.latest_live_cloud), initial_pose=node.pose().__dict__)

        node.drive(0.14, 0.00, 0.00, 2.4)
        node.drive(0.00, 0.00, 0.35, 1.6)
        node.drive(0.12, 0.00, 0.00, 2.4)
        node.drive(0.00, -0.08, 0.00, 1.4)
        node.spin_until(lambda: point_count(node.latest_accumulated_map) >= 2500, 12.0, "accumulated 3D map")
        accumulated_points = point_count(node.latest_accumulated_map)
        stage("stage2_mapping_done", accumulated_points=accumulated_points, pose_after_mapping=node.pose().__dict__)

        node.save_map(map_id)
        saved_map_dir = map_root / map_id
        saved_pcd = saved_map_dir / "pointcloud_map_3d.pcd"
        metadata = saved_map_dir / "metadata.yaml"
        if not saved_pcd.exists() or saved_pcd.stat().st_size < 1024:
            raise RuntimeError(f"saved PCD is invalid: {saved_pcd}")
        if not metadata.exists():
            raise RuntimeError(f"metadata missing: {metadata}")
        stage("stage3_map_saved", saved_pcd=str(saved_pcd), pcd_bytes=saved_pcd.stat().st_size, metadata=str(metadata))

        run_pcd_projection(saved_pcd, saved_map_dir, env, logs_dir / "pcd_to_2d_map.log")
        map_yaml = saved_map_dir / "map.yaml"
        if not map_yaml.exists() or not (saved_map_dir / "map.pgm").exists():
            raise RuntimeError("2D projected map was not generated")
        stage("stage3_nav2_projection_saved", map_yaml=str(map_yaml), map_pgm=str(saved_map_dir / "map.pgm"))

        processes.append(run_process([
            "ros2", "run", "map_manager", "pointcloud_map_loader",
            "--ros-args",
            "-p", f"map_root:={map_root}",
            "-p", f"map_id:={map_id}",
            "-p", "output_topic:=/a2/map/pointcloud_3d",
            "-p", "frame_id:=map",
            "-p", "publish_rate_hz:=5.0",
        ], logs_dir / "pointcloud_map_loader.log", env))
        node.latest_loaded_map = None
        node.spin_until(lambda: point_count(node.latest_loaded_map) >= 2500, 10.0, "reloaded 3D map")
        usage = map_usage_score(node.latest_live_cloud, node.latest_loaded_map)
        if usage["matched"] < 50:
            raise RuntimeError(f"loaded map is not being matched by live scan: {usage}")
        stage("stage4_map_loaded_and_localized", loaded_points=point_count(node.latest_loaded_map), map_usage_score=usage)

        current_pose = node.pose()
        if current_pose is None:
            raise RuntimeError("no current pose before planning")
        goal = (current_pose.x + 0.70, current_pose.y)
        path = plan_grid_path(map_yaml, current_pose, goal)
        path_file = run_dir / "planned_path.json"
        path_file.write_text(json.dumps({"start": current_pose.__dict__, "goal": goal, "path": path}, indent=2), encoding="utf-8")
        if len(path) < 4:
            raise RuntimeError("planned path did not contain enough waypoints")
        stage("stage4_planning_done", path_waypoints=len(path), path_file=str(path_file), goal=list(goal))

        before_exec = node.pose()
        movement_records = execute_two_map_steps(node, path)
        after_exec = node.pose()
        stage(
            "stage5_two_steps_executed",
            before=before_exec.__dict__,
            after=after_exec.__dict__,
            movement_records=movement_records,
        )

        report["result"] = "PASS"
        report["artifacts"] = {
            "saved_pcd": str(saved_pcd),
            "metadata": str(metadata),
            "map_yaml": str(map_yaml),
            "map_pgm": str(saved_map_dir / "map.pgm"),
            "planned_path": str(path_file),
            "logs": str(logs_dir),
        }
        report_path = run_dir / "closed_loop_report.json"
        report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"PASS industrial 3D closed loop report={report_path}", flush=True)
        return 0
    except Exception as exc:
        report["result"] = "FAIL"
        report["error"] = str(exc)
        report_path = run_dir / "closed_loop_report.json"
        report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"FAIL industrial 3D closed loop report={report_path} error={exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            if not args.keep_processes:
                stop_processes(processes)


if __name__ == "__main__":
    sys.exit(main())
