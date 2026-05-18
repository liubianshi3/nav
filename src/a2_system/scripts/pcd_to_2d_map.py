#!/usr/bin/env python3
"""
pcd_to_2d_map — project a 3D PCD pointcloud map to a Nav2-compatible 2D occupancy grid.

Two run modes:
  ros2 run a2_system pcd_to_2d_map.py --ros-args -p pcd_path:=<path> -p output_dir:=<dir>
  python3 pcd_to_2d_map.py <pcd_path> [--output <dir>] [--resolution <m>]

Z-height classification:
  z < ground_threshold   → ground (marks free / scanned space)
  ground ≤ z ≤ ceiling   → obstacle (marks occupied cells)
  z > ceiling            → ignored

Outputs:  map.yaml + map.pgm  (Nav2 map_server compatible)
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
from pathlib import Path
from typing import List, Tuple

# ---------------------------------------------------------------------------
# PCD parser — supports ASCII (what map_manager writes) and binary FLOAT32
# ---------------------------------------------------------------------------


def _parse_pcd_header(path: str):
    """Return dict of PCD header fields and byte offset where point data starts."""
    header = {}
    offset = 0
    with open(path, "rb") as fh:
        for raw_line in fh:
            offset += len(raw_line)
            line = raw_line.decode("ascii", errors="replace").strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            key = parts[0]
            if key == "DATA":
                header["DATA"] = parts[1] if len(parts) > 1 else "ascii"
                break
            header[key] = parts[1:] if len(parts) > 1 else ""
    return header, offset


def _point_step(fields: List[str], sizes: List[int]) -> int:
    return sum(sizes)


def read_pcd(path: str):
    """Read a PCD file and return (x, y, z) arrays as lists of floats.

    Supports ASCII (v0.7) and binary FLOAT32 with fields "x y z".
    """
    header, data_offset = _parse_pcd_header(path)
    fields = header.get("FIELDS", [])
    sizes = [int(s) for s in header.get("SIZE", [])]
    types_ = header.get("TYPE", [])
    data_type = header.get("DATA", "ascii")
    width = int(header["WIDTH"][0]) if "WIDTH" in header else 0
    height = int(header["HEIGHT"][0]) if "HEIGHT" in header else 0
    total = width * height
    if total <= 0:
        raise RuntimeError(f"PCD has zero points: WIDTH={width} HEIGHT={height}")

    x_idx = fields.index("x") if "x" in fields else 0
    y_idx = fields.index("y") if "y" in fields else 1
    z_idx = fields.index("z") if "z" in fields else 2

    step = _point_step(fields, sizes)
    x_offset = sum(sizes[:x_idx])
    y_offset = sum(sizes[:y_idx])
    z_offset = sum(sizes[:z_idx])

    xs, ys, zs = [], [], []

    with open(path, "rb") as fh:
        fh.seek(data_offset)
        body = fh.read()

    if data_type == "ascii":
        lines = body.decode("ascii", errors="replace").strip().split("\n")
        for line in lines:
            parts = line.split()
            if len(parts) < 3:
                continue
            try:
                x, y, z = float(parts[x_idx]), float(parts[y_idx]), float(parts[z_idx])
            except (ValueError, IndexError):
                continue
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            xs.append(x)
            ys.append(y)
            zs.append(z)
    else:
        endian = ">" if "binary" in str(data_type) and "big" not in str(data_type) else "<"
        # default to little-endian for raw binary
        if "big" in str(data_type).lower():
            endian = ">"
        unpack_float = struct.Struct(f"{endian}f").unpack_from
        for i in range(total):
            base = i * step
            x = unpack_float(body, base + x_offset)[0]
            y = unpack_float(body, base + y_offset)[0]
            z = unpack_float(body, base + z_offset)[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            xs.append(x)
            ys.append(y)
            zs.append(z)

    return xs, ys, zs


# ---------------------------------------------------------------------------
# Projection
# ---------------------------------------------------------------------------


def project(
    xs, ys, zs,
    resolution=0.05,
    ground_threshold=0.08,
    ceiling_threshold=2.0,
    min_obstacle_points=2,
    min_ground_points=1,
    border_padding_m=1.0,
    dilate_radius_cells=0,
    ignore_obstacles_within_radius=0.0,
    clear_radius_around_origin=0.0,
):
    """Project classified points to a 2D occupancy grid.

    Returns ``(grid, origin_x, origin_y, width, height, cleared_cells)``.
    Grid values: 100 = occupied, 0 = free, -1 = unknown.

    ``ignore_obstacles_within_radius`` (meters, world frame around (0, 0))
    skips obstacle accumulation for points whose XY radius is below this
    value. It does NOT mark those cells occupied; it just keeps near-origin
    self-shell / leg / near-field noise out of the obstacle classifier.

    ``clear_radius_around_origin`` (meters, world frame around (0, 0)) is
    applied AFTER classification and FORCES every cell whose center lies
    inside the disk to free (0). Used to guarantee the static map publishes a
    real free patch around the build origin / robot start location.
    """
    ignore_radius_sq = float(ignore_obstacles_within_radius) ** 2

    # Bounding box from obstacle points (also subject to the ignore radius so
    # a tight self-shell does not inflate the map bounds).
    obs_pts = [
        (x, y) for x, y, z in zip(xs, ys, zs)
        if ground_threshold <= z <= ceiling_threshold
        and math.isfinite(x) and math.isfinite(y)
        and (ignore_radius_sq == 0.0 or (x * x + y * y) > ignore_radius_sq)
    ]

    if not obs_pts:
        raise RuntimeError("No obstacle points found in z-range — check ground_threshold / ceiling_threshold")

    min_x = min(p[0] for p in obs_pts) - border_padding_m
    max_x = max(p[0] for p in obs_pts) + border_padding_m
    min_y = min(p[1] for p in obs_pts) - border_padding_m
    max_y = max(p[1] for p in obs_pts) + border_padding_m

    # If we are going to force-clear a disk around the world origin, the grid
    # must actually contain that disk. Without this, a scene whose obstacles
    # are all far from (0, 0) — combined with ignore_obstacles_within_radius
    # dropping the near-origin self-shell — would put the origin outside the
    # grid and clear_disk_around_world_point would silently no-op.
    if clear_radius_around_origin > 0.0:
        r = float(clear_radius_around_origin)
        min_x = min(min_x, -r - border_padding_m)
        max_x = max(max_x, r + border_padding_m)
        min_y = min(min_y, -r - border_padding_m)
        max_y = max(max_y, r + border_padding_m)

    # Snap origin to resolution grid
    origin_x = math.floor(min_x / resolution) * resolution
    origin_y = math.floor(min_y / resolution) * resolution

    width = int(math.ceil((max_x - origin_x) / resolution)) + 1
    height = int(math.ceil((max_y - origin_y) / resolution)) + 1

    # Accumulators
    obstacle_count = [[0] * width for _ in range(height)]
    ground_count = [[0] * width for _ in range(height)]

    for x, y, z in zip(xs, ys, zs):
        col = int((x - origin_x) / resolution)
        row = int((y - origin_y) / resolution)
        if not (0 <= col < width and 0 <= row < height):
            continue
        if ground_threshold <= z <= ceiling_threshold:
            # Skip near-origin obstacle accumulation only; do not mark it
            # free. The cell can still become free via ground hits or via the
            # explicit clear_radius_around_origin pass below.
            if ignore_radius_sq > 0.0 and (x * x + y * y) <= ignore_radius_sq:
                continue
            obstacle_count[row][col] += 1
        elif z < ground_threshold:
            ground_count[row][col] += 1

    # Classify cells (PGM row 0 = top, so we walk world rows in reverse)
    grid = []
    for r in range(height - 1, -1, -1):
        for c in range(width):
            if obstacle_count[r][c] >= min_obstacle_points:
                grid.append(100)
            elif ground_count[r][c] >= min_ground_points:
                grid.append(0)
            else:
                grid.append(-1)

    # Projection-stage dilation defaults to 0: Nav2 global / local costmap
    # inflation already adds the safety buffer, so double-inflating here was
    # producing a permanent ~0.5 m occupied shell on the JT128 maps.
    if dilate_radius_cells > 0:
        grid = _dilate_occupied(grid, width, height, dilate_radius_cells)

    cleared_cells = 0
    if clear_radius_around_origin > 0.0:
        cleared_cells = clear_disk_around_world_point(
            grid, width, height, origin_x, origin_y, resolution,
            0.0, 0.0, float(clear_radius_around_origin),
        )

    return grid, origin_x, origin_y, width, height, cleared_cells


def clear_disk_around_world_point(
    grid: list,
    width: int,
    height: int,
    origin_x: float,
    origin_y: float,
    resolution: float,
    center_x: float,
    center_y: float,
    radius: float,
) -> int:
    """Force every grid cell whose center lies within ``radius`` (meters) of
    (``center_x``, ``center_y``) to free (0).

    ``grid`` is the flat PGM-ordered list produced by :func:`project`
    (PGM row 0 = top), so cell (pgm_row, c) corresponds to world row
    ``world_r = height - 1 - pgm_row``, and the cell center is
    ``(origin_x + (c + 0.5) * resolution, origin_y + (world_r + 0.5) * resolution)``.

    Returns the number of cells cleared. Short-term patch: ideally a future
    revision should carve a robot-footprint tube along the recorded mapping
    trajectory rather than only clearing a disk at world origin.
    """
    if radius <= 0.0 or resolution <= 0.0:
        return 0
    radius_sq = radius * radius
    cleared = 0
    for pgm_row in range(height):
        world_r = height - 1 - pgm_row
        wy = origin_y + (world_r + 0.5) * resolution
        dy = wy - center_y
        if dy * dy > radius_sq:
            continue
        for c in range(width):
            wx = origin_x + (c + 0.5) * resolution
            dx = wx - center_x
            if dx * dx + dy * dy <= radius_sq:
                idx = pgm_row * width + c
                if grid[idx] != 0:
                    grid[idx] = 0
                    cleared += 1
    return cleared


def _dilate_occupied(grid, width, height, radius):
    occupied_coords = set()
    for r in range(height):
        for c in range(width):
            if grid[r * width + c] == 100:
                occupied_coords.add((r, c))

    if not occupied_coords:
        return grid

    new = list(grid)
    for r in range(height):
        for c in range(width):
            if grid[r * width + c] == 100:
                continue
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if dr * dr + dc * dc > radius * radius:
                        continue
                    nr, nc = r + dr, c + dc
                    if (nr, nc) in occupied_coords:
                        new[r * width + c] = 100
                        break
                else:
                    continue
                break
    return new


# ---------------------------------------------------------------------------
# PGM / YAML output (Nav2 format)
# ---------------------------------------------------------------------------


def write_map_output(grid, width, height, resolution, origin_x, origin_y, output_dir):
    """Write map.pgm and map.yaml into output_dir."""
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    # PGM (P5 binary)
    # 0 = black = occupied, 254 = white = free, 205 = gray = unknown
    pgm_path = out / "map.pgm"
    with open(pgm_path, "wb") as fh:
        fh.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        for val in grid:
            if val == 100:
                pixel = 0
            elif val == 0:
                pixel = 254
            else:  # -1 unknown
                pixel = 205
            fh.write(bytes([pixel]))

    yaml_path = out / "map.yaml"
    yaml_text = (
        f"image: map.pgm\n"
        f"resolution: {resolution:.4f}\n"
        f"origin: [{origin_x:.4f}, {origin_y:.4f}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.25\n"
        f"mode: trinary\n"
    )
    yaml_path.write_text(yaml_text, encoding="utf-8")

    print(f"[pcd_to_2d_map] Wrote {pgm_path} ({width}x{height}, res={resolution}m)")
    print(f"[pcd_to_2d_map] Wrote {yaml_path}")
    print(f"[pcd_to_2d_map] Grid bounds: x=[{origin_x:.2f}, {origin_x + width * resolution:.2f}], "
          f"y=[{origin_y:.2f}, {origin_y + height * resolution:.2f}]")
    occ_pct = sum(1 for v in grid if v == 100) / max(len(grid), 1) * 100
    free_pct = sum(1 for v in grid if v == 0) / max(len(grid), 1) * 100
    unk_pct = sum(1 for v in grid if v == -1) / max(len(grid), 1) * 100
    print(f"[pcd_to_2d_map] Cells: {occ_pct:.1f}% occupied, {free_pct:.1f}% free, {unk_pct:.1f}% unknown")


def _log_projection_summary(
    *,
    ignore_obstacles_within_radius: float,
    clear_radius_around_origin: float,
    cleared_cells: int,
    dilate_radius_cells: int,
    log=print,
) -> None:
    log(
        f"[pcd_to_2d_map] dilate_radius_cells={dilate_radius_cells}, "
        f"ignore_obstacles_within_radius={ignore_obstacles_within_radius:.3f} m, "
        f"clear_radius_around_origin={clear_radius_around_origin:.3f} m"
    )
    if clear_radius_around_origin > 0.0:
        log(f"[pcd_to_2d_map] clear_disk_around_world_point cleared {cleared_cells} cells")


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------


def _cli_args():
    p = argparse.ArgumentParser(description="Project a PCD pointcloud map to a Nav2 2D occupancy grid")
    p.add_argument("pcd_path", nargs="?", help="Path to PCD file")
    p.add_argument("--output", "-o", default=None, help="Output directory (default: same as PCD)")
    p.add_argument("--resolution", "-r", type=float, default=0.05, help="Grid resolution in meters (default: 0.05)")
    p.add_argument("--ground-threshold", type=float, default=0.08,
                   help="Max z-height for ground points in meters (default: 0.08)")
    p.add_argument("--ceiling-threshold", type=float, default=2.0,
                   help="Max z-height for obstacle points in meters (default: 2.0)")
    p.add_argument("--min-obstacle-points", type=int, default=2,
                   help="Minimum points per cell to mark occupied (default: 2)")
    p.add_argument("--min-ground-points", type=int, default=1,
                   help="Minimum ground points per cell to mark free (default: 1)")
    p.add_argument("--border-padding", type=float, default=1.0,
                   help="Padding around obstacle bounds in meters (default: 1.0)")
    p.add_argument("--dilate", type=int, default=0,
                   help=(
                       "Dilation radius in cells for occupied regions (default: 0). "
                       "Keep at 0 in production: Nav2 global/local costmap "
                       "inflation already adds the safety buffer; pass a small "
                       "positive value (e.g. 1) only if you must inflate at "
                       "projection time."
                   ))
    p.add_argument("--ignore-obstacles-within-radius", type=float, default=0.0,
                   help=(
                       "Skip obstacle accumulation for points whose XY radius "
                       "from the world origin (0, 0) is below this value "
                       "(meters). Default 0.0 (no-op). Use ~0.45-0.5 to drop "
                       "self-shell / leg / near-field noise the SLAM build "
                       "kept around the start pose."
                   ))
    p.add_argument("--clear-radius-around-origin", type=float, default=0.0,
                   help=(
                       "After classification, force every cell within this "
                       "radius (meters) of the world origin (0, 0) to free. "
                       "Default 0.0 (no-op). Use ~0.45-0.5 to guarantee the "
                       "static map exposes a real free patch around the robot "
                       "start location."
                   ))
    return p.parse_args()


def _ros_entry():
    """Run as ROS 2 node. Returns True on success, False if CLI fallback needed."""
    import rclpy
    from rclpy.node import Node

    # Pass only ROS args to rclpy (filter out our own CLI args)
    ros_args = []
    non_ros_args = []
    for a in sys.argv:
        if a.startswith("--ros-args") or a.startswith("-r") or a.startswith("__"):
            ros_args.append(a)
        else:
            non_ros_args.append(a)

    # Only enter ROS mode if user explicitly passed --ros-args
    if "--ros-args" not in sys.argv:
        return False

    rclpy.init(args=ros_args)
    node = Node("pcd_to_2d_map_tool")

    pcd_path = node.declare_parameter("pcd_path", "").value
    if not pcd_path:
        node.get_logger().error("pcd_path parameter is required")
        rclpy.shutdown()
        sys.exit(1)

    output_dir = node.declare_parameter("output_dir", str(Path(pcd_path).parent)).value
    resolution = float(node.declare_parameter("resolution", 0.05).value)
    ground_threshold = float(node.declare_parameter("ground_threshold", 0.08).value)
    ceiling_threshold = float(node.declare_parameter("ceiling_threshold", 2.0).value)
    min_obstacle_points = int(node.declare_parameter("min_obstacle_points", 2).value)
    min_ground_points = int(node.declare_parameter("min_ground_points", 1).value)
    border_padding = float(node.declare_parameter("border_padding_m", 1.0).value)
    dilate = int(node.declare_parameter("dilate_radius_cells", 0).value)
    ignore_obstacles_within_radius = float(
        node.declare_parameter("ignore_obstacles_within_radius", 0.0).value
    )
    clear_radius_around_origin = float(
        node.declare_parameter("clear_radius_around_origin", 0.0).value
    )
    oneshot = bool(node.declare_parameter("oneshot", True).value)

    node.get_logger().info(f"Projecting {pcd_path} → {output_dir}")

    xs, ys, zs = read_pcd(pcd_path)
    node.get_logger().info(f"Read {len(xs)} points from PCD")

    grid, ox, oy, w, h, cleared_cells = project(
        xs, ys, zs,
        resolution=resolution,
        ground_threshold=ground_threshold,
        ceiling_threshold=ceiling_threshold,
        min_obstacle_points=min_obstacle_points,
        min_ground_points=min_ground_points,
        border_padding_m=border_padding,
        dilate_radius_cells=dilate,
        ignore_obstacles_within_radius=ignore_obstacles_within_radius,
        clear_radius_around_origin=clear_radius_around_origin,
    )
    write_map_output(grid, w, h, resolution, ox, oy, output_dir)
    _log_projection_summary(
        ignore_obstacles_within_radius=ignore_obstacles_within_radius,
        clear_radius_around_origin=clear_radius_around_origin,
        cleared_cells=cleared_cells,
        dilate_radius_cells=dilate,
        log=node.get_logger().info,
    )
    node.get_logger().info("Done.")
    if oneshot:
        rclpy.shutdown()
    else:
        rclpy.spin(node)
    return True


def main():
    # -- ROS 2 mode (only when --ros-args is present) -----------------------
    if "--ros-args" in sys.argv:
        try:
            if _ros_entry():
                return
        except SystemExit:
            raise
        except Exception:
            # Fall through to CLI mode
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()

    # -- CLI mode ------------------------------------------------------------
    args = _cli_args()
    if not args.pcd_path:
        print("Usage: pcd_to_2d_map.py <pcd_path> [--output <dir>] [--resolution <m>]", file=sys.stderr)
        sys.exit(1)

    pcd_path = args.pcd_path
    if not os.path.isfile(pcd_path):
        print(f"ERROR: PCD file not found: {pcd_path}", file=sys.stderr)
        sys.exit(1)

    output_dir = args.output or str(Path(pcd_path).parent)

    xs, ys, zs = read_pcd(pcd_path)
    print(f"[pcd_to_2d_map] Read {len(xs)} points from {pcd_path}")

    grid, ox, oy, w, h, cleared_cells = project(
        xs, ys, zs,
        resolution=args.resolution,
        ground_threshold=args.ground_threshold,
        ceiling_threshold=args.ceiling_threshold,
        min_obstacle_points=args.min_obstacle_points,
        min_ground_points=args.min_ground_points,
        border_padding_m=args.border_padding,
        dilate_radius_cells=args.dilate,
        ignore_obstacles_within_radius=args.ignore_obstacles_within_radius,
        clear_radius_around_origin=args.clear_radius_around_origin,
    )
    write_map_output(grid, w, h, args.resolution, ox, oy, output_dir)
    _log_projection_summary(
        ignore_obstacles_within_radius=args.ignore_obstacles_within_radius,
        clear_radius_around_origin=args.clear_radius_around_origin,
        cleared_cells=cleared_cells,
        dilate_radius_cells=args.dilate,
    )


if __name__ == "__main__":
    main()
