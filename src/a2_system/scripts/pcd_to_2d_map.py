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
    dilate_radius_cells=2,
):
    """Project classified points to a 2D occupancy grid.

    Returns (OccupancyGrid data as list[int], origin_x, origin_y, grid_width, grid_height).
    Grid values: 100 = occupied, 0 = free, -1 = unknown.
    """
    # Find bounding box from obstacle points
    obs_pts = [(x, y) for x, y, z in zip(xs, ys, zs)
               if ground_threshold <= z <= ceiling_threshold and math.isfinite(x) and math.isfinite(y)]

    if not obs_pts:
        raise RuntimeError("No obstacle points found in z-range — check ground_threshold / ceiling_threshold")

    min_x = min(p[0] for p in obs_pts) - border_padding_m
    max_x = max(p[0] for p in obs_pts) + border_padding_m
    min_y = min(p[1] for p in obs_pts) - border_padding_m
    max_y = max(p[1] for p in obs_pts) + border_padding_m

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
            obstacle_count[row][col] += 1
        elif z < ground_threshold:
            ground_count[row][col] += 1

    # Classify cells
    grid = []
    for r in range(height - 1, -1, -1):  # PGM row 0 = top, so reverse
        for c in range(width):
            if obstacle_count[r][c] >= min_obstacle_points:
                grid.append(100)
            elif ground_count[r][c] >= min_ground_points:
                grid.append(0)
            else:
                grid.append(-1)

    # Dilate occupied cells (morphological)
    if dilate_radius_cells > 0:
        grid = _dilate_occupied(grid, width, height, dilate_radius_cells)

    return grid, origin_x, origin_y, width, height


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
    p.add_argument("--dilate", type=int, default=2,
                   help="Dilation radius in cells for occupied regions (default: 2)")
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
    dilate = int(node.declare_parameter("dilate_radius_cells", 2).value)
    oneshot = bool(node.declare_parameter("oneshot", True).value)

    node.get_logger().info(f"Projecting {pcd_path} → {output_dir}")

    xs, ys, zs = read_pcd(pcd_path)
    node.get_logger().info(f"Read {len(xs)} points from PCD")

    grid, ox, oy, w, h = project(
        xs, ys, zs,
        resolution=resolution,
        ground_threshold=ground_threshold,
        ceiling_threshold=ceiling_threshold,
        min_obstacle_points=min_obstacle_points,
        min_ground_points=min_ground_points,
        border_padding_m=border_padding,
        dilate_radius_cells=dilate,
    )
    write_map_output(grid, w, h, resolution, ox, oy, output_dir)
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

    grid, ox, oy, w, h = project(
        xs, ys, zs,
        resolution=args.resolution,
        ground_threshold=args.ground_threshold,
        ceiling_threshold=args.ceiling_threshold,
        min_obstacle_points=args.min_obstacle_points,
        min_ground_points=args.min_ground_points,
        border_padding_m=args.border_padding,
        dilate_radius_cells=args.dilate,
    )
    write_map_output(grid, w, h, args.resolution, ox, oy, output_dir)


if __name__ == "__main__":
    main()
