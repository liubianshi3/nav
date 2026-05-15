#!/usr/bin/env python3
"""Convert an OctoMap (.bt/.ot) into a Nav2 map.pgm/map.yaml pair.

The heavy lifting is done by the compiled octomap_to_2d_grid_cpp helper so the
conversion can traverse octree leaves directly and preserve free space.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path


def _find_helper() -> str:
    candidates = []
    env_helper = os.environ.get("A2_OCTOMAP_TO_2D_HELPER")
    if env_helper:
        candidates.append(env_helper)

    # Installed ROS layout.
    candidates.extend(
        [
            "/opt/a2_system_ws/install/a2_system/lib/a2_system/octomap_to_2d_grid_cpp",
            str(Path(__file__).resolve().parents[2] / "build" / "a2_system" / "octomap_to_2d_grid_cpp"),
        ]
    )

    path_helper = shutil.which("octomap_to_2d_grid_cpp")
    if path_helper:
        candidates.append(path_helper)

    for candidate in candidates:
        if candidate and os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate

    raise FileNotFoundError(
        "octomap_to_2d_grid_cpp was not found. Build a2_system first, then source install/setup.bash."
    )


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Project OctoMap free/occupied leaves to a Nav2 2D grid")
    parser.add_argument("octomap_path", help="Input .bt or .ot OctoMap file")
    parser.add_argument("--output", "-o", required=True, help="Output directory for map.pgm/map.yaml")
    parser.add_argument("--resolution", type=float, default=0.05, help="Output grid resolution in meters")
    parser.add_argument(
        "--ground-threshold",
        type=float,
        default=0.12,
        help="Occupied voxels below this z-height are treated as traversable ground",
    )
    parser.add_argument(
        "--robot-height",
        type=float,
        default=1.2,
        help="Occupied voxels above this z-height are ignored as overhead structure",
    )
    parser.add_argument(
        "--min-obstacle-points",
        type=int,
        default=1,
        help="Minimum occupied voxel count needed to mark a 2D cell occupied",
    )
    parser.add_argument(
        "--pcd-output",
        default="",
        help="Optional output path for an occupied-voxel ASCII PCD export",
    )
    parser.add_argument("--border-padding", type=float, default=1.0, help="Map border padding in meters")
    return parser.parse_args()


def main() -> int:
    args = _args()
    helper = _find_helper()
    cmd = [
        helper,
        args.octomap_path,
        "--output",
        args.output,
        "--resolution",
        str(args.resolution),
        "--ground-threshold",
        str(args.ground_threshold),
        "--robot-height",
        str(args.robot_height),
        "--min-obstacle-points",
        str(args.min_obstacle_points),
        "--border-padding",
        str(args.border_padding),
    ]
    if args.pcd_output:
        cmd.extend(["--pcd-output", args.pcd_output])
    return subprocess.call(cmd)


if __name__ == "__main__":
    sys.exit(main())
