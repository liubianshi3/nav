#!/usr/bin/env python3
"""
check_map_quality — assess PCD map suitability for NDT relocalization.

Evaluates:
  1. Voxel coverage (occupied / total cells in bounding box)
  2. Eigenvalue degeneracy (min/max eigenvalue ratio of point distribution)
  3. Hollowness (empty voxel ratio)
  4. Point density uniformity (stddev of points per occupied voxel)

Output: JSON report with pass/fail verdict.
Thresholds are configurable via command-line arguments.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import List

import numpy as np


def read_pcd_xyz(path: str) -> np.ndarray:
    """Read xyz points from ASCII PCD file. Returns (N,3) float64 array."""
    pts: List[tuple[float, float, float]] = []
    fields: List[str] = []
    data_started = False
    with open(path, "r", encoding="ascii", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if data_started:
                parts = line.split()
                if len(parts) < 3:
                    continue
                try:
                    if fields and {"x", "y", "z"}.issubset(fields):
                        xi, yi, zi = fields.index("x"), fields.index("y"), fields.index("z")
                        pts.append((float(parts[xi]), float(parts[yi]), float(parts[zi])))
                    else:
                        pts.append((float(parts[0]), float(parts[1]), float(parts[2])))
                except (ValueError, IndexError):
                    continue
            else:
                key, _, val = line.partition(" ")
                if key.upper() == "FIELDS":
                    fields = val.split()
                if key.upper() == "DATA":
                    data_started = True
    return np.array(pts, dtype=np.float64).reshape((-1, 3))


def evaluate_map(
    points: np.ndarray,
    voxel_size: float = 1.0,
    coverage_min: float = 0.30,
    degeneracy_max: float = 0.01,
    hollowness_max: float = 0.85,
) -> dict:
    """Evaluate PCD map quality for NDT relocalization suitability.

    Returns dict with metrics and a pass/fail verdict.
    """
    report: dict = {
        "total_points": int(points.shape[0]),
        "voxel_size": voxel_size,
    }

    if points.shape[0] < 100:
        report["verdict"] = "FAIL"
        report["verdict_reason"] = f"too few points ({points.shape[0]})"
        return report

    # Bounding box
    x_min, y_min, z_min = points.min(axis=0)
    x_max, y_max, z_max = points.max(axis=0)

    # Round out to voxel grid
    x_bins = max(1, int(math.ceil((x_max - x_min) / voxel_size)))
    y_bins = max(1, int(math.ceil((y_max - y_min) / voxel_size)))
    z_bins = max(1, int(math.ceil((z_max - z_min) / voxel_size)))
    total_voxels = x_bins * y_bins * z_bins

    # Voxelize
    x_idx = np.floor((points[:, 0] - x_min) / voxel_size).astype(np.int64)
    y_idx = np.floor((points[:, 1] - y_min) / voxel_size).astype(np.int64)
    z_idx = np.floor((points[:, 2] - z_min) / voxel_size).astype(np.int64)

    # Combined key
    keys = (x_idx * y_bins * z_bins + y_idx * z_bins + z_idx)
    unique_voxels = len(np.unique(keys))

    coverage = unique_voxels / max(1, total_voxels)
    hollowness = 1.0 - coverage

    # Points per voxel distribution
    pts_per_voxel = np.bincount(keys - keys.min()) if len(keys) > 0 else np.array([])
    pts_per_voxel = pts_per_voxel[pts_per_voxel > 0]
    if len(pts_per_voxel) > 0:
        density_mean = float(np.mean(pts_per_voxel))
        density_std = float(np.std(pts_per_voxel))
        density_cv = density_std / max(1.0, density_mean)  # coefficient of variation
    else:
        density_mean = density_std = density_cv = 0.0

    # Eigenvalue degeneracy — PCA on point distribution
    if points.shape[0] >= 3:
        centered = points - points.mean(axis=0)
        cov = centered.T @ centered / (points.shape[0] - 1)
        eigenvalues = np.linalg.eigvalsh(cov)
        eigenvalues = np.sort(eigenvalues)[::-1]
        e_max = float(eigenvalues[0])
        e_min = float(eigenvalues[-1])
        degeneracy_ratio = e_min / max(1e-12, e_max)
    else:
        e_max = e_min = degeneracy_ratio = 0.0

    report.update({
        "voxels_total": total_voxels,
        "voxels_occupied": unique_voxels,
        "coverage": round(coverage, 4),
        "hollowness": round(hollowness, 4),
        "degeneracy_ratio": round(degeneracy_ratio, 6),
        "eigenvalue_max": round(e_max, 2),
        "eigenvalue_min": round(e_min, 4),
        "density_mean_pts_per_voxel": round(density_mean, 2),
        "density_std_pts_per_voxel": round(density_std, 2),
        "density_cv": round(density_cv, 4),
        "bounds": {
            "x": [round(float(x_min), 2), round(float(x_max), 2)],
            "y": [round(float(y_min), 2), round(float(y_max), 2)],
            "z": [round(float(z_min), 2), round(float(z_max), 2)],
        },
    })

    # Verdict
    failures = []
    if coverage < coverage_min:
        failures.append(f"coverage={coverage:.3f} < {coverage_min}")
    if degeneracy_ratio < degeneracy_max:
        failures.append(f"degeneracy={degeneracy_ratio:.6f} < {degeneracy_max}")
    if hollowness > hollowness_max:
        failures.append(f"hollowness={hollowness:.3f} > {hollowness_max}")

    report["verdict"] = "FAIL" if failures else "PASS"
    report["verdict_reason"] = "; ".join(failures) if failures else "ok"
    report["thresholds"] = {
        "coverage_min": coverage_min,
        "degeneracy_max": degeneracy_max,
        "hollowness_max": hollowness_max,
    }
    return report


def main():
    ap = argparse.ArgumentParser(description="Assess PCD map quality for NDT relocalization")
    ap.add_argument("pcd_path", help="Path to pointcloud_map_3d.pcd")
    ap.add_argument("--output", "-o", default=None, help="JSON output path (default: stdout)")
    ap.add_argument("--voxel-size", type=float, default=1.0, help="NDT voxel size (default: 1.0m)")
    ap.add_argument("--coverage-min", type=float, default=0.30, help="Min coverage ratio (default: 0.30)")
    ap.add_argument("--degeneracy-max", type=float, default=0.01, help="Max degeneracy ratio (default: 0.01)")
    ap.add_argument("--hollowness-max", type=float, default=0.85, help="Max hollowness ratio (default: 0.85)")
    ap.add_argument("--quiet", action="store_true", help="Only print verdict")
    args = ap.parse_args()

    pcd = Path(args.pcd_path)
    if not pcd.exists():
        print(f"ERROR: PCD not found: {pcd}", file=sys.stderr)
        sys.exit(1)

    points = read_pcd_xyz(str(pcd))
    if not args.quiet:
        print(f"[check_map_quality] Read {points.shape[0]} points from {pcd}")

    report = evaluate_map(
        points,
        voxel_size=args.voxel_size,
        coverage_min=args.coverage_min,
        degeneracy_max=args.degeneracy_max,
        hollowness_max=args.hollowness_max,
    )

    report["source_pcd"] = str(pcd.resolve())

    if args.output:
        out_path = Path(args.output)
        out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        if not args.quiet:
            print(f"[check_map_quality] Report → {out_path}")
    else:
        print(json.dumps(report, indent=2))

    if args.quiet:
        print(report["verdict"])

    sys.exit(0 if report["verdict"] == "PASS" else 2)


if __name__ == "__main__":
    main()
