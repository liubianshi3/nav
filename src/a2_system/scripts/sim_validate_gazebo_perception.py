#!/usr/bin/env python3
"""Validate Gazebo perception pipeline with continuous sampling.

Run after launching a2_jt128_gazebo_rviz.launch.py.
Collects multiple frames of each topic and checks minimum data expectations.
"""

import argparse
import math
import re
import struct
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


def _cloud_stats(msg: PointCloud2):
    finite = 0
    rs = []
    step = msg.point_step
    num = msg.width * msg.height
    # cap at 50k points to keep this fast
    limit = min(num, 50000)
    for i in range(limit):
        off = i * step
        x, y, z = struct.unpack_from("<fff", msg.data, off)
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            finite += 1
            rs.append(math.sqrt(x * x + y * y + z * z))
    p50 = None; p95 = None
    if rs:
        s = sorted(rs)
        p50 = s[len(s) // 2]
        p95 = s[int(len(s) * 0.95)]
    return {
        "frame_id": msg.header.frame_id,
        "width": msg.width, "height": msg.height,
        "finite": finite, "total": num,
        "range_min": min(rs) if rs else None,
        "range_max": max(rs) if rs else None,
        "range_p50": p50, "range_p95": p95,
    }


def _occupancy_stats(msg: OccupancyGrid):
    known = sum(1 for v in msg.data if v >= 0)
    lethal = sum(1 for v in msg.data if v >= 65)
    return {
        "frame_id": msg.header.frame_id,
        "width": msg.info.width, "height": msg.info.height,
        "resolution": msg.info.resolution,
        "known_cells": known, "lethal_cells": lethal,
    }


def _parse_status(text: str) -> dict[str, str]:
    """Parse semicolon-separated key=value status string."""
    out: dict[str, str] = {}
    for part in text.split(";"):
        part = part.strip()
        if "=" in part:
            k, v = part.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    parser.add_argument("--min-frames", type=int, default=3)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("sim_validate_gazebo_perception")

    samples: dict[str, list] = {
        "raw": [], "ground": [], "obstacle": [],
        "traversability": [], "trav_obs": [], "status": [],
    }

    def _save(key, data):
        samples[key].append(data)

    def cb_raw(msg):
        _save("raw", _cloud_stats(msg))

    def cb_ground(msg):
        _save("ground", _cloud_stats(msg))

    def cb_obstacle(msg):
        _save("obstacle", _cloud_stats(msg))

    def cb_trav(msg):
        _save("traversability", _occupancy_stats(msg))

    def cb_tobs(msg):
        _save("trav_obs", _cloud_stats(msg))

    def cb_status(msg):
        _save("status", _parse_status(msg.data))

    node.create_subscription(PointCloud2, "/jt128/front/points", cb_raw, 10)
    node.create_subscription(PointCloud2, "/a2/ground/points", cb_ground, 10)
    node.create_subscription(PointCloud2, "/a2/obstacle/points", cb_obstacle, 10)
    node.create_subscription(OccupancyGrid, "/a2/traversability", cb_trav, 10)
    node.create_subscription(PointCloud2, "/a2/traversability/obstacle_points", cb_tobs, 10)
    node.create_subscription(String, "/a2/perception/ground_segmentation/status", cb_status, 10)

    deadline = time.time() + args.timeout_sec
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.3)
        # stop early when we have enough frames for every topic
        if all(len(v) >= args.min_frames for v in samples.values()):
            break

    node.destroy_node()
    rclpy.shutdown()

    def _best(seq, key, prefer_larger=True):
        """Pick the best sample for validation — largest width/processed/etc."""
        vals = [s.get(key) for s in seq if s.get(key) is not None]
        if not vals:
            return None
        return max(vals) if prefer_larger else min(vals)

    def _latest(seq, key):
        for s in reversed(seq):
            v = s.get(key)
            if v is not None:
                return v
        return None

    all_pass = True

    def check(label, cond, detail=""):
        nonlocal all_pass
        status = "PASS" if cond else "FAIL"
        if not cond:
            all_pass = False
        print(f"  [{status}] {label}: {detail}")

    print(f"\n=== Gazebo Perception Validation ({len(samples['raw'])} raw frames, {len(samples['status'])} status frames) ===\n")

    # ── Raw cloud ──
    raw_best = _best(samples["raw"], "range_p95") or {}
    print("[Raw /jt128/front/points — best p95 frame]")
    for k in ("frame_id", "width", "height", "finite", "total",
              "range_min", "range_max", "range_p50", "range_p95"):
        print(f"  {k}: {raw_best.get(k)}")
    check("raw width > 1000", (raw_best.get("width") or 0) > 1000,
          f"width={raw_best.get('width')}")
    check("raw finite > 1000", (raw_best.get("finite") or 0) > 1000,
          f"finite={raw_best.get('finite')}")
    check("raw range_p95 > 1.0m",
          (raw_best.get("range_p95") or 0) > 1.0,
          f"p95={raw_best.get('range_p95'):.3f}" if raw_best.get("range_p95") else "no_p95")

    # ── Ground ──
    ground_best = _best(samples["ground"], "width") or {}
    print("\n[Ground /a2/ground/points — best width frame]")
    for k in ("frame_id", "width", "height", "finite"):
        print(f"  {k}: {ground_best.get(k)}")
    check("ground width >= 100", (ground_best.get("width") or 0) >= 100,
          f"width={ground_best.get('width')}")

    # ── Obstacle ──
    obstacle_best = _best(samples["obstacle"], "width") or {}
    print("\n[Obstacle /a2/obstacle/points — best width frame]")
    for k in ("frame_id", "width", "height", "finite"):
        print(f"  {k}: {obstacle_best.get(k)}")
    check("obstacle width >= 10", (obstacle_best.get("width") or 0) >= 10,
          f"width={obstacle_best.get('width')}")

    # ── Traversability ──
    trav_best = _best(samples["traversability"], "known_cells") or {}
    print("\n[Traversability /a2/traversability — best known_cells frame]")
    for k in ("frame_id", "width", "height", "resolution", "known_cells", "lethal_cells"):
        print(f"  {k}: {trav_best.get(k)}")
    check("trav known_cells > 100", (trav_best.get("known_cells") or 0) > 100,
          f"known={trav_best.get('known_cells')}")

    # ── Trav obstacle points ──
    tobs_best = _best(samples["trav_obs"], "width") or {}
    print("\n[TravObstacle /a2/traversability/obstacle_points — best width frame]")
    for k in ("frame_id", "width", "finite"):
        print(f"  {k}: {tobs_best.get(k)}")

    # ── Status ──
    status_best = _best(samples["status"], "processed") or {}
    print("\n[Status /a2/perception/ground_segmentation/status — best processed frame]")
    for k in ("state", "ready", "reason", "processed", "empty",
              "input_points", "filtered_points",
              "dropped_min_range", "dropped_self_filter"):
        print(f"  {k}: {status_best.get(k, 'N/A')}")
    check("status processed > 0", int(status_best.get("processed") or 0) > 0,
          f"processed={status_best.get('processed')}")
    check("status filtered_points > 100",
          int(status_best.get("filtered_points") or 0) > 100,
          f"filtered_points={status_best.get('filtered_points')}")

    print()
    if all_pass:
        print("=== OVERALL: PASS ===")
    else:
        print("=== OVERALL: FAIL ===")
        sys.exit(1)


if __name__ == "__main__":
    main()
