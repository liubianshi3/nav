#!/usr/bin/env python3
"""Validate simple_car + JT128-like Gazebo perception pipeline.

This validates the generic perception pipeline, not A2-specific
self-filter or real JT128 noise.

Run after: ros2 launch a2_bringup simple_car_jt128_gazebo_rviz.launch.py
"""

import argparse
import math
import struct
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


def _field_offsets(msg: PointCloud2):
    """Return (x_off, y_off, z_off) from msg.fields, or None if missing."""
    x_off = y_off = z_off = None
    for f in msg.fields:
        if f.name == "x":
            x_off = f.offset
        elif f.name == "y":
            y_off = f.offset
        elif f.name == "z":
            z_off = f.offset
    return x_off, y_off, z_off


def cloud_stats(msg: PointCloud2, max_pts=50000):
    """Return stats dict for a PointCloud2 message.

    Uses field-based offsets (fallback to 0,4,8 if single float triplet).
    """
    x_off, y_off, z_off = _field_offsets(msg)
    if x_off is None or y_off is None or z_off is None:
        # fallback: assume standard xyz triplet at 0,4,8
        x_off, y_off, z_off = 0, 4, 8

    finite = 0
    rs = []
    step = msg.point_step
    total = msg.width * msg.height
    limit = min(total, max_pts)

    for i in range(limit):
        off = i * step
        x = struct.unpack_from("<f", msg.data, off + x_off)[0]
        y = struct.unpack_from("<f", msg.data, off + y_off)[0]
        z = struct.unpack_from("<f", msg.data, off + z_off)[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            finite += 1
            rs.append(math.sqrt(x * x + y * y + z * z))

    p50 = p95 = None
    if rs:
        s = sorted(rs)
        p50 = s[len(s) // 2]
        idx95 = min(len(s) - 1, int(round(len(s) * 0.95)))
        p95 = s[idx95] if idx95 >= 0 else None

    return {
        "frame_id": msg.header.frame_id,
        "width": msg.width, "height": msg.height,
        "finite": finite, "total": total,
        "range_min": min(rs) if rs else None,
        "range_max": max(rs) if rs else None,
        "range_p50": p50, "range_p95": p95,
    }


def occupancy_stats(msg: OccupancyGrid):
    known = sum(1 for v in msg.data if v >= 0)
    unknown = sum(1 for v in msg.data if v == -1)
    lethal = sum(1 for v in msg.data if v >= 90)
    return {
        "frame_id": msg.header.frame_id,
        "width": msg.info.width, "height": msg.info.height,
        "resolution": msg.info.resolution,
        "known_cells": known, "unknown_cells": unknown,
        "lethal_cells": lethal,
        "max_cost": max(msg.data) if msg.data else 0,
    }


def parse_status(text: str):
    out = {}
    for part in text.split(";"):
        part = part.strip()
        if "=" in part:
            k, v = part.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def safe_int(val, default=0):
    try:
        return int(val)
    except (ValueError, TypeError):
        return default


def best_sample(seq, key, prefer_larger=True):
    """Return the full sample dict whose key value is best."""
    vals = [(s.get(key), s) for s in seq if s.get(key) is not None]
    if not vals:
        return {}
    vals.sort(key=lambda x: x[0], reverse=prefer_larger)
    return vals[0][1]


def best_status_by_processed(seq):
    """Return the status dict with the largest int(processed)."""
    best_v = -1
    best_s = {}
    for s in seq:
        v = safe_int(s.get("processed", "0"))
        if v > best_v:
            best_v = v
            best_s = s
    return best_s


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--timeout-sec", type=float, default=25)
    p.add_argument("--min-frames", type=int, default=3)
    p.add_argument("--raw-min-points", type=int, default=1000)
    p.add_argument("--raw-min-range-p95", type=float, default=2.0)
    p.add_argument("--ground-min-points", type=int, default=100)
    p.add_argument("--obstacle-min-points", type=int, default=1)
    p.add_argument("--traversability-min-known-cells", type=int, default=100)
    p.add_argument("--trav-obs-min-points", type=int, default=1)
    args = p.parse_args()

    print("This validates the generic perception pipeline, "
          "not A2-specific self-filter or real JT128 noise.\n")

    rclpy.init()
    node = rclpy.create_node("sim_validate_simple_car")

    samples = {"raw": [], "ground": [], "obstacle": [],
               "trav": [], "trav_obs": [], "status": []}

    node.create_subscription(PointCloud2, "/jt128/front/points",
                             lambda m: samples["raw"].append(cloud_stats(m)), 10)
    node.create_subscription(PointCloud2, "/a2/ground/points",
                             lambda m: samples["ground"].append(cloud_stats(m)), 10)
    node.create_subscription(PointCloud2, "/a2/obstacle/points",
                             lambda m: samples["obstacle"].append(cloud_stats(m)), 10)
    node.create_subscription(OccupancyGrid, "/a2/traversability",
                             lambda m: samples["trav"].append(occupancy_stats(m)), 10)
    node.create_subscription(PointCloud2, "/a2/traversability/obstacle_points",
                             lambda m: samples["trav_obs"].append(cloud_stats(m)), 10)
    node.create_subscription(String, "/a2/perception/ground_segmentation/status",
                             lambda m: samples["status"].append(parse_status(m.data)), 10)

    deadline = time.time() + args.timeout_sec
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.3)
        if all(len(v) >= args.min_frames for v in samples.values()):
            break

    node.destroy_node()
    rclpy.shutdown()

    all_pass = True
    failures = []

    def check(label, cond, detail=""):
        nonlocal all_pass
        ok = "PASS" if cond else "FAIL"
        if not cond:
            all_pass = False
            failures.append(f"{label}: {detail}")
        print(f"  [{ok}] {label}: {detail}")

    n_raw = len(samples["raw"])
    n_gnd = len(samples["ground"])
    n_obs = len(samples["obstacle"])
    n_trav = len(samples["trav"])
    n_tobs = len(samples["trav_obs"])
    n_st = len(samples["status"])
    print(f"\n=== Simple Car Perception Validation "
          f"(raw={n_raw} ground={n_gnd} obstacle={n_obs} "
          f"trav={n_trav} trav_obs={n_tobs} status={n_st}) ===\n")

    if n_raw == 0 and n_gnd == 0 and n_obs == 0:
        print("  [FAIL] no samples received for any topic")
        print("\n=== OVERALL: FAIL ===")
        sys.exit(1)

    # ── Raw ──
    raw = best_sample(samples["raw"], "range_p95")
    print("[Raw /jt128/front/points]")
    for k in ("frame_id", "width", "height", "finite", "total",
              "range_min", "range_max", "range_p50", "range_p95"):
        print(f"  {k}: {raw.get(k)}")
    check("raw received", n_raw >= args.min_frames, f"frames={n_raw}")
    check("raw frame_id == jt128_front_link",
          raw.get("frame_id") == "jt128_front_link",
          f"frame_id={raw.get('frame_id')}")
    check(f"raw finite > {args.raw_min_points}",
          (raw.get("finite") or 0) > args.raw_min_points,
          f"finite={raw.get('finite')}")
    check(f"raw range_p95 > {args.raw_min_range_p95}m",
          (raw.get("range_p95") or 0) > args.raw_min_range_p95,
          f"p95={raw.get('range_p95'):.3f}" if raw.get("range_p95") else "no_p95")

    # ── Ground ──
    gnd = best_sample(samples["ground"], "finite")
    print("\n[Ground /a2/ground/points]")
    for k in ("frame_id", "width", "height", "finite"):
        print(f"  {k}: {gnd.get(k)}")
    check("ground received", n_gnd >= args.min_frames, f"frames={n_gnd}")
    check("ground frame_id == map",
          gnd.get("frame_id") == "map", f"frame_id={gnd.get('frame_id')}")
    check(f"ground finite >= {args.ground_min_points}",
          (gnd.get("finite") or 0) >= args.ground_min_points,
          f"finite={gnd.get('finite')}")

    # ── Obstacle ──
    obs = best_sample(samples["obstacle"], "finite")
    print("\n[Obstacle /a2/obstacle/points]")
    for k in ("frame_id", "width", "height", "finite"):
        print(f"  {k}: {obs.get(k)}")
    check("obstacle received", n_obs >= args.min_frames, f"frames={n_obs}")
    check("obstacle frame_id == map",
          obs.get("frame_id") == "map", f"frame_id={obs.get('frame_id')}")
    check(f"obstacle finite >= {args.obstacle_min_points}",
          (obs.get("finite") or 0) >= args.obstacle_min_points,
          f"finite={obs.get('finite')}")

    # ── Traversability ──
    trav = best_sample(samples["trav"], "known_cells")
    print("\n[Traversability /a2/traversability]")
    for k in ("frame_id", "width", "height", "resolution",
              "known_cells", "unknown_cells", "lethal_cells", "max_cost"):
        print(f"  {k}: {trav.get(k)}")
    check("trav received", n_trav >= args.min_frames, f"frames={n_trav}")
    check("trav frame_id == map",
          trav.get("frame_id") == "map", f"frame_id={trav.get('frame_id')}")
    check(f"trav known_cells > {args.traversability_min_known_cells}",
          (trav.get("known_cells") or 0) > args.traversability_min_known_cells,
          f"known={trav.get('known_cells')}")

    # ── Trav Obstacle Points ──
    tobs = best_sample(samples["trav_obs"], "finite")
    print("\n[TravObstacle /a2/traversability/obstacle_points]")
    for k in ("frame_id", "width", "height", "finite"):
        print(f"  {k}: {tobs.get(k)}")
    check("trav_obs received", n_tobs >= args.min_frames, f"frames={n_tobs}")
    check("trav_obs frame_id == base_link",
          tobs.get("frame_id") == "base_link",
          f"frame_id={tobs.get('frame_id')}")
    check(f"trav_obs finite >= {args.trav_obs_min_points}",
          (tobs.get("finite") or 0) >= args.trav_obs_min_points,
          f"finite={tobs.get('finite')}")

    # ── Status (numeric-aware best) ──
    st = best_status_by_processed(samples["status"])
    print("\n[Status /a2/perception/ground_segmentation/status]")
    for k in ("state", "ready", "reason", "processed", "empty",
              "input_points", "filtered_points", "dropped_self_filter",
              "dropped_min_range"):
        print(f"  {k}: {st.get(k, 'N/A')}")
    check("status received", n_st >= args.min_frames, f"frames={n_st}")
    check("status ready == true",
          st.get("ready") == "true", f"ready={st.get('ready')}")
    check("status processed > 0",
          safe_int(st.get("processed")) > 0,
          f"processed={st.get('processed')}")
    check("status filtered_points > 0",
          safe_int(st.get("filtered_points")) > 0,
          f"filtered_points={st.get('filtered_points')}")

    print()
    if all_pass:
        print("=== OVERALL: PASS ===")
        print("This validates the generic perception pipeline, "
              "not A2-specific self-filter or real JT128 noise.")
    else:
        print("=== OVERALL: FAIL ===")
        print("Failures:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)


if __name__ == "__main__":
    main()
