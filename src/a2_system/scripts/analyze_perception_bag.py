#!/usr/bin/env python3
"""Offline bag analysis for the JT128 perception/traversability pipeline.

Per-topic independent aggregation: each topic (input/obstacle/ground/
traversability/status) produces its own frame list. No cross-topic
synchronisation is assumed.

All STOP/self/forward statistics are computed after transforming the
obstacle pointcloud into ``base_link`` via the bag's TF tree.

Outputs:
  runtime/bag_validation/reports/<bag>_summary.json
  runtime/bag_validation/reports/<bag>_report.md
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Any, Sequence

import numpy as np

try:
    import rosbag2_py
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
except ImportError:
    rosbag2_py = None

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped


# ── polygons ────────────────────────────────────────────────────────
STOP_POLY_X = (-0.3, 0.5)
STOP_POLY_Y = (-0.4, 0.4)
STOP_POLY_Z = (0.05, 0.85)

SELF_BOX_X = (-0.45, 0.45)
SELF_BOX_Y = (-0.35, 0.35)
SELF_BOX_Z = (-0.20, 0.45)

FORWARD_BINS = (0.5, 1.0, 1.5, 2.0)
FW_Y_MIN, FW_Y_MAX = -0.5, 0.5
FW_Z_MIN, FW_Z_MAX = 0.05, 1.2

SCENES = ("empty_front_clear", "box_front_1m", "low_obstacle_front",
          "side_obstacle_or_wall", "unknown")


# ── per-topic frame types ───────────────────────────────────────────

@dataclass
class InputFrame:
    frame_index: int
    timestamp_ns: int
    frame_id: str = ""
    point_count: int = 0
    near_zero_005: int = 0
    near_zero_015: int = 0


@dataclass
class ObstacleFrame:
    frame_index: int
    timestamp_ns: int
    frame_id: str = ""
    point_count: int = 0
    stop_points: int = 0
    self_box_points: int = 0
    forward: dict[float, int] = field(default_factory=dict)
    z_min: float | None = None
    z_max: float | None = None
    z_mean: float | None = None
    tf_missing: bool = False
    tf_future_fallback: bool = False


@dataclass
class GroundFrame:
    frame_index: int
    timestamp_ns: int
    frame_id: str = ""
    point_count: int = 0
    z_min: float | None = None
    z_max: float | None = None
    z_mean: float | None = None


@dataclass
class TravFrame:
    frame_index: int
    timestamp_ns: int
    frame_id: str = ""
    known_cells: int = 0
    unknown_cells: int = 0
    lethal_cells: int = 0
    max_cost: int = 0
    mean_cost: float = 0.0


@dataclass
class StatusFrame:
    frame_index: int
    timestamp_ns: int
    dropped_min_range: int = 0
    dropped_self_filter: int = 0
    filtered_points: int = 0
    state: str = ""
    ready: str = ""
    has_self_filter_field: bool = False


# ── TF lookup status ────────────────────────────────────────────────

class TfStatus(Enum):
    EXACT = "exact_or_past"
    FUTURE = "future_fallback"
    MISSING = "missing"


# ── TF math ─────────────────────────────────────────────────────────

def quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[0, 0] = 1 - 2*qy*qy - 2*qz*qz
    T[0, 1] = 2*qx*qy - 2*qz*qw
    T[0, 2] = 2*qx*qz + 2*qy*qw
    T[1, 0] = 2*qx*qy + 2*qz*qw
    T[1, 1] = 1 - 2*qx*qx - 2*qz*qz
    T[1, 2] = 2*qy*qz - 2*qx*qw
    T[2, 0] = 2*qx*qz - 2*qy*qw
    T[2, 1] = 2*qy*qz + 2*qx*qw
    T[2, 2] = 1 - 2*qx*qx - 2*qy*qy
    return T


def transform_to_matrix(tf: TransformStamped) -> np.ndarray:
    T = quat_to_matrix(tf.transform.rotation.x, tf.transform.rotation.y,
                       tf.transform.rotation.z, tf.transform.rotation.w)
    T[0, 3] = tf.transform.translation.x
    T[1, 3] = tf.transform.translation.y
    T[2, 3] = tf.transform.translation.z
    return T


def invert_matrix(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    Tinv = np.eye(4, dtype=np.float64)
    RT = R.T
    Tinv[:3, :3] = RT
    Tinv[:3, 3] = -RT @ t
    return Tinv


def apply_transform(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    N = points.shape[0]
    pts_h = np.column_stack([points, np.ones(N, dtype=np.float64)])
    return (T @ pts_h.T).T[:, :3]


class TfBuffer:
    """Minimal TF buffer supporting static/dynamic transforms and multi-hop BFS."""

    def __init__(self) -> None:
        self._static: dict[tuple[str, str], TransformStamped] = {}
        self._dynamic: dict[tuple[str, str], list[tuple[int, TransformStamped]]] = {}

    def add_static(self, tf: TransformStamped) -> None:
        self._static[(tf.header.frame_id, tf.child_frame_id)] = tf

    def add_dynamic(self, tf: TransformStamped, stamp_ns: int) -> None:
        key = (tf.header.frame_id, tf.child_frame_id)
        self._dynamic.setdefault(key, []).append((stamp_ns, tf))

    def lookup(self, target: str, source: str, stamp_ns: int) -> tuple[np.ndarray | None, TfStatus]:
        """Return (matrix, status). Matrix maps source→target."""
        if target == source:
            return np.eye(4, dtype=np.float64), TfStatus.EXACT

        # BFS graph
        adj: dict[str, list[tuple[str, tuple[str, str], str]]] = {}
        for (p, c) in set(list(self._static.keys()) + list(self._dynamic.keys())):
            adj.setdefault(p, []).append((c, (p, c), "fwd"))
            adj.setdefault(c, []).append((p, (p, c), "rev"))

        visited: dict[str, tuple[np.ndarray, TfStatus] | None] = {
            source: (np.eye(4, dtype=np.float64), TfStatus.EXACT)
        }
        queue: deque[str] = deque([source])
        worst_status = TfStatus.EXACT

        while queue:
            node = queue.popleft()
            entry = visited.get(node)
            if entry is None:
                continue
            T_cum, cum_status = entry

            for neighbor, edge_key, direction in adj.get(node, []):
                if neighbor in visited:
                    continue
                tf_stamped, edge_status = self._resolve_edge(edge_key, stamp_ns)
                if tf_stamped is None:
                    continue

                T_edge = transform_to_matrix(tf_stamped)
                if direction == "fwd":
                    T_edge = invert_matrix(T_edge)

                T_new = T_edge @ T_cum
                new_status = cum_status if cum_status != TfStatus.EXACT else edge_status
                visited[neighbor] = (T_new, new_status)
                if neighbor == target:
                    return T_new, new_status
                queue.append(neighbor)

        return None, TfStatus.MISSING

    def _resolve_edge(self, key: tuple[str, str], stamp_ns: int
                      ) -> tuple[TransformStamped | None, TfStatus]:
        tf_s = self._static.get(key)
        if tf_s is not None:
            return tf_s, TfStatus.EXACT
        entries = self._dynamic.get(key)
        if not entries:
            return None, TfStatus.MISSING
        best = None
        for ts, tf in entries:
            if ts <= stamp_ns and (best is None or ts > best[0]):
                best = (ts, tf)
        if best is not None:
            return best[1], TfStatus.EXACT
        # future fallback
        best_after = min(entries, key=lambda x: x[0])
        return best_after[1], TfStatus.FUTURE

    @property
    def static_edges(self) -> list[str]:
        return [f"{p}→{c}" for (p, c) in self._static]

    @property
    def dynamic_edges(self) -> list[str]:
        return [f"{p}→{c}" for (p, c) in self._dynamic]


# ── helpers ─────────────────────────────────────────────────────────

def _infer_scene(bag_path: Path) -> str:
    name = bag_path.name.lower()
    for scene in SCENES:
        if scene != "unknown" and scene in name:
            return scene
    return "unknown"


def _detect_storage_id(bag_uri: str) -> str:
    import yaml as _yaml
    meta_path = Path(bag_uri) / "metadata.yaml"
    if meta_path.exists():
        try:
            meta = _yaml.safe_load(meta_path.read_text(encoding="utf-8")) or {}
            info = meta.get("rosbag2_bagfile_information", {})
            sid = info.get("storage_identifier", "")
            if sid in ("sqlite3", "mcap"):
                return sid
        except Exception:
            pass
    if list(Path(bag_uri).glob("*.db3")):
        return "sqlite3"
    return "sqlite3"


def _stamp_to_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _iter_xyz(msg: PointCloud2):
    for pt in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        yield pt


def _parse_status(msg: String) -> dict[str, str]:
    result: dict[str, str] = {}
    for part in msg.data.split(";"):
        part = part.strip()
        if not part or "=" not in part:
            continue
        key, _, value = part.partition("=")
        result[key.strip()] = value.strip()
    return result


def _compute_percentiles(values: Sequence[float]) -> dict[str, float]:
    if not values:
        return {"p50": 0, "p95": 0, "p99": 0, "max": 0}
    arr = np.array(sorted(values), dtype=np.float64)
    return {
        "p50": float(np.percentile(arr, 50)),
        "p95": float(np.percentile(arr, 95)),
        "p99": float(np.percentile(arr, 99)),
        "max": float(arr[-1]),
    }


def _safe_min_or_none(values: list[float | None]) -> float | None:
    vals = [v for v in values if v is not None]
    return float(np.min(vals)) if vals else None


def _safe_max_or_none(values: list[float | None]) -> float | None:
    vals = [v for v in values if v is not None]
    return float(np.max(vals)) if vals else None


def _safe_mean_or_none(values: list[float | None]) -> float | None:
    vals = [v for v in values if v is not None]
    return float(np.mean(vals)) if vals else None


def _fmt_or_na(value: float | None, fmt: str = ".2f") -> str:
    if value is None:
        return "N/A"
    return f"{value:{fmt}}"

def analyze_bag(
    bag_path: Path,
    output_dir: Path,
    *,
    scene: str = "unknown",
    mode: str = "recorded",
    stop_poly_x: tuple[float, float] = STOP_POLY_X,
    stop_poly_y: tuple[float, float] = STOP_POLY_Y,
    stop_poly_z: tuple[float, float] = STOP_POLY_Z,
    self_box_x: tuple[float, float] = SELF_BOX_X,
    self_box_y: tuple[float, float] = SELF_BOX_Y,
    self_box_z: tuple[float, float] = SELF_BOX_Z,
) -> dict[str, Any]:
    if rosbag2_py is None:
        raise ImportError("rosbag2_py not available")

    bag_uri = str(bag_path)
    if bag_uri.endswith(".db3"):
        bag_uri = str(bag_path.parent)
    storage_id = _detect_storage_id(bag_uri)

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_uri, storage_id=storage_id),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    type_map: dict[str, str] = {tm.name: tm.type for tm in reader.get_all_topics_and_types()}
    _TFMessage = None
    tf_msg_type = type_map.get("/tf", type_map.get("/tf_static", ""))
    if "TFMessage" in tf_msg_type:
        from tf2_msgs.msg import TFMessage as _TFMsg
        _TFMessage = _TFMsg

    tf_buffer = TfBuffer()
    TARGET_FRAME = "base_link"

    # ── per-topic independent frame lists ────────────────────────────
    input_frames: list[InputFrame] = []
    obstacle_frames: list[ObstacleFrame] = []
    ground_frames: list[GroundFrame] = []
    trav_frames: list[TravFrame] = []
    status_frames: list[StatusFrame] = []

    obstacle_source_frame: str | None = None
    tf_fallback_count = 0       # header.stamp == 0
    tf_future_count = 0         # TF lookup used future fallback
    tf_missing_count = 0
    tf_warnings: list[str] = []

    idx = 0
    while reader.has_next():
        topic_name, msg_bytes, stamp_ns = reader.read_next()
        idx += 1

        # ── TF ──────────────────────────────────────────────────────
        if topic_name == "/tf_static":
            for tf in _deserialize_tf(msg_bytes, _TFMessage):
                tf_buffer.add_static(tf)
            continue
        if topic_name == "/tf":
            for tf in _deserialize_tf(msg_bytes, _TFMessage):
                tf_buffer.add_dynamic(tf, stamp_ns)
            continue

        # ── input ───────────────────────────────────────────────────
        if topic_name == "/jt128/front/points":
            msg = deserialize_message(msg_bytes, PointCloud2)
            pts = np.array(list(_iter_xyz(msg)), dtype=np.float32)
            f = InputFrame(frame_index=idx, timestamp_ns=stamp_ns,
                           frame_id=msg.header.frame_id,
                           point_count=msg.width if msg.height <= 1 else msg.width * msg.height)
            if pts.size:
                ranges = np.linalg.norm(pts, axis=1)
                f.near_zero_005 = int(np.sum(ranges < 0.05))
                f.near_zero_015 = int(np.sum(ranges < 0.15))
            input_frames.append(f)
            continue

        # ── obstacle ────────────────────────────────────────────────
        if topic_name == "/a2/obstacle/points":
            msg = deserialize_message(msg_bytes, PointCloud2)
            if obstacle_source_frame is None:
                obstacle_source_frame = msg.header.frame_id

            pts = np.array(list(_iter_xyz(msg)), dtype=np.float32)
            f = ObstacleFrame(frame_index=idx, timestamp_ns=stamp_ns,
                              frame_id=msg.header.frame_id,
                              point_count=msg.width if msg.height <= 1 else msg.width * msg.height)
            for fwd in FORWARD_BINS:
                f.forward[fwd] = 0
            f.forward[1.55] = 0  # sentinel for 0.5-1.5m

            if pts.size == 0:
                obstacle_frames.append(f)
                continue

            # TF query
            query_ns = stamp_ns
            header_ns = _stamp_to_ns(msg.header.stamp) if (hasattr(msg, 'header')
                          and msg.header.stamp.sec > 0) else 0
            if header_ns > 0:
                query_ns = header_ns
            else:
                tf_fallback_count += 1

            if msg.header.frame_id == TARGET_FRAME:
                pts_base = pts.astype(np.float64)
            else:
                T_matrix, tf_status = tf_buffer.lookup(TARGET_FRAME, msg.header.frame_id, query_ns)
                if T_matrix is not None:
                    pts_base = apply_transform(pts.astype(np.float64), T_matrix)
                    if tf_status == TfStatus.FUTURE:
                        f.tf_future_fallback = True
                        tf_future_count += 1
                        tf_warnings.append(
                            f"Frame {idx}: TF {msg.header.frame_id}→{TARGET_FRAME} "
                            f"used future fallback at query_ns={query_ns}"
                        )
                else:
                    f.tf_missing = True
                    tf_missing_count += 1
                    pts_base = pts.astype(np.float64)
                    tf_warnings.append(
                        f"Frame {idx}: TF {msg.header.frame_id}→{TARGET_FRAME} "
                        f"missing at query_ns={query_ns}"
                    )

            # stats in base_link
            in_stop = (
                (pts_base[:, 0] >= stop_poly_x[0]) & (pts_base[:, 0] <= stop_poly_x[1])
                & (pts_base[:, 1] >= stop_poly_y[0]) & (pts_base[:, 1] <= stop_poly_y[1])
                & (pts_base[:, 2] >= stop_poly_z[0]) & (pts_base[:, 2] <= stop_poly_z[1])
            )
            f.stop_points = int(np.sum(in_stop))

            in_self = (
                (pts_base[:, 0] >= self_box_x[0]) & (pts_base[:, 0] <= self_box_x[1])
                & (pts_base[:, 1] >= self_box_y[0]) & (pts_base[:, 1] <= self_box_y[1])
                & (pts_base[:, 2] >= self_box_z[0]) & (pts_base[:, 2] <= self_box_z[1])
            )
            f.self_box_points = int(np.sum(in_self))

            in_fwd_yz = (
                (pts_base[:, 1] >= FW_Y_MIN) & (pts_base[:, 1] <= FW_Y_MAX)
                & (pts_base[:, 2] >= FW_Z_MIN) & (pts_base[:, 2] <= FW_Z_MAX)
            )
            for fwd in FORWARD_BINS:
                in_fwd = in_fwd_yz & (pts_base[:, 0] >= 0) & (pts_base[:, 0] <= fwd)
                f.forward[fwd] = int(np.sum(in_fwd))
            in_zone_05_15 = (
                in_fwd_yz & (pts_base[:, 0] >= 0.5) & (pts_base[:, 0] <= 1.5)
            )
            f.forward[1.55] = int(np.sum(in_zone_05_15))

            if len(pts_base):
                f.z_min = float(np.min(pts_base[:, 2]))
                f.z_max = float(np.max(pts_base[:, 2]))
                f.z_mean = float(np.mean(pts_base[:, 2]))

            obstacle_frames.append(f)
            continue

        # ── ground ──────────────────────────────────────────────────
        if topic_name == "/a2/ground/points":
            msg = deserialize_message(msg_bytes, PointCloud2)
            pts = np.array(list(_iter_xyz(msg)), dtype=np.float32)
            f = GroundFrame(frame_index=idx, timestamp_ns=stamp_ns,
                            frame_id=msg.header.frame_id,
                            point_count=msg.width if msg.height <= 1 else msg.width * msg.height)
            if len(pts):
                f.z_min = float(np.min(pts[:, 2]))
                f.z_max = float(np.max(pts[:, 2]))
                f.z_mean = float(np.mean(pts[:, 2]))
            ground_frames.append(f)
            continue

        # ── traversability ──────────────────────────────────────────
        if topic_name == "/a2/traversability":
            msg = deserialize_message(msg_bytes, OccupancyGrid)
            data = np.array(msg.data, dtype=np.int8)
            f = TravFrame(frame_index=idx, timestamp_ns=stamp_ns,
                          frame_id=msg.header.frame_id)
            if data.size:
                f.known_cells = int(np.sum(data >= 0))
                f.unknown_cells = int(np.sum(data == -1))
                f.lethal_cells = int(np.sum(data >= 90))
                known = data[data >= 0]
                f.max_cost = int(np.max(known)) if known.size else 0
                f.mean_cost = float(np.mean(known)) if known.size else 0.0
            trav_frames.append(f)
            continue

        # ── status: count EVERY message ─────────────────────────────
        if topic_name == "/a2/perception/ground_segmentation/status":
            msg = deserialize_message(msg_bytes, String)
            parsed = _parse_status(msg)
            has_sf = "dropped_self_filter" in parsed
            f = StatusFrame(
                frame_index=idx, timestamp_ns=stamp_ns,
                dropped_min_range=int(parsed.get("dropped_min_range", 0)),
                dropped_self_filter=int(parsed.get("dropped_self_filter", 0)),
                filtered_points=int(parsed.get("filtered_points", 0)),
                state=parsed.get("state", ""),
                ready=parsed.get("ready", ""),
                has_self_filter_field=has_sf,
            )
            status_frames.append(f)
            continue

    reader.close()

    # ── aggregate report ────────────────────────────────────────────

    n_obstacle = len(obstacle_frames)
    tf_missing_ratio = tf_missing_count / max(n_obstacle, 1) * 100.0
    tf_future_ratio = tf_future_count / max(n_obstacle, 1) * 100.0

    report: dict[str, Any] = {
        "bag_path": str(bag_path),
        "analyzed_at": datetime.now().isoformat(),
        "scene": scene,
        "mode": mode,
        "total_messages": idx,
        "input_frames": len(input_frames),
        "obstacle_frames": n_obstacle,
        "ground_frames": len(ground_frames),
        "traversability_frames": len(trav_frames),
        "status_frames": len(status_frames),
        "tf": {
            "target_frame": TARGET_FRAME,
            "obstacle_source_frame": obstacle_source_frame,
            "static_edges": tf_buffer.static_edges,
            "dynamic_edges": tf_buffer.dynamic_edges,
            "tf_missing_frames": tf_missing_count,
            "tf_missing_ratio_pct": round(tf_missing_ratio, 1),
            "tf_fallback_frames": tf_fallback_count,
            "tf_future_fallback_frames": tf_future_count,
            "tf_future_fallback_ratio_pct": round(tf_future_ratio, 1),
        },
    }

    if input_frames:
        pts_list = [f.point_count for f in input_frames]
        n005 = [f.near_zero_005 for f in input_frames]
        n015 = [f.near_zero_015 for f in input_frames]
        report["input"] = {
            "points_per_frame": _compute_percentiles(pts_list),
            "near_zero_005_per_frame": _compute_percentiles(n005),
            "near_zero_015_per_frame": _compute_percentiles(n015),
            "near_zero_005_ratio_pct": round(np.mean(n005) / max(np.mean(pts_list), 1) * 100, 1),
            "near_zero_015_ratio_pct": round(np.mean(n015) / max(np.mean(pts_list), 1) * 100, 1),
            "frame_ids": sorted({f.frame_id for f in input_frames if f.frame_id}),
        }

    if obstacle_frames:
        obs_pts_list = [f.point_count for f in obstacle_frames]
        stop_counts = [f.stop_points for f in obstacle_frames]
        self_counts = [f.self_box_points for f in obstacle_frames]
        obs_with_pts = [f for f in obstacle_frames if f.point_count > 0]
        all_zs = [f.z_min for f in obstacle_frames] + [f.z_max for f in obstacle_frames] + [f.z_mean for f in obstacle_frames]
        any_z = any(v is not None for v in all_zs)

        report["obstacle"] = {
            "frames_total": n_obstacle,
            "frames_with_points": len(obs_with_pts),
            "points_per_frame": _compute_percentiles(obs_pts_list),
            "z_min": _safe_min_or_none([f.z_min for f in obstacle_frames]),
            "z_max": _safe_max_or_none([f.z_max for f in obstacle_frames]),
            "z_mean": _safe_mean_or_none([f.z_mean for f in obstacle_frames]),
            "frame_ids": sorted({f.frame_id for f in obstacle_frames if f.frame_id}),
            "stop_polygon_frame": TARGET_FRAME,
            "self_box_frame": TARGET_FRAME,
            "forward_bins_frame": TARGET_FRAME,
            "stop_polygon": {
                "definition": {"x": list(stop_poly_x), "y": list(stop_poly_y),
                               "z": list(stop_poly_z), "frame": TARGET_FRAME},
                "points_per_frame": _compute_percentiles(stop_counts),
                "frames_with_stop_points": int(sum(1 for c in stop_counts if c > 0)),
                "frames_zero_stop": int(sum(1 for c in stop_counts if c == 0)),
            },
            "self_box": {
                "definition": {"x": list(self_box_x), "y": list(self_box_y),
                               "z": list(self_box_z), "frame": TARGET_FRAME},
                "points_per_frame": _compute_percentiles(self_counts),
            },
        }

        fwd_summary: dict[str, Any] = {}
        for fwd in FORWARD_BINS:
            counts = [f.forward.get(fwd, 0) for f in obstacle_frames]
            fwd_summary[f"0-{fwd}m"] = _compute_percentiles(counts)
        zone_counts = [f.forward.get(1.55, 0) for f in obstacle_frames]
        fwd_summary["0.5-1.5m"] = _compute_percentiles(zone_counts)
        report["obstacle"]["forward_bins"] = fwd_summary

    if ground_frames:
        g_pts = [f.point_count for f in ground_frames]
        gnd_with_pts = [f for f in ground_frames if f.point_count > 0]
        report["ground"] = {
            "frames_total": len(ground_frames),
            "frames_with_points": len(gnd_with_pts),
            "points_per_frame": _compute_percentiles(g_pts),
            "z_min": _safe_min_or_none([f.z_min for f in ground_frames]),
            "z_max": _safe_max_or_none([f.z_max for f in ground_frames]),
            "z_mean_mean": _safe_mean_or_none([f.z_mean for f in ground_frames]),
            "frame_ids": sorted({f.frame_id for f in ground_frames if f.frame_id}),
        }

    if trav_frames:
        report["traversability"] = {
            "frames": len(trav_frames),
            "known_cells_per_frame": _compute_percentiles([f.known_cells for f in trav_frames]),
            "unknown_cells_per_frame": _compute_percentiles([f.unknown_cells for f in trav_frames]),
            "lethal_cells_per_frame": _compute_percentiles([f.lethal_cells for f in trav_frames]),
            "max_cost_per_frame": _compute_percentiles([float(f.max_cost) for f in trav_frames]),
            "mean_cost_per_frame": _compute_percentiles([f.mean_cost for f in trav_frames]),
            "mean_cost_overall": float(np.mean([f.mean_cost for f in trav_frames])),
            "frame_ids": sorted({f.frame_id for f in trav_frames if f.frame_id}),
        }

    # Status: always include if ANY status messages seen
    if status_frames:
        has_any_sf = any(f.has_self_filter_field for f in status_frames)
        report["status"] = {
            "frames": len(status_frames),
            "has_self_filter_field": has_any_sf,
            "dropped_min_range": _compute_percentiles([f.dropped_min_range for f in status_frames]),
            "dropped_self_filter": _compute_percentiles([f.dropped_self_filter for f in status_frames]),
            "filtered_points": _compute_percentiles([f.filtered_points for f in status_frames]),
            "states_seen": sorted({f.state for f in status_frames if f.state}),
        }

    # ── write outputs ───────────────────────────────────────────────
    output_dir.mkdir(parents=True, exist_ok=True)
    bag_name = bag_path.name
    json_path = output_dir / f"{bag_name}_summary.json"
    json_path.write_text(json.dumps(report, indent=2, default=str), encoding="utf-8")
    md_path = output_dir / f"{bag_name}_report.md"
    _write_markdown_report(report, bag_name, tf_warnings, md_path)
    print(f"Wrote {json_path}")
    print(f"Wrote {md_path}")
    _print_summary(report)
    return report


def _deserialize_tf(msg_bytes: bytes, _TFMessage=None) -> list[TransformStamped]:
    if _TFMessage is not None:
        msg = deserialize_message(msg_bytes, _TFMessage)
        return list(msg.transforms)
    try:
        return [deserialize_message(msg_bytes, TransformStamped)]
    except Exception:
        return []


# ── markdown report ─────────────────────────────────────────────────

def _write_markdown_report(report: dict[str, Any], bag_name: str,
                           tf_warnings: list[str], path: Path) -> None:
    lines: list[str] = []

    def a(*args: str) -> None:
        lines.append(" ".join(args))

    def h(level: int, text: str) -> None:
        lines.append(f"{'#' * level} {text}")

    def tbl(headers: list[str], rows: list[list[str]]) -> None:
        lines.append("| " + " | ".join(headers) + " |")
        lines.append("|" + "|".join(["------"] * len(headers)) + "|")
        for row in rows:
            lines.append("| " + " | ".join(str(c) for c in row) + " |")

    mode_note = (
        "This validates **recorded** outputs already present in the bag. "
        "It does NOT prove current source regeneration unless the bag was produced "
        "by the current source pipeline (raw-only bag → replay → current algorithm → "
        "record processed bag)."
        if report.get("mode") == "recorded"
        else "This validates **regenerated** outputs from the current source, "
        "assuming the bag was produced by the raw-only replay pipeline "
        "(record raw bag → replay with current algorithm → record processed bag)."
    )

    h(1, f"Perception Bag Analysis: {bag_name}")
    a(f"Analyzed: {report['analyzed_at']}")
    a(f"Bag: {report['bag_path']}")
    a(f"Scene: {report.get('scene', 'unknown')}  |  Mode: {report.get('mode', 'recorded')}")
    a(f"> {mode_note}")
    a(f"Frames: total_msgs={report['total_messages']}, input={report['input_frames']}, "
      f"obstacle={report['obstacle_frames']}, ground={report.get('ground_frames', 0)}, "
      f"traversability={report.get('traversability_frames', 0)}, "
      f"status={report['status_frames']}")
    a()
    a("**Per-topic independent aggregation** — each topic is analysed from its own frame list. "
      "No cross-topic synchronisation is assumed.")
    a()

    # TF
    if "tf" in report:
        tf = report["tf"]
        h(2, "TF")
        a(f"Target frame: **{tf['target_frame']}**")
        a(f"Obstacle source frame: {tf.get('obstacle_source_frame', 'unknown')}")
        a(f"Static edges: {', '.join(tf['static_edges']) if tf['static_edges'] else 'none'}")
        a(f"Dynamic edges: {', '.join(tf['dynamic_edges']) if tf['dynamic_edges'] else 'none'}")
        a(f"TF missing: {tf['tf_missing_frames']} ({tf['tf_missing_ratio_pct']}%)")
        a(f"TF header.stamp==0 fallback: {tf.get('tf_fallback_frames', 0)}")
        a(f"TF future fallback: {tf.get('tf_future_fallback_frames', 0)} "
          f"({tf.get('tf_future_fallback_ratio_pct', 0)}%)")
        a()
        a("STOP/self/forward statistics are computed **after transforming obstacle cloud into base_link**.")
        if tf['tf_missing_frames'] > 0:
            a("⚠ TF missing for some frames — STOP/self/forward on those frames use **untransformed** coordinates (INVALID).")
        if tf.get('tf_future_fallback_frames', 0) > 0:
            a("⚠ TF future fallback used — STOP/self/forward stats on those frames may be less accurate.")
        a()
        if tf_warnings:
            h(3, "TF Warnings")
            for w in tf_warnings[:10]:
                a(f"- {w}")
            if len(tf_warnings) > 10:
                a(f"- ... and {len(tf_warnings) - 10} more")
            a()

    # Input
    if "input" in report:
        inp = report["input"]
        h(2, "Input Cloud /jt128/front/points")
        a(f"Frame IDs: {', '.join(inp['frame_ids']) if inp['frame_ids'] else 'none'}")
        tbl(["metric", "p50", "p95", "p99", "max"],
            [["points/frame", *[str(inp["points_per_frame"].get(k, "-")) for k in ("p50", "p95", "p99", "max")]],
             ["near_zero (<0.05m)/frame", *[str(inp["near_zero_005_per_frame"].get(k, "-")) for k in ("p50", "p95", "p99", "max")]],
             ["near_zero (<0.15m)/frame", *[str(inp["near_zero_015_per_frame"].get(k, "-")) for k in ("p50", "p95", "p99", "max")]]])
        a(f"near_zero_005 ratio: {inp['near_zero_005_ratio_pct']}%")
        a()

    # Obstacle
    if "obstacle" in report:
        obs = report["obstacle"]
        h(2, "Obstacle Cloud /a2/obstacle/points")
        a(f"Frames: total={obs.get('frames_total', obs.get('frames_with_points', '?'))}, "
          f"with_points={obs.get('frames_with_points', '?')}")
        a(f"Frame IDs: {', '.join(obs.get('frame_ids', [])) if obs.get('frame_ids') else 'none'}")
        a(f"STOP/self/forward computed in: **{obs.get('stop_polygon_frame', 'base_link')}**")
        tbl(["metric", "p50", "p95", "p99", "max"],
            [["points/frame", *[str(obs["points_per_frame"].get(k, "-")) for k in ("p50", "p95", "p99", "max")]],
             ["z_range", f"{_fmt_or_na(obs['z_min'])}–{_fmt_or_na(obs['z_max'])}",
              f"mean={_fmt_or_na(obs['z_mean'])}", "", ""]])
        a()

        h(3, "STOP Polygon")
        sp = obs["stop_polygon"]
        a(f"Definition: x={sp['definition']['x']}, y={sp['definition']['y']}, "
          f"z={sp['definition']['z']} in **{sp['definition']['frame']}**")
        tbl(["metric", "p50", "p95", "p99", "max"],
            [["STOP points/frame", *[str(sp["points_per_frame"].get(k, "-")) for k in ("p50", "p95", "p99", "max")]]])
        n_obs_total = obs.get("frames_total", obs.get("frames_with_points", "?"))
        a(f"Frames with STOP points: {sp['frames_with_stop_points']} / {n_obs_total}")
        a(f"Frames with zero STOP points: {sp['frames_zero_stop']}")
        a()

        h(3, "Forward Obstacle Points (|y|≤0.5m, z∈[0.05,1.2m])")
        fwd_rows = []
        for label in ["0-0.5m", "0-1.0m", "0.5-1.5m", "0-1.5m", "0-2.0m"]:
            d = obs["forward_bins"].get(label, {})
            if d:
                fwd_rows.append([label, *[str(d.get(k, "-")) for k in ("p50", "p95", "p99", "max")]])
        tbl(["metric", "p50", "p95", "p99", "max"], fwd_rows)
        a()

        h(3, "Self Box")
        sb = obs["self_box"]
        a(f"Definition: x={sb['definition']['x']}, y={sb['definition']['y']}, "
          f"z={sb['definition']['z']} in **{sb['definition']['frame']}**")
        tbl(["metric", "p50", "p95", "p99", "max"],
            [["self_box points/frame", *[str(sb["points_per_frame"].get(k, "-")) for k in ("p50", "p95", "p99", "max")]]])
        a()

    # Ground
    if "ground" in report:
        gnd = report["ground"]
        h(2, "Ground Cloud /a2/ground/points")
        a(f"Frames: total={gnd.get('frames_total', gnd.get('frames', '?'))}, "
          f"with_points={gnd.get('frames_with_points', '?')}")
        if gnd.get("frame_ids"):
            a(f"Frame IDs: {', '.join(gnd['frame_ids'])}")
        tbl(["metric", "value"],
            [["frames", str(gnd.get("frames_total", gnd.get("frames", "?")))],
             ["points/frame (p50)", str(gnd["points_per_frame"].get("p50", "-"))],
             ["z min", _fmt_or_na(gnd.get("z_min"), ".3f")],
             ["z max", _fmt_or_na(gnd.get("z_max"), ".3f")],
             ["z mean (avg)", _fmt_or_na(gnd.get("z_mean_mean"), ".3f")]])
        a()

    # Traversability
    if "traversability" in report:
        tr = report["traversability"]
        h(2, "Traversability /a2/traversability")
        if tr.get("frame_ids"):
            a(f"Frame IDs: {', '.join(tr['frame_ids'])}")
        tbl(["metric", "p50", "p95", "max"],
            [["known cells/frame", *[str(tr["known_cells_per_frame"].get(k, "-")) for k in ("p50", "p95", "max")]],
             ["unknown cells/frame", *[str(tr["unknown_cells_per_frame"].get(k, "-")) for k in ("p50", "p95", "max")]],
             ["lethal cells/frame", *[str(tr["lethal_cells_per_frame"].get(k, "-")) for k in ("p50", "p95", "max")]],
             ["max cost/frame", *[str(tr["max_cost_per_frame"].get(k, "-")) for k in ("p50", "p95", "max")]]])
        a(f"Mean cost overall: {tr['mean_cost_overall']:.1f}")
        a()

    # Status
    if "status" in report:
        st = report["status"]
        h(2, "Status /a2/perception/ground_segmentation/status")
        a(f"Has self_filter field: {st.get('has_self_filter_field', False)}")
        dr = st.get("dropped_min_range", {})
        ds = st.get("dropped_self_filter", {})
        fp = st.get("filtered_points", {})
        tbl(["metric", "p50", "p95", "max"],
            [["dropped_min_range/frame", *[str(dr.get(k, "-")) for k in ("p50", "p95", "max")]],
             ["dropped_self_filter/frame", *[str(ds.get(k, "-")) for k in ("p50", "p95", "max")]],
             ["filtered_points/frame", *[str(fp.get(k, "-")) for k in ("p50", "p95", "max")]]])
        if st.get("states_seen"):
            a(f"States seen: {', '.join(st['states_seen'])}")
        a()

    h(2, "Pass/Fail Checklist")
    _write_pass_fail(report, lines, a)
    path.write_text("\n".join(lines), encoding="utf-8")


# ── pass/fail ───────────────────────────────────────────────────────

def _write_pass_fail(report: dict[str, Any], lines: list[str], a) -> None:
    scene = report.get("scene", "unknown")
    if scene == "unknown":
        a("⚪ Scene is 'unknown' — no pass/fail checks applied. Metrics only.")
        return

    checks: list[tuple[str, bool, str]] = []
    tf_info = report.get("tf", {})

    # TF health
    tf_miss = tf_info.get("tf_missing_ratio_pct", 100)
    tf_future = tf_info.get("tf_future_fallback_ratio_pct", 100)
    checks.append(("TF missing ratio == 0", tf_miss == 0, f"{tf_miss:.1f}%"))

    # TF future fallback: fail if obstacle frame != base_link and future fallback used
    obs_src = tf_info.get("obstacle_source_frame")
    if obs_src and obs_src != "base_link":
        checks.append(("TF future fallback ratio == 0", tf_future == 0, f"{tf_future:.1f}%"))

    # ── status topic / self-filter ──────────────────────────────────
    if "status" not in report:
        checks.append(("Status topic present", False, "no /a2/perception/ground_segmentation/status messages"))
    else:
        st = report["status"]
        if not st.get("has_self_filter_field"):
            checks.append(("Self-filter field present", False, "dropped_self_filter field missing from status"))
        else:
            dropped_sf = st.get("dropped_self_filter", {})
            sf_active = dropped_sf.get("p50", 0) > 0 or dropped_sf.get("max", 0) > 0
            checks.append((
                "Self-filter active (dropped_self_filter p50>0 or max>0)",
                sf_active,
                f"p50={dropped_sf.get('p50', -1):.0f} max={dropped_sf.get('max', -1):.0f}",
            ))

    # ── obstacle topic ──────────────────────────────────────────────
    has_obstacle = "obstacle" in report
    checks.append(("Obstacle topic present", has_obstacle,
                   "has /a2/obstacle/points" if has_obstacle else "no /a2/obstacle/points messages"))
    obs_has_points = has_obstacle and report["obstacle"].get("frames_with_points", 0) > 0

    # ── traverability topic (low_obstacle_front requires it) ────────
    if scene == "low_obstacle_front":
        has_trav = "traversability" in report
        checks.append(("Traversability topic present", has_trav,
                       "has /a2/traversability" if has_trav else "no /a2/traversability messages"))

    # ── scene-specific ──────────────────────────────────────────────
    if scene == "empty_front_clear":
        if has_obstacle:
            sp = report["obstacle"]["stop_polygon"]["points_per_frame"]
            checks += [
                ("STOP polygon p95 ≤ 3", sp.get("p95", 999) <= 3, f"p95={sp.get('p95',-1):.0f}"),
                ("STOP polygon max ≤ 10", sp.get("max", 999) <= 10, f"max={sp.get('max',-1):.0f}"),
            ]
            fwd_05 = report["obstacle"]["forward_bins"].get("0-0.5m", {})
            checks.append(("Forward 0-0.5m p95 ≤ 3", fwd_05.get("p95", 999) <= 3, f"p95={fwd_05.get('p95',-1):.0f}"))

    elif scene == "box_front_1m":
        if has_obstacle:
            if not obs_has_points:
                checks.append(("Obstacle cloud has points (box_front_1m expects obstacle)",
                               False, "all obstacle frames have 0 points"))
            z = report["obstacle"]["forward_bins"].get("0.5-1.5m", {})
            has = (z.get("p50", 0) > 0) or (z.get("p95", 0) > 10)
            checks.append(("0.5-1.5m zone has obstacle points", has,
                           f"p50={z.get('p50',-1):.0f} p95={z.get('p95',-1):.0f}"))
            sp = report["obstacle"]["stop_polygon"]["points_per_frame"]
            checks.append(("STOP polygon p95 ≤ 5", sp.get("p95", 999) <= 5,
                           f"p95={sp.get('p95',-1):.0f}"))

    elif scene == "low_obstacle_front":
        if has_obstacle:
            if not obs_has_points:
                checks.append(("Obstacle cloud has points (low_obstacle expects obstacle)",
                               False, "all obstacle frames have 0 points"))
            z = report["obstacle"]["forward_bins"].get("0.5-1.5m", {})
            checks.append(("0.5-1.5m zone obstacle p95 > 0", z.get("p95", 0) > 0,
                           f"p95={z.get('p95',-1):.0f}"))
        if "traversability" in report:
            lp = report["traversability"]["lethal_cells_per_frame"].get("p95", 0)
            cp = report["traversability"]["max_cost_per_frame"].get("p95", 0)
            checks.append(("Traversability lethal or high-cost p95≥70", lp > 0 or cp >= 70,
                           f"lethal_p95={lp:.0f} cost_p95={cp:.0f}"))

    elif scene == "side_obstacle_or_wall":
        if has_obstacle:
            sp = report["obstacle"]["stop_polygon"]["points_per_frame"]
            checks.append(("STOP polygon p95 ≤ 5", sp.get("p95", 999) <= 5,
                           f"p95={sp.get('p95',-1):.0f}"))
            fwd_05 = report["obstacle"]["forward_bins"].get("0-0.5m", {})
            checks.append(("Forward 0-0.5m p95 ≤ 5", fwd_05.get("p95", 999) <= 5,
                           f"p95={fwd_05.get('p95',-1):.0f}"))

    if not tf_info.get("tf_missing_ratio_pct", 0) == 0 and _obs_src_not_base(report):
        checks.append(("Obstacle frame≠base_link + TF missing → INVALID", False, "untransformable coordinates"))

    for label, passed, detail in checks:
        icon = "✅" if passed else "❌"
        lines.append(f"- {icon} {label}: {detail}")
        if not passed:
            a(f"  ⚠ FAIL: {label}")

    a()
    all_pass = all(p for _, p, _ in checks)
    a(f"{'✅ ALL CHECKS PASS' if all_pass else '❌ SOME CHECKS FAIL — review above'}")


def _obs_src_not_base(report: dict[str, Any]) -> bool:
    src = report.get("tf", {}).get("obstacle_source_frame")
    return src is not None and src != "base_link"


def _print_summary(report: dict[str, Any]) -> None:
    print("\n=== BAG ANALYSIS SUMMARY ===")
    print(f"Bag: {report['bag_path']}")
    print(f"Scene: {report.get('scene', 'unknown')}  Mode: {report.get('mode', 'recorded')}")
    print(f"Frames: input={report['input_frames']} obstacle={report['obstacle_frames']} "
          f"status={report['status_frames']}")
    tf = report.get("tf", {})
    print(f"TF: missing={tf.get('tf_missing_ratio_pct','?')}% "
          f"future_fallback={tf.get('tf_future_fallback_ratio_pct','?')}%")
    if "obstacle" in report:
        sp = report["obstacle"]["stop_polygon"]["points_per_frame"]
        print(f"STOP points: p50={sp.get('p50',0):.0f} p95={sp.get('p95',0):.0f} max={sp.get('max',0):.0f}")
    if "status" in report:
        ds = report["status"]["dropped_self_filter"]
        print(f"Self-filter dropped: p50={ds.get('p50',0):.0f} max={ds.get('max',0):.0f}")
    print("=============================\n")


# ── CLI ─────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze a perception rosbag for self-filter and obstacle validation.")
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--scene", choices=SCENES, default=None)
    parser.add_argument("--mode", choices=("recorded", "regenerated"), default="recorded")
    parser.add_argument("--stop-x-min", type=float, default=STOP_POLY_X[0])
    parser.add_argument("--stop-x-max", type=float, default=STOP_POLY_X[1])
    parser.add_argument("--stop-y-min", type=float, default=STOP_POLY_Y[0])
    parser.add_argument("--stop-y-max", type=float, default=STOP_POLY_Y[1])
    args = parser.parse_args()

    scene = args.scene or _infer_scene(args.bag_path)
    ws_root = Path(__file__).resolve().parents[3]
    output_dir = args.output_dir or (ws_root / "runtime" / "bag_validation" / "reports")

    analyze_bag(args.bag_path, output_dir, scene=scene, mode=args.mode,
                stop_poly_x=(args.stop_x_min, args.stop_x_max),
                stop_poly_y=(args.stop_y_min, args.stop_y_max))


if __name__ == "__main__":
    main()
