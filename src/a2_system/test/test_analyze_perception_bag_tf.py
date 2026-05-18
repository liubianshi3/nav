"""Unit tests for analyze_perception_bag — TF math, per-topic types, pass/fail rules."""

import math
import sys
import tempfile
from pathlib import Path

import numpy as np
import pytest

SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from analyze_perception_bag import (
    TfBuffer, TfStatus,
    InputFrame, ObstacleFrame, GroundFrame, TravFrame, StatusFrame,
    _infer_scene, _compute_percentiles, _detect_storage_id, _stamp_to_ns,
    _write_markdown_report, _write_pass_fail,
    _safe_min_or_none, _safe_max_or_none, _safe_mean_or_none,
    _fmt_or_na,
    apply_transform, invert_matrix, quat_to_matrix, transform_to_matrix,
)
from geometry_msgs.msg import Transform, TransformStamped
from std_msgs.msg import Header


def _make_tf(parent: str, child: str, x: float, y: float, z: float,
             roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> TransformStamped:
    tf = TransformStamped()
    tf.header = Header(frame_id=parent)
    tf.child_frame_id = child
    tf.transform = Transform()
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.translation.z = z
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    tf.transform.rotation.x = sr * cp * cy - cr * sp * sy
    tf.transform.rotation.y = cr * sp * cy + sr * cp * sy
    tf.transform.rotation.z = cr * cp * sy - sr * sp * cy
    tf.transform.rotation.w = cr * cp * cy + sr * sp * sy
    return tf


# ── 4x4 transform math ──────────────────────────────────────────────

def test_quat_identity():
    T = quat_to_matrix(0, 0, 0, 1)
    assert np.allclose(T[:3, :3], np.eye(3))

def test_quat_yaw_90():
    sq2 = math.sqrt(2) / 2
    T = quat_to_matrix(0, 0, sq2, sq2)
    assert np.allclose((T @ np.array([1.,0.,0.,1.]))[:2], [0., 1.], atol=1e-10)

def test_transform_to_matrix_t():
    T = transform_to_matrix(_make_tf("a", "b", 1., 2., 3.))
    assert T[0, 3] == 1. and T[1, 3] == 2. and T[2, 3] == 3.

def test_invert_roundtrip():
    T = transform_to_matrix(_make_tf("a", "b", 1., 2., 3., yaw=0.5))
    Ti = invert_matrix(T)
    assert np.allclose(T @ Ti, np.eye(4), atol=1e-10)

def test_apply_translation():
    pts = np.array([[0.,0.,0.],[1.,2.,3.]], dtype=np.float64)
    T = transform_to_matrix(_make_tf("a", "b", 10., 20., 30.))
    assert np.allclose(apply_transform(pts, T)[0], [10.,20.,30.])


# ── TF direction (ROS convention: T maps child→parent) ─────────────

def test_parent_child_forward():
    """T(map, base_link, x=1.0): p_base=[0.5,0,0] → p_map=[1.5,0,0]."""
    buf = TfBuffer()
    buf.add_static(_make_tf("map", "base_link", 1., 0., 0.))
    T, st = buf.lookup("map", "base_link", 0)
    assert T is not None and st == TfStatus.EXACT
    assert np.allclose(apply_transform(np.array([[0.5,0.,0.]]), T)[0], [1.5, 0., 0.], atol=1e-6)

def test_parent_child_inverse():
    """T(map,base_link,x=1.0) inv: p_map=[1.5,0,0] → p_base=[0.5,0,0]."""
    buf = TfBuffer()
    buf.add_static(_make_tf("map", "base_link", 1., 0., 0.))
    T, st = buf.lookup("base_link", "map", 0)
    assert T is not None
    assert np.allclose(apply_transform(np.array([[1.5,0.,0.]]), T)[0], [0.5, 0., 0.], atol=1e-6)

def test_multihop_map_odom_base():
    buf = TfBuffer()
    buf.add_static(_make_tf("map", "odom", 5., 0., 0.))
    buf.add_static(_make_tf("odom", "base_link", 1., 0., 0.))
    T, _ = buf.lookup("base_link", "map", 0)
    assert T is not None
    assert np.allclose(apply_transform(np.array([[0.,0.,0.]]), T)[0], [-6., 0., 0.], atol=1e-6)

def test_multihop_reverse():
    buf = TfBuffer()
    buf.add_static(_make_tf("map", "odom", 5., 0., 0.))
    buf.add_static(_make_tf("odom", "base_link", 1., 0., 0.))
    T, _ = buf.lookup("map", "base_link", 0)
    assert T is not None
    assert np.allclose(apply_transform(np.array([[0.,0.,0.]]), T)[0], [6., 0., 0.], atol=1e-6)

def test_yaw_90():
    buf = TfBuffer()
    buf.add_static(_make_tf("map", "base_link", 0., 0., 0., yaw=math.pi/2))
    T, _ = buf.lookup("map", "base_link", 0)
    assert T is not None
    assert np.allclose(apply_transform(np.array([[1.,0.,0.]]), T)[0], [0., 1., 0.], atol=1e-6)

def test_yaw_90_inverse():
    buf = TfBuffer()
    buf.add_static(_make_tf("map", "base_link", 0., 0., 0., yaw=math.pi/2))
    T, _ = buf.lookup("base_link", "map", 0)
    assert T is not None
    assert np.allclose(apply_transform(np.array([[0.,1.,0.]]), T)[0], [1., 0., 0.], atol=1e-6)

def test_multihop_with_rotation():
    """map→odom(5,0,0) → base_link(yaw=90°). p_base=[1,0,0] → p_map=[5,1,0]."""
    buf = TfBuffer()
    buf.add_static(_make_tf("map", "odom", 5., 0., 0.))
    buf.add_static(_make_tf("odom", "base_link", 0., 0., 0., yaw=math.pi/2))
    T, _ = buf.lookup("map", "base_link", 0)
    assert T is not None
    assert np.allclose(apply_transform(np.array([[1.,0.,0.]]), T)[0], [5., 1., 0.], atol=1e-6)

def test_identity_lookup():
    T, st = TfBuffer().lookup("base_link", "base_link", 0)
    assert T is not None and st == TfStatus.EXACT

def test_missing_returns_none():
    T, st = TfBuffer().lookup("map", "base_link", 0)
    assert T is None and st == TfStatus.MISSING


# ── dynamic TF ──────────────────────────────────────────────────────

def test_dynamic_tf_best_before():
    buf = TfBuffer()
    buf.add_dynamic(_make_tf("odom", "base_link", 0.5, 0., 0.), 1000)
    T, st = buf.lookup("base_link", "odom", 1500)
    assert T is not None and st == TfStatus.EXACT
    assert np.allclose(apply_transform(np.array([[0.,0.,0.]]), T)[0], [-0.5, 0., 0.], atol=1e-6)

def test_dynamic_tf_future_fallback():
    """Only TF is at t=2000, query at t=500 → future fallback."""
    buf = TfBuffer()
    buf.add_dynamic(_make_tf("odom", "base_link", 1., 0., 0.), 2000)
    T, st = buf.lookup("base_link", "odom", 500)
    assert T is not None
    assert st == TfStatus.FUTURE


# ── per-topic types ─────────────────────────────────────────────────

def test_input_frame_fields():
    f = InputFrame(1, 100, frame_id="jt128", point_count=2000, near_zero_005=10)
    assert f.frame_index == 1 and f.point_count == 2000 and f.near_zero_005 == 10

def test_obstacle_frame_does_not_require_input():
    """Obstacle stats can be created without a preceding input frame."""
    f = ObstacleFrame(5, 500, frame_id="map", point_count=100, stop_points=3)
    assert f.frame_index == 5 and f.stop_points == 3

def test_status_frame_counts_zero_metrics():
    """A status message with all zeros is still counted."""
    f = StatusFrame(3, 300, dropped_min_range=0, dropped_self_filter=0)
    assert f.dropped_self_filter == 0 and f.has_self_filter_field is False

def test_status_frame_has_self_filter_field():
    f = StatusFrame(3, 300, dropped_self_filter=5, has_self_filter_field=True)
    assert f.has_self_filter_field is True


# ── markdown report does not crash ──────────────────────────────────

def test_markdown_report_with_obstacle_does_not_crash():
    report = {
        "bag_path": "/tmp/test_bag",
        "analyzed_at": "2026-01-01T00:00:00",
        "scene": "empty_front_clear",
        "mode": "recorded",
        "total_messages": 500,
        "input_frames": 100,
        "obstacle_frames": 100,
        "ground_frames": 100,
        "traversability_frames": 100,
        "status_frames": 100,
        "tf": {
            "target_frame": "base_link",
            "obstacle_source_frame": "map",
            "static_edges": ["map→odom"],
            "dynamic_edges": [],
            "tf_missing_frames": 0,
            "tf_missing_ratio_pct": 0.0,
            "tf_fallback_frames": 0,
            "tf_future_fallback_frames": 0,
            "tf_future_fallback_ratio_pct": 0.0,
        },
        "input": {
            "points_per_frame": {"p50": 20000, "p95": 20000, "max": 20000},
            "near_zero_005_per_frame": {"p50": 0, "p95": 0, "max": 0},
            "near_zero_015_per_frame": {"p50": 0, "p95": 0, "max": 0},
            "near_zero_005_ratio_pct": 0.0,
            "near_zero_015_ratio_pct": 0.0,
            "frame_ids": ["jt128_front_link"],
        },
        "obstacle": {
            "frames_with_points": 100,
            "points_per_frame": {"p50": 500, "p95": 800, "max": 1000},
            "z_min": 0.05, "z_max": 1.5, "z_mean": 0.3,
            "frame_ids": ["map"],
            "stop_polygon_frame": "base_link",
            "self_box_frame": "base_link",
            "forward_bins_frame": "base_link",
            "stop_polygon": {
                "definition": {"x": [-0.3, 0.5], "y": [-0.4, 0.4], "z": [0.05, 0.85], "frame": "base_link"},
                "points_per_frame": {"p50": 0, "p95": 2, "max": 5},
                "frames_with_stop_points": 10,
                "frames_zero_stop": 90,
            },
            "self_box": {
                "definition": {"x": [-0.45, 0.45], "y": [-0.35, 0.35], "z": [-0.2, 0.45], "frame": "base_link"},
                "points_per_frame": {"p50": 0, "p95": 0, "max": 0},
            },
            "forward_bins": {
                "0-0.5m": {"p50": 0, "p95": 1, "max": 3},
                "0-1.0m": {"p50": 5, "p95": 15, "max": 30},
                "0.5-1.5m": {"p50": 5, "p95": 15, "max": 30},
            },
        },
        "ground": {
            "frames": 100,
            "points_per_frame": {"p50": 15000, "p95": 15000, "max": 15000},
            "z_min": -0.30, "z_max": -0.25, "z_mean_mean": -0.28,
            "frame_ids": ["map"],
        },
        "traversability": {
            "frames": 100,
            "known_cells_per_frame": {"p50": 5000, "p95": 5000, "max": 5000},
            "unknown_cells_per_frame": {"p50": 0, "p95": 0, "max": 0},
            "lethal_cells_per_frame": {"p50": 0, "p95": 0, "max": 0},
            "max_cost_per_frame": {"p50": 30, "p95": 50, "max": 70},
            "mean_cost_per_frame": {},
            "mean_cost_overall": 15.0,
            "frame_ids": ["map"],
        },
        "status": {
            "frames": 100,
            "has_self_filter_field": True,
            "dropped_min_range": {"p50": 0, "p95": 0, "max": 0},
            "dropped_self_filter": {"p50": 300, "p95": 500, "max": 800},
            "filtered_points": {"p50": 20000, "p95": 20000, "max": 20000},
            "states_seen": ["ready"],
        },
    }
    with tempfile.NamedTemporaryFile(mode="w", suffix=".md", delete=False) as f:
        _write_markdown_report(report, "test_bag", [], Path(f.name))
        content = Path(f.name).read_text()
    assert "Frames with STOP points" in content
    assert "self_filter" in content.lower()

def test_markdown_report_no_obstacle_still_works():
    report = {
        "bag_path": "/tmp/test_bag",
        "analyzed_at": "2026-01-01",
        "scene": "unknown",
        "mode": "recorded",
        "total_messages": 50,
        "input_frames": 50,
        "obstacle_frames": 0,
        "ground_frames": 0,
        "traversability_frames": 0,
        "status_frames": 0,
        "tf": {"target_frame": "base_link", "obstacle_source_frame": None,
               "static_edges": [], "dynamic_edges": [],
               "tf_missing_frames": 0, "tf_missing_ratio_pct": 0.,
               "tf_fallback_frames": 0, "tf_future_fallback_frames": 0,
               "tf_future_fallback_ratio_pct": 0.},
        "input": {"points_per_frame": {"p50": 20000}, "near_zero_005_per_frame": {},
                  "near_zero_015_per_frame": {}, "near_zero_005_ratio_pct": 0.,
                  "near_zero_015_ratio_pct": 0., "frame_ids": []},
    }
    with tempfile.NamedTemporaryFile(mode="w", suffix=".md", delete=False) as f:
        _write_markdown_report(report, "test_bag", [], Path(f.name))
        content = Path(f.name).read_text()
    assert "Perception Bag Analysis" in content


# ── pass/fail rules ────────────────────────────────────────────────

def _run_pass_fail(report: dict) -> str:
    lines: list[str] = []
    def a(*args): lines.append(" ".join(args))
    _write_pass_fail(report, lines, a)
    return "\n".join(lines)

def test_unknown_scene_no_pass_fail():
    text = _run_pass_fail({"scene": "unknown", "tf": {}, "mode": "recorded"})
    assert "no pass/fail checks applied" in text
    assert "ALL CHECKS PASS" not in text

def test_empty_scene_produces_verdict():
    text = _run_pass_fail({
        "scene": "empty_front_clear",
        "mode": "recorded",
        "tf": {"tf_missing_ratio_pct": 0., "tf_future_fallback_ratio_pct": 0.,
               "obstacle_source_frame": "base_link"},
        "status": {"has_self_filter_field": True,
                   "dropped_self_filter": {"p50": 300, "max": 800}},
        "obstacle": {
            "stop_polygon": {"points_per_frame": {"p95": 1, "max": 5}},
            "forward_bins": {"0-0.5m": {"p95": 1}},
        },
    })
    assert "ALL CHECKS PASS" in text or "SOME CHECKS FAIL" in text

def test_known_scene_missing_status_fails():
    """A known scene without status messages must FAIL."""
    report = {
        "scene": "empty_front_clear",
        "mode": "recorded",
        "tf": {"tf_missing_ratio_pct": 0., "tf_future_fallback_ratio_pct": 0.,
               "obstacle_source_frame": "base_link"},
        "obstacle": {
            "stop_polygon": {"points_per_frame": {"p95": 0, "max": 0}},
            "forward_bins": {"0-0.5m": {"p95": 0}},
        },
    }
    text = _run_pass_fail(report)
    assert "Status topic present" in text
    assert "❌" in text

def test_status_zero_metrics_still_counted():
    """Self-filter p50==0 with max==0 should FAIL the self-filter check."""
    report = {
        "scene": "empty_front_clear",
        "mode": "recorded",
        "tf": {"tf_missing_ratio_pct": 0., "tf_future_fallback_ratio_pct": 0.,
               "obstacle_source_frame": "base_link"},
        "status": {"has_self_filter_field": True,
                   "dropped_self_filter": {"p50": 0, "max": 0}},
        "obstacle": {
            "stop_polygon": {"points_per_frame": {"p95": 0, "max": 0}},
            "forward_bins": {"0-0.5m": {"p95": 0}},
        },
    }
    text = _run_pass_fail(report)
    assert "Self-filter active" in text
    assert "❌" in text  # p50==0 and max==0 → not active → FAIL

def test_tf_future_fallback_is_reported_in_pass_fail():
    report = {
        "scene": "empty_front_clear",
        "mode": "recorded",
        "tf": {"tf_missing_ratio_pct": 0., "tf_future_fallback_ratio_pct": 5.,
               "obstacle_source_frame": "map"},
        "status": {"has_self_filter_field": True,
                   "dropped_self_filter": {"p50": 300, "max": 800}},
        "obstacle": {
            "stop_polygon": {"points_per_frame": {"p95": 0, "max": 0}},
            "forward_bins": {"0-0.5m": {"p95": 0}},
        },
    }
    text = _run_pass_fail(report)
    assert "future fallback" in text.lower()


# ── scene inference ─────────────────────────────────────────────────

def test_infer_scenes():
    assert _infer_scene(Path("20260517_empty_front_clear")) == "empty_front_clear"
    assert _infer_scene(Path("box_front_1m_processed")) == "box_front_1m"
    assert _infer_scene(Path("low_obstacle_front")) == "low_obstacle_front"
    assert _infer_scene(Path("side_obstacle_or_wall")) == "side_obstacle_or_wall"
    assert _infer_scene(Path("random")) == "unknown"


# ── storage detection ───────────────────────────────────────────────

def test_detect_sqlite3_from_metadata(tmp_path):
    import yaml
    (tmp_path / "metadata.yaml").write_text(yaml.dump(
        {"rosbag2_bagfile_information": {"storage_identifier": "sqlite3"}}))
    assert _detect_storage_id(str(tmp_path)) == "sqlite3"

def test_detect_db3_fallback(tmp_path):
    (tmp_path / "dummy.db3").write_text("")
    assert _detect_storage_id(str(tmp_path)) == "sqlite3"


# ── stamp conversion ────────────────────────────────────────────────

def test_stamp_to_ns():
    class S: sec=1; nanosec=500_000_000
    assert _stamp_to_ns(S()) == 1_500_000_000


# ── percentiles ─────────────────────────────────────────────────────

def test_percentiles():
    assert _compute_percentiles([])["p50"] == 0
    assert _compute_percentiles(list(range(101)))["p50"] == 50.
    assert _compute_percentiles([42.])["p50"] == 42.


# ── forward zone 0.5-1.5m isolation ─────────────────────────────────

def test_forward_zone_05_15_excludes_self():
    from analyze_perception_bag import FORWARD_BINS, FW_Y_MIN, FW_Y_MAX, FW_Z_MIN, FW_Z_MAX
    pts = np.array([
        [0.2, 0., 0.2], [0.7, 0., 0.3], [1.2, 0., 0.3], [1.8, 0., 0.3]], dtype=np.float64)
    yz = (pts[:,1]>=FW_Y_MIN) & (pts[:,1]<=FW_Y_MAX) & (pts[:,2]>=FW_Z_MIN) & (pts[:,2]<=FW_Z_MAX)
    c = {}
    for fwd in FORWARD_BINS:
        c[fwd] = int(np.sum(yz & (pts[:,0]>=0) & (pts[:,0]<=fwd)))
    c[1.55] = int(np.sum(yz & (pts[:,0]>=0.5) & (pts[:,0]<=1.5)))
    assert c[0.5] == 1   # only pt[0]
    assert c[1.55] == 2  # pt[1]+pt[2]
    assert c[2.0] == 4   # all four


# ── safe helpers ────────────────────────────────────────────────────

def test_safe_min_empty_returns_none():
    assert _safe_min_or_none([]) is None
    assert _safe_min_or_none([None, None]) is None

def test_safe_min_valid():
    assert _safe_min_or_none([3., 1., 2.]) == 1.
    assert _safe_min_or_none([None, 3., None]) == 3.

def test_safe_mean_empty_returns_none():
    assert _safe_mean_or_none([]) is None

def test_fmt_or_na_none():
    assert _fmt_or_na(None) == "N/A"

def test_fmt_or_na_value():
    assert _fmt_or_na(3.14159, ".2f") == "3.14"


# ── empty cloud z stats do not crash ────────────────────────────────

def test_obstacle_empty_cloud_zs_are_none_not_crash():
    """All obstacle frames have 0 points → z min/max/mean should be None, not crash."""
    obs = {
        "frames_total": 50, "frames_with_points": 0,
        "points_per_frame": {"p50": 0, "p95": 0, "max": 0},
        "z_min": None, "z_max": None, "z_mean": None,
        "frame_ids": ["map"],
        "stop_polygon_frame": "base_link",
        "self_box_frame": "base_link",
        "forward_bins_frame": "base_link",
        "stop_polygon": {
            "definition": {"x": [-0.3, 0.5], "y": [-0.4, 0.4], "z": [0.05, 0.85], "frame": "base_link"},
            "points_per_frame": {"p50": 0, "p95": 0, "max": 0},
            "frames_with_stop_points": 0, "frames_zero_stop": 50,
        },
        "self_box": {
            "definition": {"x": [-0.45, 0.45], "y": [-0.35, 0.35], "z": [-0.2, 0.45], "frame": "base_link"},
            "points_per_frame": {"p50": 0, "p95": 0, "max": 0},
        },
        "forward_bins": {"0-0.5m": {"p50": 0, "p95": 0, "max": 0}},
    }
    report = {
        "bag_path": "/t", "analyzed_at": "x", "scene": "empty_front_clear",
        "mode": "recorded", "total_messages": 50,
        "input_frames": 0, "obstacle_frames": 50,
        "ground_frames": 0, "traversability_frames": 0, "status_frames": 50,
        "tf": {"target_frame": "base_link", "obstacle_source_frame": "base_link",
               "static_edges": [], "dynamic_edges": [],
               "tf_missing_frames": 0, "tf_missing_ratio_pct": 0.,
               "tf_fallback_frames": 0, "tf_future_fallback_frames": 0,
               "tf_future_fallback_ratio_pct": 0.},
        "obstacle": obs,
        "status": {"has_self_filter_field": True, "frames": 50,
                   "dropped_self_filter": {"p50": 300, "max": 800}},
    }
    with tempfile.NamedTemporaryFile(mode="w", suffix=".md", delete=False) as f:
        _write_markdown_report(report, "test_bag", [], Path(f.name))
        content = Path(f.name).read_text()
    assert "N/A" in content  # z values rendered as N/A
    assert "with_points=0" in content


def test_ground_empty_cloud_zs_are_none_not_crash():
    gnd = {
        "frames_total": 50, "frames_with_points": 0,
        "points_per_frame": {"p50": 0, "p95": 0, "max": 0},
        "z_min": None, "z_max": None, "z_mean_mean": None,
        "frame_ids": ["map"],
    }
    report = {
        "bag_path": "/t", "analyzed_at": "x", "scene": "unknown",
        "mode": "recorded", "total_messages": 50,
        "input_frames": 0, "obstacle_frames": 0,
        "ground_frames": 50, "traversability_frames": 0, "status_frames": 0,
        "tf": {"target_frame": "base_link", "obstacle_source_frame": None,
               "static_edges": [], "dynamic_edges": [],
               "tf_missing_frames": 0, "tf_missing_ratio_pct": 0.,
               "tf_fallback_frames": 0, "tf_future_fallback_frames": 0,
               "tf_future_fallback_ratio_pct": 0.},
        "ground": gnd,
    }
    with tempfile.NamedTemporaryFile(mode="w", suffix=".md", delete=False) as f:
        _write_markdown_report(report, "test_bag", [], Path(f.name))
        content = Path(f.name).read_text()
    assert "N/A" in content  # z values rendered as N/A
    assert "with_points=0" in content


# ── known scene missing critical topic ──────────────────────────────

def test_known_scene_missing_obstacle_fails():
    """empty_front_clear without obstacle topic must FAIL, not ALL PASS."""
    report = {
        "scene": "empty_front_clear", "mode": "recorded",
        "tf": {"tf_missing_ratio_pct": 0., "tf_future_fallback_ratio_pct": 0.,
               "obstacle_source_frame": None},
        "status": {"has_self_filter_field": True,
                   "dropped_self_filter": {"p50": 300, "max": 800}},
    }
    text = _run_pass_fail(report)
    assert "Obstacle topic present" in text
    assert "❌" in text  # obstacle missing → FAIL
    assert "ALL CHECKS PASS" not in text


def test_low_obstacle_missing_traversability_fails():
    """low_obstacle_front without traversability must FAIL that check."""
    report = {
        "scene": "low_obstacle_front", "mode": "recorded",
        "tf": {"tf_missing_ratio_pct": 0., "tf_future_fallback_ratio_pct": 0.,
               "obstacle_source_frame": "base_link"},
        "status": {"has_self_filter_field": True,
                   "dropped_self_filter": {"p50": 300, "max": 800}},
        "obstacle": {
            "frames_total": 50, "frames_with_points": 50,
            "points_per_frame": {"p50": 100}, "z_min": 0.1, "z_max": 0.5, "z_mean": 0.2,
            "frame_ids": ["map"], "stop_polygon_frame": "base_link",
            "forward_bins_frame": "base_link",
            "stop_polygon": {"definition": {"x": [0,1], "y": [0,1], "z": [0,1], "frame": "b"},
                             "points_per_frame": {"p50":0}, "frames_with_stop_points": 0,
                             "frames_zero_stop": 50},
            "forward_bins": {"0.5-1.5m": {"p95": 5}},
        },
    }
    text = _run_pass_fail(report)
    assert "Traversability topic present" in text
    assert "❌" in text


def test_empty_scene_zero_obstacle_points_can_pass():
    """empty_front_clear with obstacle topic + all-zero points + valid self-filter → CAN pass."""
    report = {
        "scene": "empty_front_clear", "mode": "regenerated",
        "tf": {"tf_missing_ratio_pct": 0., "tf_future_fallback_ratio_pct": 0.,
               "obstacle_source_frame": "base_link"},
        "status": {"has_self_filter_field": True,
                   "dropped_self_filter": {"p50": 300, "max": 800}},
        "obstacle": {
            "frames_total": 50, "frames_with_points": 0,
            "points_per_frame": {"p50": 0, "p95": 0, "max": 0},
            "z_min": None, "z_max": None, "z_mean": None,
            "frame_ids": ["base_link"],
            "stop_polygon_frame": "base_link",
            "self_box_frame": "base_link",
            "forward_bins_frame": "base_link",
            "stop_polygon": {
                "definition": {"x": [-0.3, 0.5], "y": [-0.4, 0.4], "z": [0.05, 0.85], "frame": "base_link"},
                "points_per_frame": {"p50": 0, "p95": 0, "max": 0},
                "frames_with_stop_points": 0, "frames_zero_stop": 50,
            },
            "self_box": {
                "definition": {"x": [-0.45, 0.45], "y": [-0.35, 0.35], "z": [-0.2, 0.45], "frame": "base_link"},
                "points_per_frame": {"p50": 0},
            },
            "forward_bins": {"0-0.5m": {"p95": 0, "p50": 0}},
        },
    }
    text = _run_pass_fail(report)
    assert "ALL CHECKS PASS" in text


def test_box_front_zero_obstacle_points_fails():
    """box_front_1m with obstacle topic but all zero points must FAIL."""
    report = {
        "scene": "box_front_1m", "mode": "recorded",
        "tf": {"tf_missing_ratio_pct": 0., "tf_future_fallback_ratio_pct": 0.,
               "obstacle_source_frame": "base_link"},
        "status": {"has_self_filter_field": True,
                   "dropped_self_filter": {"p50": 300, "max": 800}},
        "obstacle": {
            "frames_total": 50, "frames_with_points": 0,
            "points_per_frame": {"p50": 0}, "z_min": None, "z_max": None, "z_mean": None,
            "frame_ids": ["base_link"],
            "stop_polygon_frame": "base_link", "forward_bins_frame": "base_link",
            "stop_polygon": {"definition": {"x": [0,1], "y": [0,1], "z": [0,1], "frame": "b"},
                             "points_per_frame": {"p50":0}, "frames_with_stop_points": 0,
                             "frames_zero_stop": 50},
            "forward_bins": {"0.5-1.5m": {"p50": 0, "p95": 0}},
        },
    }
    text = _run_pass_fail(report)
    assert "has points" in text
    assert "❌" in text  # zero obstacle points → FAIL for box_front_1m


# ── markdown formats None as N/A ────────────────────────────────────

def test_markdown_formats_none_z_as_na():
    report = {
        "bag_path": "/t", "analyzed_at": "x", "scene": "unknown",
        "mode": "recorded", "total_messages": 50,
        "input_frames": 0, "obstacle_frames": 50,
        "ground_frames": 50, "traversability_frames": 0, "status_frames": 0,
        "tf": {"target_frame": "base_link", "obstacle_source_frame": "base_link",
               "static_edges": [], "dynamic_edges": [],
               "tf_missing_frames": 0, "tf_missing_ratio_pct": 0.,
               "tf_fallback_frames": 0, "tf_future_fallback_frames": 0,
               "tf_future_fallback_ratio_pct": 0.},
        "obstacle": {
            "frames_total": 50, "frames_with_points": 0,
            "points_per_frame": {"p50": 0}, "z_min": None, "z_max": None, "z_mean": None,
            "frame_ids": ["base_link"], "stop_polygon_frame": "base_link",
            "self_box_frame": "base_link", "forward_bins_frame": "base_link",
            "stop_polygon": {
                "definition": {"x": [0,1], "y": [0,1], "z": [0,1], "frame": "b"},
                "points_per_frame": {"p50":0}, "frames_with_stop_points": 0,
                "frames_zero_stop": 50,
            },
            "self_box": {"definition": {"x": [0,1], "y": [0,1], "z": [0,1], "frame": "b"},
                         "points_per_frame": {"p50": 0}},
            "forward_bins": {},
        },
        "ground": {
            "frames_total": 50, "frames_with_points": 0,
            "points_per_frame": {"p50": 0},
            "z_min": None, "z_max": None, "z_mean_mean": None,
            "frame_ids": ["map"],
        },
    }
    with tempfile.NamedTemporaryFile(mode="w", suffix=".md", delete=False) as f:
        _write_markdown_report(report, "test_bag", [], Path(f.name))
        content = Path(f.name).read_text()
    assert "N/A" in content
    assert "with_points=0" in content
