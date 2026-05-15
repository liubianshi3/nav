from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path


def load_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "abc_isolation_eval.py"
    spec = importlib.util.spec_from_file_location("abc_isolation_eval_under_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


eval_mod = load_module()


def make_pose(x: float, y: float, yaw: float) -> eval_mod.PoseSample:
    return eval_mod.PoseSample(x=x, y=y, yaw=yaw, frame_id="map", stamp_monotonic=0.0)


def test_parse_status_fields_extracts_semicolon_pairs():
    fields = eval_mod.parse_status_fields("mode=real;state=goal_running;ready=true;reason=distance_remaining=0.42")

    assert fields["mode"] == "real"
    assert fields["state"] == "goal_running"
    assert fields["ready"] == "true"
    assert fields["reason"] == "distance_remaining=0.42"


def test_path_length_sums_segment_distances():
    samples = [
        make_pose(0.0, 0.0, 0.0),
        make_pose(3.0, 4.0, 0.0),
        make_pose(6.0, 8.0, 0.0),
    ]

    assert math.isclose(eval_mod.path_length(samples), 10.0, rel_tol=1e-6)


def test_yaw_error_deg_wraps_across_pi_boundary():
    reference = make_pose(0.0, 0.0, math.radians(179.0))
    actual = make_pose(0.0, 0.0, math.radians(-179.0))

    assert math.isclose(eval_mod.yaw_error_deg(reference, actual), 2.0, abs_tol=1e-6)
