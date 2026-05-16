"""Lightweight tests for nav2_corridor_gate sample schema.

The gate JSON is consumed by humans and by ``runtime/test_records/*``. After
the static-map / costmap audit the per-sample schema must let us tell a
``/map`` blocker apart from a costmap-inflation blocker.
"""

from __future__ import annotations

import importlib.util
import sys
from dataclasses import asdict
from pathlib import Path


def _load_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "nav2_corridor_gate.py"
    spec = importlib.util.spec_from_file_location("nav2_corridor_gate_under_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


corridor_gate = _load_module()


def test_grid_probe_separates_static_and_costmap_fields():
    probe = corridor_gate.GridProbe(
        sample_x=1.0,
        sample_y=2.0,
        sample_reason="static_map_blocker",
        static_gx=10,
        static_gy=11,
        static_value=100,
        costmap_gx=20,
        costmap_gy=21,
        costmap_value=42,
        nearest_occupied_distance=0.07,
        nearest_occupied_x=1.05,
        nearest_occupied_y=2.0,
    )
    payload = asdict(probe)

    # Schema must distinguish static vs costmap probes — the whole point of
    # the audit fix. Old ambiguous names must NOT come back.
    assert "static_gx" in payload and "static_value" in payload
    assert "costmap_gx" in payload and "costmap_value" in payload
    for legacy in ("grid_x", "grid_y", "value", "state",
                   "static_grid_x", "costmap_grid_x", "nearest_occupied_m"):
        assert legacy not in payload, f"legacy field {legacy} re-introduced"

    assert payload["sample_reason"] == "static_map_blocker"
    assert payload["nearest_occupied_distance"] == 0.07
    assert payload["nearest_occupied_x"] == 1.05
    assert payload["nearest_occupied_y"] == 2.0


def test_nearest_occupied_distance_returns_xy_triple():
    centers = [(2.0, 0.0), (0.0, 3.0)]
    distance, ox, oy = corridor_gate.nearest_occupied_distance((0.5, 0.0), centers)
    assert ox == 2.0 and oy == 0.0
    assert abs(distance - 1.5) < 1e-9


def test_nearest_occupied_distance_empty_returns_none_triple():
    distance, ox, oy = corridor_gate.nearest_occupied_distance((0.0, 0.0), [])
    assert distance is None and ox is None and oy is None
