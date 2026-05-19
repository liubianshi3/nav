from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "src/a2_system/scripts/pointcloud_preview_node.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("pointcloud_preview_node", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_preview_topic_name_appends_suffix_without_touching_source_topic():
    module = _load_module()

    assert module.preview_topic_name("/jt128/front/points") == "/jt128/front/points_preview"
    assert module.preview_topic_name("/jt128/dlio/map_points") == "/jt128/dlio/map_points_preview"
    assert module.preview_topic_name("/jt128/front/points_preview") == "/jt128/front/points_preview"


def test_preview_rate_gate_publishes_first_sample_and_limits_rate():
    module = _load_module()

    assert module.should_publish_preview(None, 1_000_000_000, 5.0)
    assert not module.should_publish_preview(1_000_000_000, 1_100_000_000, 5.0)
    assert module.should_publish_preview(1_000_000_000, 1_250_000_000, 5.0)
    assert module.should_publish_preview(1_000_000_000, 1_000_000_001, 0.0)


def test_prepare_preview_points_filters_voxels_and_caps_points():
    module = _load_module()
    points = np.array(
        [
            [0.00, 0.00, 0.00],
            [0.01, 0.01, 0.01],
            [1.00, 0.00, 0.00],
            [2.00, 0.00, 0.00],
            [float("nan"), 0.0, 0.0],
        ],
        dtype=np.float32,
    )

    preview = module.prepare_preview_points(
        points,
        voxel_size_m=0.1,
        min_range_m=0.0,
        max_range_m=0.0,
        max_points=2,
    )

    assert preview.shape == (2, 3)
    assert np.isfinite(preview).all()
    assert [tuple(row) for row in preview] == [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)]


def test_dlio_mapping_launch_starts_visualization_preview_nodes():
    launch = (ROOT / "src/a2_bringup/launch/dlio_mapping.launch.py").read_text(encoding="utf-8")

    assert "start_pointcloud_previews" in launch
    assert "pointcloud_preview_node.py" in launch
    assert "/jt128/front/points_preview" in launch
    assert "/jt128/dlio/map_points_preview" in launch


def test_a2_system_installs_pointcloud_preview_executable():
    cmake = (ROOT / "src/a2_system/CMakeLists.txt").read_text(encoding="utf-8")

    assert "scripts/pointcloud_preview_node.py" in cmake
