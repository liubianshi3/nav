from __future__ import annotations

import importlib.util
import struct
import sys
from pathlib import Path

from sensor_msgs.msg import PointCloud2, PointField


def load_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "octomap_mapping_node.py"
    spec = importlib.util.spec_from_file_location("octomap_mapping_node_under_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


octomap_mapping = load_module()


def make_cloud(points: list[tuple[float, float, float]]) -> PointCloud2:
    msg = PointCloud2()
    msg.header.frame_id = "jt128_front_link"
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.data = b"".join(struct.pack("<fff", *point) for point in points)
    msg.is_dense = True
    return msg


def unpack_cloud(msg: PointCloud2) -> list[tuple[float, float, float]]:
    return [
        struct.unpack_from("<fff", msg.data, index * msg.point_step)
        for index in range(msg.width)
    ]


def assert_points_close(
    actual: list[tuple[float, float, float]],
    expected: list[tuple[float, float, float]],
) -> None:
    assert len(actual) == len(expected)
    for actual_point, expected_point in zip(actual, expected):
        for actual_value, expected_value in zip(actual_point, expected_point):
            assert abs(actual_value - expected_value) < 1.0e-5


def test_filter_octomap_cloud_removes_self_box_points_and_preserves_others():
    msg = make_cloud([
        (0.0, 0.0, 0.0),
        (1.2, 0.0, 0.0),
        (0.0, 0.8, 0.0),
    ])

    filtered, stats = octomap_mapping.filter_octomap_cloud(
        msg,
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        (-0.7, 0.7, -0.45, 0.45, -0.3, 0.8),
        0.0,
        12.0,
        True,
    )

    assert stats.input_points == 3
    assert stats.self_points == 1
    assert stats.kept_points == 2
    assert filtered.header.frame_id == "jt128_front_link"
    assert_points_close(
        unpack_cloud(filtered),
        [(1.2, 0.0, 0.0), (0.0, 0.8, 0.0)],
    )


def test_filter_octomap_cloud_applies_min_range_before_octomap():
    msg = make_cloud([(0.05, 0.0, 0.0), (0.4, 0.0, 0.0)])

    filtered, stats = octomap_mapping.filter_octomap_cloud(
        msg,
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        (-0.01, 0.01, -0.01, 0.01, -0.01, 0.01),
        0.20,
        12.0,
        True,
    )

    assert stats.range_points == 1
    assert stats.kept_points == 1
    assert_points_close(unpack_cloud(filtered), [(0.4, 0.0, 0.0)])
