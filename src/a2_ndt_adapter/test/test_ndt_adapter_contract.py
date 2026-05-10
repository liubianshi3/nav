import math

import numpy as np

from a2_ndt_adapter.ndt_adapter_node import (
    clamp_map_radius,
    make_map_cell_id,
    matrix_to_quaternion,
    quaternion_to_matrix,
    score_is_acceptable,
    select_points_for_area,
)


def test_score_gate_min_is_good():
    assert score_is_acceptable(2.4, 2.3, True)
    assert not score_is_acceptable(2.2, 2.3, True)


def test_score_gate_max_is_good():
    assert score_is_acceptable(2.9, 3.0, False)
    assert not score_is_acceptable(3.1, 3.0, False)


def test_score_gate_rejects_missing_or_bad_values():
    assert not score_is_acceptable(None, 2.3, True)
    assert not score_is_acceptable(float("nan"), 2.3, True)
    assert not score_is_acceptable(float("inf"), 2.3, True)


def test_quaternion_matrix_round_trip_yaw():
    yaw = math.pi / 3.0
    q = np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])
    rotation = quaternion_to_matrix(*q)
    q_back = np.array(matrix_to_quaternion(rotation))
    assert np.allclose(q, q_back) or np.allclose(q, -q_back)


def test_map_radius_is_clamped_for_dynamic_loading():
    assert clamp_map_radius(0.0, 1.0, 150.0) == 150.0
    assert clamp_map_radius(float("nan"), 1.0, 150.0) == 150.0
    assert clamp_map_radius(0.5, 1.0, 150.0) == 1.0
    assert clamp_map_radius(200.0, 1.0, 150.0) == 150.0
    assert clamp_map_radius(25.0, 1.0, 150.0) == 25.0


def test_select_points_for_area_filters_and_downsamples():
    points = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [4.0, 0.0, 0.0],
        ],
        dtype=np.float32,
    )
    selected = select_points_for_area(points, 0.0, 0.0, radius=2.0, margin=0.0, max_points=2)
    assert selected.shape == (2, 3)
    assert np.all(selected[:, 0] <= 2.0)


def test_map_cell_id_is_stable_and_safe_for_cached_ids():
    cell_id = make_map_cell_id("a2_map_cell", -1.25, 3.5, 40.0)
    assert cell_id == "a2_map_cell_m1p2_3p5_r40p0"
