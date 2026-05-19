import math
from pathlib import Path

import numpy as np

from a2_ndt_adapter.pose_math import (
    choose_periodic_initial_guess_stamp,
    choose_ndt_initial_stamp,
    clamp_map_radius,
    compose_map_pose_from_odom,
    make_map_cell_id,
    matrix_to_quaternion,
    quaternion_to_matrix,
    score_is_acceptable,
    select_points_for_area,
    seeded_odom_tracking_status,
    should_feed_ndt_pose_buffer,
    should_publish_periodic_guess,
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


def test_ndt_initial_stamp_prefers_latest_cloud_when_enabled():
    candidate_stamp = object()
    cloud_stamp = object()

    assert choose_ndt_initial_stamp(candidate_stamp, cloud_stamp, True) is cloud_stamp
    assert choose_ndt_initial_stamp(candidate_stamp, cloud_stamp, False) is candidate_stamp
    assert choose_ndt_initial_stamp(candidate_stamp, None, True) is candidate_stamp


def test_periodic_initial_guess_stamp_can_follow_odom_for_interpolation_buffer():
    candidate_stamp = object()
    cloud_stamp = object()

    assert choose_periodic_initial_guess_stamp(candidate_stamp, cloud_stamp, False) is candidate_stamp
    assert choose_periodic_initial_guess_stamp(candidate_stamp, cloud_stamp, True) is cloud_stamp


def test_periodic_initial_guess_publish_gate():
    assert should_publish_periodic_guess(None, 0.1)
    assert should_publish_periodic_guess(0.05, 0.1, force=True)
    assert should_publish_periodic_guess(0.11, 0.1)
    assert not should_publish_periodic_guess(0.05, 0.1)
    assert should_publish_periodic_guess(0.0, 0.0)


def test_ndt_pose_buffer_feed_continues_after_first_fix():
    assert should_feed_ndt_pose_buffer(
        has_seed=True,
        odom_available=True,
        awaiting_first_ndt_fix=True,
    )
    assert should_feed_ndt_pose_buffer(
        has_seed=True,
        odom_available=True,
        awaiting_first_ndt_fix=False,
    )
    assert not should_feed_ndt_pose_buffer(
        has_seed=False,
        odom_available=True,
        awaiting_first_ndt_fix=True,
    )
    assert not should_feed_ndt_pose_buffer(
        has_seed=True,
        odom_available=False,
        awaiting_first_ndt_fix=False,
    )


def test_map_pose_from_odom_uses_current_map_to_odom_anchor():
    odom_to_base = np.eye(4, dtype=np.float64)
    odom_to_base[:3, 3] = [1.2, -0.5, 0.0]
    map_to_odom = np.eye(4, dtype=np.float64)
    map_to_odom[:3, 3] = [2.0, 3.0, 0.0]

    map_to_base = compose_map_pose_from_odom(map_to_odom, odom_to_base)

    assert np.allclose(map_to_base[:3, 3], [3.2, 2.5, 0.0])


def test_seeded_odom_tracking_remains_ready_when_ndt_score_is_stale():
    ready, state, reason = seeded_odom_tracking_status(
        has_seed=True,
        odom_fresh=True,
        score=6.7,
        score_threshold=2.3,
        score_min_is_good=True,
        map_ready=True,
    )

    assert ready is True
    assert state == "tracking"
    assert reason == "odom_tracking"


def test_seeded_odom_tracking_requires_a_valid_prior_ndt_score():
    ready, state, reason = seeded_odom_tracking_status(
        has_seed=True,
        odom_fresh=True,
        score=-1.0,
        score_threshold=2.3,
        score_min_is_good=True,
        map_ready=True,
    )

    assert ready is False
    assert state == "waiting_first_score"
    assert reason == "ndt_not_scored_yet"


def test_ndt_adapter_logs_runtime_correction_limits():
    source = Path(__file__).resolve().parents[1] / "a2_ndt_adapter" / "ndt_adapter_node.py"
    text = source.read_text(encoding="utf-8")

    assert "(limits: 1.0m, 20deg)" not in text
    assert "max_map_to_odom_translation_step" in text
    assert "max_map_to_odom_rotation_step_deg" in text


def test_ndt_adapter_initializes_status_timestamps_before_first_pose():
    source = Path(__file__).resolve().parents[1] / "a2_ndt_adapter" / "ndt_adapter_node.py"
    text = source.read_text(encoding="utf-8")

    assert "self.last_pose_stamp = None" in text
    assert "self.last_odom_receive_time = None" in text
    assert "self.last_odom_msg_stamp = None" in text
    assert "self.last_odom_receive_time = self.get_clock().now()" in text
