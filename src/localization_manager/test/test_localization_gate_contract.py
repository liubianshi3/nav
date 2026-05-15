from __future__ import annotations

from localization_manager.localization_gate import evaluate_localization_contract


def covariance(x=0.05, y=0.05, yaw=0.05):
    values = [0.0] * 36
    values[0] = x
    values[7] = y
    values[35] = yaw
    return values


def test_ready_pose_contract():
    ready, state, reason, pose_ok = evaluate_localization_contract(
        age_sec=0.1,
        covariance=covariance(),
        max_pose_age_sec=0.5,
        max_xy_variance=0.2,
        max_yaw_variance=0.15,
    )
    assert ready is True
    assert state == "ready"
    assert "pose_ok" in reason
    assert pose_ok is True


def test_stale_pose_contract():
    ready, state, reason, pose_ok = evaluate_localization_contract(
        age_sec=2.0,
        covariance=covariance(),
        max_pose_age_sec=0.5,
        max_xy_variance=0.2,
        max_yaw_variance=0.15,
    )
    assert ready is False
    assert state == "stale_pose"
    assert "pose_timeout" in reason
    assert pose_ok is True


def test_covariance_rejection_contract():
    ready, state, reason, pose_ok = evaluate_localization_contract(
        age_sec=0.1,
        covariance=covariance(x=0.5),
        max_pose_age_sec=0.5,
        max_xy_variance=0.2,
        max_yaw_variance=0.15,
    )
    assert ready is False
    assert state == "covariance_rejected"
    assert "xy_ok=false" in reason
    assert pose_ok is False


def test_latched_pose_contract():
    ready, state, reason, pose_ok = evaluate_localization_contract(
        age_sec=2.0,
        covariance=covariance(),
        max_pose_age_sec=0.5,
        max_xy_variance=0.2,
        max_yaw_variance=0.15,
        latch_valid_pose=True,
        latched_age_sec=10.0,
        latched_pose_timeout_sec=60.0,
    )
    assert ready is True
    assert state == "ready"
    assert "pose_latched" in reason
    assert pose_ok is True
