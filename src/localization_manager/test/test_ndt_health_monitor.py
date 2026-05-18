from __future__ import annotations

from localization_manager.ndt_health_monitor import (
    classify_ndt_health_status,
    parse_ndt_status,
)


def test_parse_healthy_status():
    parsed = parse_ndt_status(
        "state=ready;ready=true;reason=converged;matcher=autoware_ndt;"
        "score=0.850;score_threshold=0.500;iteration_num=12;"
        "map_ready=true;map_points=5000;live_cloud_topic=/jt128/front/points"
    )
    assert parsed["state"] == "ready"
    assert parsed["ready"] == "true"
    assert parsed["score"] == "0.850"
    assert parsed["iteration_num"] == "12"
    assert parsed["map_ready"] == "true"


def test_parse_degraded_status():
    parsed = parse_ndt_status(
        "state=rejected;ready=false;reason=score_below_threshold;"
        "matcher=autoware_ndt;score=0.120;score_threshold=0.500;iteration_num=30;"
        "map_ready=true;map_points=5000"
    )
    assert parsed["state"] == "rejected"
    assert parsed["ready"] == "false"
    assert parsed["score"] == "0.120"


def test_parse_empty():
    assert parse_ndt_status("") == {}


def test_parse_malformed():
    parsed = parse_ndt_status("garbage_without_equals;;score=0.5")
    assert parsed.get("score") == "0.5"


def test_classifies_waiting_initial_guess():
    parsed = parse_ndt_status(
        "state=waiting_seed;ready=false;reason=send_initialpose;"
        "score=-1.000;initial_guess_count=0"
    )

    healthy, state, reason = classify_ndt_health_status(parsed, min_score=2.3)

    assert healthy is False
    assert state == "waiting_initial_guess"
    assert reason == "initial_guess_missing"


def test_classifies_pose_buffer_insufficient_from_stale_guess_feed():
    parsed = parse_ndt_status(
        "state=tracking;ready=true;reason=odom_tracking;score=3.786;"
        "score_fresh=true;initial_guess_count=3;last_initial_guess_age=1.500"
    )

    healthy, state, reason = classify_ndt_health_status(
        parsed,
        min_score=2.3,
        initial_guess_timeout_sec=1.0,
    )

    assert healthy is False
    assert state == "pose_buffer_insufficient"
    assert reason == "initial_guess_stale"


def test_classifies_no_recent_score_before_generic_failure():
    parsed = parse_ndt_status(
        "state=tracking;ready=true;reason=odom_tracking;score=3.786;"
        "score_fresh=false;initial_guess_count=20;last_initial_guess_age=0.050"
    )

    healthy, state, reason = classify_ndt_health_status(parsed, min_score=2.3)

    assert healthy is False
    assert state == "no_recent_ndt_score"
    assert reason == "score_stale"


def test_classifies_score_low():
    parsed = parse_ndt_status(
        "state=rejected;ready=false;reason=score_below_threshold;score=1.200;"
        "score_fresh=true;initial_guess_count=20;last_initial_guess_age=0.050"
    )

    healthy, state, reason = classify_ndt_health_status(parsed, min_score=2.3)

    assert healthy is False
    assert state == "score_low"
    assert reason == "score_below_threshold"


def test_classifies_healthy_status():
    parsed = parse_ndt_status(
        "state=ready;ready=true;reason=converged;score=3.800;"
        "score_fresh=true;initial_guess_count=20;last_initial_guess_age=0.050"
    )

    healthy, state, reason = classify_ndt_health_status(parsed, min_score=2.3)

    assert healthy is True
    assert state == "healthy"
    assert reason == "converged"
