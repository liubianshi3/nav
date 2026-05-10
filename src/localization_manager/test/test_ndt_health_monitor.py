from __future__ import annotations

from localization_manager.ndt_health_monitor import parse_ndt_status


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
