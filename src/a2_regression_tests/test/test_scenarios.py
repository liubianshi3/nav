#!/usr/bin/env python3
"""
Regression scenario tests — replay rosbag and validate metrics.

Each scenario expects a rosbag in test/bags/<scenario_name>/.
If the bag is not present, the test is skipped (not failed).

Bags can be downloaded from internal storage or recorded from the robot.
"""

import json
import os
import sys
from pathlib import Path

import pytest
import yaml


BAGS_DIR = Path(__file__).resolve().parent / "bags"
CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "regression_thresholds.yaml"


def _load_thresholds():
    if not CONFIG_PATH.exists():
        return {}
    with open(CONFIG_PATH) as f:
        return yaml.safe_load(f) or {}


THRESHOLDS = _load_thresholds()


SCENARIOS = [
    "straight_line",
    "turning",
    "obstacle_avoidance",
    "kidnapping",
    "full_mission",
]


def _bag_available(name: str) -> bool:
    """Check if rosbag directory exists and contains data."""
    bag_dir = BAGS_DIR / name
    if not bag_dir.is_dir():
        return False
    has_data = any(
        bag_dir.glob("*.mcap") or bag_dir.glob("*.db3") or bag_dir.glob("metadata.yaml")
    )
    return has_data


def _check_metrics(metrics: dict, scenario: str) -> list[str]:
    """Validate metrics against thresholds. Returns list of failures."""
    thresholds = THRESHOLDS.get("scenarios", {}).get(scenario, {})
    failures = []
    for key, limit in thresholds.items():
        if key.startswith("max_"):
            actual = metrics.get(key[4:])
            if actual is not None and actual > limit:
                failures.append(f"{key[4:]}={actual:.3f} > max={limit}")
        elif key.startswith("min_"):
            actual = metrics.get(key[4:])
            if isinstance(limit, bool):
                if bool(actual) != limit:
                    failures.append(f"{key[4:]}={actual} != expected={limit}")
            elif actual is not None and actual < limit:
                failures.append(f"{key[4:]}={actual} < min={limit}")
    return failures


def _run_bag_scenario(name: str) -> dict:
    """Replay a rosbag and extract metrics.

    This test must not manufacture passable metrics. Until the replay harness is
    implemented, a present bag is an explicit failure so CI does not mistake this
    placeholder for industrial validation.
    """
    bag_dir = BAGS_DIR / name
    raise NotImplementedError(
        "Rosbag replay metrics are not implemented yet. "
        f"Implement launch/playback/metric extraction before enabling bag scenario '{name}' at {bag_dir}."
    )


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_regression_scenario(scenario: str):
    """Regression test for a navigation scenario.

    Skips if the rosbag is not available (graceful degradation
    for CI environments without bag storage).
    """
    if not _bag_available(scenario):
        pytest.skip(f"Rosbag not available for scenario '{scenario}'. "
                    f"Place bag at {BAGS_DIR / scenario}/ to enable.")

    metrics = _run_bag_scenario(scenario)
    failures = _check_metrics(metrics, scenario)

    # Write metrics JSON for CI consumption
    results_dir = Path(os.environ.get("REGRESSION_RESULTS_DIR", "/tmp/a2_regression_results"))
    results_dir.mkdir(parents=True, exist_ok=True)
    metrics_path = results_dir / f"{scenario}_metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2))

    assert not failures, (
        f"Scenario '{scenario}' regression failures:\n"
        + "\n".join(f"  - {f}" for f in failures)
    )


def test_all_scenarios_defined():
    """Ensure all required scenarios have threshold configs."""
    for scenario in SCENARIOS:
        assert scenario in THRESHOLDS.get("scenarios", {}), (
            f"No thresholds defined for scenario '{scenario}'"
        )


def test_global_thresholds_present():
    """Ensure global safety thresholds are defined."""
    global_cfg = THRESHOLDS.get("global", {})
    assert "hard_clearance_m" in global_cfg
    assert "near_collision_m" in global_cfg
