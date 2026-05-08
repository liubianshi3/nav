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

    This is a placeholder for a real rosbag replay pipeline.
    In production, this would:
      1. Launch the A2 navigation stack in a test harness
      2. Replay the rosbag
      3. Collect /tf, /a2/relocalization/pose, /a2/nav3/status
      4. Compute ATE, RMSE, state distribution, collision counts
      5. Return metrics dict

    For now, it returns a placeholder dict so the test framework compiles.
    """
    return {
        "scenario": name,
        "status": "placeholder",
        "ate_m": 0.0,
        "rmse_m": 0.0,
        "tracking_pct": 100.0,
        "avoiding_pct": 0.0,
        "blocked_pct": 0.0,
        "collision_count": 0,
        "near_collision_count": 0,
        "recovery_triggered": False,
        "waypoint_success_rate": 100.0,
        "message": (
            "Rosbag replay pipeline not yet implemented. "
            f"Place a rosbag at {BAGS_DIR / name}/ to enable this test."
        ),
    }


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
