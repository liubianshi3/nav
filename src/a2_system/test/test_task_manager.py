from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def load_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "task_manager.py"
    spec = importlib.util.spec_from_file_location("task_manager_under_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


task_manager = load_module()


def test_normalize_route_yaml_rewrites_waypoints_and_mission_name():
    mission_name, waypoints, normalized = task_manager.normalize_route_yaml(
        """
mission_name: office_loop
waypoints:
  - id: p1
    x: 1
    y: 2
    yaw: 6.5
    dwell_sec: 1.0
    note: start
""",
        default_mission_name="fallback",
    )

    assert mission_name == "office_loop"
    assert len(waypoints) == 1
    assert waypoints[0].waypoint_id == "p1"
    assert -3.141592653589793 <= waypoints[0].yaw <= 3.141592653589793
    assert "mission_name: office_loop" in normalized
    assert "id: p1" in normalized


def test_normalize_route_yaml_rejects_duplicate_ids():
    try:
        task_manager.normalize_route_yaml(
            """
waypoints:
  - id: same
    x: 0
    y: 0
  - id: same
    x: 1
    y: 1
"""
        )
    except RuntimeError as exc:
        assert "duplicate waypoint id" in str(exc)
    else:
        raise AssertionError("duplicate waypoint id was not rejected")


def test_route_asset_crud_round_trip(tmp_path):
    route_root = tmp_path / "routes"
    route_id = "lab_a"
    source_yaml = """
mission_name: lab_loop
waypoints:
  - id: a
    x: 1.0
    y: 2.0
    yaw: 0.0
"""

    saved_path, saved_yaml = task_manager.save_route(route_root, route_id, source_yaml)
    assert saved_path.exists()
    assert route_id in task_manager.list_routes(route_root)

    loaded_path, loaded_yaml = task_manager.load_route(route_root, route_id)
    assert loaded_path == saved_path
    assert loaded_yaml == saved_yaml

    deleted_path = task_manager.delete_route(route_root, route_id)
    assert deleted_path == saved_path
    assert task_manager.list_routes(route_root) == []


def test_build_auto_scan_command_uses_expected_ros_args(tmp_path):
    script_path = tmp_path / "auto_scan_mission.py"
    route_file = tmp_path / "route.yaml"
    command = task_manager.build_auto_scan_command(
        script_path,
        route_file,
        mission_name="lab",
        dry_run=True,
        stop_on_failure=False,
        save_map_on_finish=True,
        save_map_on_failure=False,
    )

    assert command[1] == str(script_path)
    assert f"waypoints_file:={route_file}" in command
    assert "mission_name:=lab" in command
    assert "dry_run:=true" in command
    assert "stop_on_failure:=false" in command
    assert "save_map_on_finish:=true" in command
