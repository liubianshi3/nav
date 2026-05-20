from __future__ import annotations

import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from backend.config import AppConfig
from backend.stack_control import ProcessInfo, StackController


def _build_controller(tmp_path: Path, start_script_name: str = "start_jt128_3d_stack.sh") -> StackController:
    workspace = tmp_path / "ws"
    map_root = workspace / "runtime" / "maps"
    config_dir = workspace / "src" / "a2_system" / "config"
    script_dir = workspace / "src" / "a2_system" / "tools"
    config_dir.mkdir(parents=True, exist_ok=True)
    script_dir.mkdir(parents=True, exist_ok=True)
    (config_dir / "slam.yaml").write_text(
        yaml.safe_dump(
            {
                "slam_manager": {
                    "ros__parameters": {
                        "navigation_representation": "pointcloud_map_3d",
                    }
                }
            },
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    start_script = script_dir / start_script_name
    stop_script = script_dir / "stop_jt128_stack.sh"
    start_script.write_text("#!/bin/bash\nexit 0\n", encoding="utf-8")
    stop_script.write_text("#!/bin/bash\nexit 0\n", encoding="utf-8")

    config = AppConfig()
    config.stack.workspace = str(workspace)
    config.stack.map_root = str(map_root)
    config.stack.start_script = str(start_script)
    config.stack.stop_script = str(stop_script)
    return StackController(config)


def _proc(args: str, pid: int) -> ProcessInfo:
    return ProcessInfo(pid=pid, ppid=1, stat="S", args=args)


def test_jt128_wrapper_mapping_does_not_require_octomap_nodes(tmp_path):
    controller = _build_controller(tmp_path, "start_jt128_3d_stack.sh")

    expected_patterns = {pattern for _, _, pattern in controller._expected_nodes_for_mode("mapping")}

    assert "map_manager_node" in expected_patterns
    assert "octomap_mapping_node.py" not in expected_patterns
    assert "octomap_server_node" not in expected_patterns


def test_direct_dlio_mapping_keeps_octomap_required_by_default(tmp_path):
    controller = _build_controller(tmp_path, "start_jt128_dlio_mapping.sh")

    expected_patterns = {pattern for _, _, pattern in controller._expected_nodes_for_mode("mapping")}

    assert "octomap_mapping_node.py" in expected_patterns
    assert "octomap_server_node" in expected_patterns


def test_status_promotes_no_octomap_mapping_start_once_required_nodes_run(tmp_path, monkeypatch):
    controller = _build_controller(tmp_path, "start_jt128_3d_stack.sh")
    controller._write_runtime_state(mode="starting", target_mode="mapping", message="建图模式启动中")
    processes = [
        _proc("ros2 run jt128_hesai_driver hesai_ros_driver_node", 1001),
        _proc("ros2 run jt128_dlio_odom dlio_odom_node", 1002),
        _proc("ros2 run jt128_dlio_map dlio_map_node", 1003),
        _proc("python3 pointcloud_preview_node.py", 1004),
        _proc("ros2 run map_manager map_manager_node", 1005),
    ]
    monkeypatch.setattr(controller, "_process_records", lambda: processes)

    status = controller.status()

    assert status.mode == "mapping"
    assert status.message is None
    assert {node.key for node in status.nodes} == {
        "driver",
        "dlio_odom",
        "dlio_map",
        "pointcloud_preview",
        "map_manager",
    }
    assert all(node.running for node in status.nodes)
