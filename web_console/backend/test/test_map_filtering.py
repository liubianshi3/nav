from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from backend.config import AppConfig
from backend.stack_control import StackControlError, StackController


def _build_controller(tmp_path: Path) -> StackController:
    workspace = tmp_path / "ws"
    map_root = workspace / "runtime" / "maps"
    config_dir = workspace / "src" / "a2_system" / "config"
    config_dir.mkdir(parents=True, exist_ok=True)
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
    start_script = workspace / "install" / "a2_system" / "share" / "a2_system" / "start_jt128_3d_stack.sh"
    stop_script = workspace / "install" / "a2_system" / "share" / "a2_system" / "stop_jt128_stack.sh"
    start_script.parent.mkdir(parents=True, exist_ok=True)
    start_script.write_text("#!/bin/bash\nexit 0\n", encoding="utf-8")
    stop_script.write_text("#!/bin/bash\nexit 0\n", encoding="utf-8")

    config = AppConfig()
    config.stack.workspace = str(workspace)
    config.stack.map_root = str(map_root)
    config.stack.start_script = str(start_script)
    config.stack.stop_script = str(stop_script)
    return StackController(config)


def test_list_maps_hides_legacy_2d_maps_when_navigation_requires_3d(tmp_path):
    controller = _build_controller(tmp_path)
    legacy_dir = controller.map_root / "legacy_2d"
    modern_dir = controller.map_root / "modern_3d"
    legacy_dir.mkdir(parents=True, exist_ok=True)
    modern_dir.mkdir(parents=True, exist_ok=True)

    (legacy_dir / "map.yaml").write_text("image: map.pgm\nresolution: 0.1\n", encoding="utf-8")
    (legacy_dir / "metadata.yaml").write_text(
        yaml.safe_dump({"created_at": "2026-04-20T00:00:00", "mode": "mapping"}, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    (modern_dir / "metadata.yaml").write_text(
        yaml.safe_dump(
            {
                "created_at": "2026-04-29T00:00:00",
                "representation": "pointcloud_map_3d",
                "artifacts": [{"kind": "native_pointcloud_map_3d", "path": "native_map.pcd"}],
            },
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )

    visible = controller.list_maps()
    all_maps = controller.list_maps(include_incompatible=True)

    assert [item.map_id for item in visible] == ["modern_3d"]
    assert sorted(item.map_id for item in all_maps) == ["legacy_2d", "modern_3d"]
    legacy = next(item for item in all_maps if item.map_id == "legacy_2d")
    assert legacy.navigation_compatible is False
    assert legacy.navigation_compatibility_reason is not None


def test_status_keeps_incompatible_maps_visible_for_web_ui(tmp_path):
    controller = _build_controller(tmp_path)
    controller._process_records = lambda: []  # type: ignore[method-assign]
    legacy_dir = controller.map_root / "legacy_2d"
    modern_dir = controller.map_root / "modern_3d"
    legacy_dir.mkdir(parents=True, exist_ok=True)
    modern_dir.mkdir(parents=True, exist_ok=True)

    (legacy_dir / "map.yaml").write_text("image: map.pgm\nresolution: 0.1\n", encoding="utf-8")
    (legacy_dir / "metadata.yaml").write_text(
        yaml.safe_dump({"created_at": "2026-04-20T00:00:00", "mode": "mapping"}, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    (modern_dir / "metadata.yaml").write_text(
        yaml.safe_dump(
            {
                "created_at": "2026-04-29T00:00:00",
                "representation": "pointcloud_map_3d",
                "artifacts": [{"kind": "native_pointcloud_map_3d", "path": "native_map.pcd"}],
            },
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )

    status_maps = controller.status().maps

    assert sorted(item.map_id for item in status_maps) == ["legacy_2d", "modern_3d"]
    legacy = next(item for item in status_maps if item.map_id == "legacy_2d")
    assert legacy.navigation_compatible is False
    assert legacy.navigation_compatibility_reason is not None


def test_start_navigation_rejects_incompatible_legacy_2d_map(tmp_path):
    controller = _build_controller(tmp_path)
    legacy_dir = controller.map_root / "legacy_2d"
    legacy_dir.mkdir(parents=True, exist_ok=True)
    (legacy_dir / "map.yaml").write_text("image: map.pgm\nresolution: 0.1\n", encoding="utf-8")
    (legacy_dir / "metadata.yaml").write_text(
        yaml.safe_dump({"created_at": "2026-04-20T00:00:00"}, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )

    try:
        controller.start_navigation("legacy_2d")
    except StackControlError as exc:
        assert "3D 点云地图" in str(exc)
    else:
        raise AssertionError("legacy 2D map should be rejected for 3D-only navigation")


def test_start_navigation_materializes_native_pcd_and_projects_nav2_map(tmp_path, monkeypatch):
    controller = _build_controller(tmp_path)
    workspace = Path(controller.config.stack.workspace)
    tool_path = workspace / "install" / "a2_system" / "lib" / "a2_system" / "pcd_to_2d_map.py"
    tool_path.parent.mkdir(parents=True, exist_ok=True)
    tool_path.write_text(
        "\n".join(
            [
                "from pathlib import Path",
                "import sys",
                "out = Path(sys.argv[sys.argv.index('--output') + 1])",
                "out.mkdir(parents=True, exist_ok=True)",
                "(out / 'map.pgm').write_text('P2\\n1 1\\n255\\n0\\n', encoding='utf-8')",
                "(out / 'map.yaml').write_text('image: map.pgm\\nresolution: 0.05\\norigin: [0, 0, 0]\\n', encoding='utf-8')",
            ]
        ),
        encoding="utf-8",
    )

    map_id = "zbe-2_map_20260519_2042"
    map_dir = controller.map_root / map_id
    map_dir.mkdir(parents=True, exist_ok=True)
    (map_dir / "native_map.pcd").write_text("VERSION .7\nDATA ascii\n", encoding="utf-8")
    (map_dir / "metadata.yaml").write_text(
        yaml.safe_dump(
            {
                "created_at": "2026-05-19T20:42:00",
                "representation": "pointcloud_map_3d",
                "artifacts": [{"kind": "native_pointcloud_map_3d", "path": "native_map.pcd"}],
            },
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )

    commands: list[list[str]] = []
    monkeypatch.setattr(
        controller,
        "_run",
        lambda command, env=None: commands.append(command) or SimpleNamespace(stdout="ok"),
    )
    monkeypatch.setattr(controller, "_process_records", lambda: [])
    monkeypatch.setattr(controller, "_wait_for_expected_nodes", lambda *args, **kwargs: None)

    controller.start_navigation(map_id, enable_nav2_3d=True)

    assert (map_dir / "pointcloud_map_3d.pcd").exists()
    assert (map_dir / "map.yaml").exists()
    assert commands and "--enable-nav2-3d" in commands[0]


def test_expected_navigation_nodes_respects_explicit_3d_flag(tmp_path):
    controller = _build_controller(tmp_path)

    nodes = controller._expected_nodes_for_mode("navigation", use_3d_navigation=True)

    assert any(pattern == "planner_server" for _, _, pattern in nodes)
    assert any(pattern == "bt_navigator" for _, _, pattern in nodes)
