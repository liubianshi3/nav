from __future__ import annotations

import importlib.util
from argparse import Namespace
from pathlib import Path


def load_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "octomap_to_2d_grid.py"
    spec = importlib.util.spec_from_file_location("octomap_to_2d_grid_under_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


octomap_to_2d_grid = load_module()


def test_helper_candidates_include_workspace_build_from_source_layout():
    script_path = Path("/home/unitree/ws/device-navigation/src/a2_system/scripts/octomap_to_2d_grid.py")

    candidates = octomap_to_2d_grid._helper_candidates(script_path, env_helper=None, path_helper=None)

    assert (
        "/home/unitree/ws/device-navigation/build/a2_system/octomap_to_2d_grid_cpp"
        in candidates
    )
    assert (
        "/home/unitree/ws/device-navigation/install/a2_system/lib/a2_system/octomap_to_2d_grid_cpp"
        in candidates
    )


def test_build_command_forwards_clear_world_points_to_cpp_helper():
    args = Namespace(
        octomap_path="/tmp/map.bt",
        output="/tmp/out",
        resolution=0.05,
        ground_threshold=0.10,
        robot_height=1.0,
        min_obstacle_points=2,
        border_padding=1.0,
        pcd_output="/tmp/out/pointcloud_map_3d.pcd",
        clear_world_point=["1.30,-0.16,0.85"],
    )

    cmd = octomap_to_2d_grid._build_command(args, "/tmp/octomap_to_2d_grid_cpp")

    assert "--clear-world-point" in cmd
    idx = cmd.index("--clear-world-point")
    assert cmd[idx + 1] == "1.30,-0.16,0.85"
