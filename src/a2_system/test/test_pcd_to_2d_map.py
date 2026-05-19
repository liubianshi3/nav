"""Lightweight tests for pcd_to_2d_map self-clear / near-origin behaviour.

These tests cover the geometry helpers added for the static map / corridor gate
fix:

* ``clear_disk_around_world_point`` clears a disk in PGM-row order.
* ``ignore_obstacles_within_radius`` keeps near-origin self-shell points out of
  the obstacle classifier without marking those cells occupied.
* The CLI default for ``--dilate`` is ``0`` so we no longer double-inflate
  against Nav2 costmap inflation.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def _load_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "pcd_to_2d_map.py"
    spec = importlib.util.spec_from_file_location("pcd_to_2d_map_under_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


pcd_to_2d_map = _load_module()


def test_clear_disk_around_world_point_clears_correct_cells():
    width, height = 5, 5
    resolution = 1.0
    origin_x, origin_y = -2.5, -2.5
    # Initialise as all occupied; helper must clear the disk only.
    grid = [100] * (width * height)

    cleared = pcd_to_2d_map.clear_disk_around_world_point(
        grid, width, height, origin_x, origin_y, resolution,
        center_x=0.0, center_y=0.0, radius=1.0,
    )

    # World center cell (0, 0) sits at PGM row=2, col=2 (height-1-world_r),
    # and a 1.0 m disk at 1.0 m resolution covers the 4-neighbours but not
    # the diagonals (distance sqrt(2) > 1.0).
    cleared_indices = {idx for idx, value in enumerate(grid) if value == 0}
    expected = {
        2 * width + 2,  # (0, 0)
        2 * width + 1,  # (-1, 0)
        2 * width + 3,  # (1, 0)
        1 * width + 2,  # (0, 1)  -- world_r=3, pgm_row=1
        3 * width + 2,  # (0, -1) -- world_r=1, pgm_row=3
    }
    assert cleared_indices == expected
    assert cleared == len(expected)


def test_clear_disk_zero_radius_is_noop():
    grid = [100, 100, 100, 100]
    cleared = pcd_to_2d_map.clear_disk_around_world_point(
        grid, width=2, height=2, origin_x=0.0, origin_y=0.0, resolution=0.5,
        center_x=0.5, center_y=0.5, radius=0.0,
    )
    assert cleared == 0
    assert grid == [100, 100, 100, 100]


def test_ignore_obstacles_within_radius_keeps_near_origin_points_out_of_obstacle_class():
    # Two stacks of obstacle-z points: one tight cluster at the origin (would
    # be counted as a self-shell) and one cluster far away that should still
    # be classified as occupied so the projection has a well-defined bounds.
    near_x, near_y = 0.05, 0.0
    far_x, far_y = 5.0, 0.0
    z_obstacle = 0.5  # within ground/ceiling range
    xs = [near_x] * 10 + [far_x] * 10
    ys = [near_y] * 10 + [far_y] * 10
    zs = [z_obstacle] * 20

    # Without the ignore radius the near cluster would mark itself occupied.
    grid_keep, ox_keep, oy_keep, w_keep, h_keep, _ = pcd_to_2d_map.project(
        xs, ys, zs,
        resolution=0.1,
        ground_threshold=0.08,
        ceiling_threshold=2.0,
        min_obstacle_points=2,
        min_ground_points=1,
        border_padding_m=0.5,
        dilate_radius_cells=0,
        ignore_obstacles_within_radius=0.0,
        clear_radius_around_origin=0.0,
    )
    near_col = int((near_x - ox_keep) / 0.1)
    near_world_r = int((near_y - oy_keep) / 0.1)
    near_pgm_row = h_keep - 1 - near_world_r
    assert grid_keep[near_pgm_row * w_keep + near_col] == 100

    # With the ignore radius the same near cluster must NOT be occupied.
    grid_drop, ox_drop, oy_drop, w_drop, h_drop, _ = pcd_to_2d_map.project(
        xs, ys, zs,
        resolution=0.1,
        ground_threshold=0.08,
        ceiling_threshold=2.0,
        min_obstacle_points=2,
        min_ground_points=1,
        border_padding_m=0.5,
        dilate_radius_cells=0,
        ignore_obstacles_within_radius=0.45,
        clear_radius_around_origin=0.0,
    )
    near_col_d = int((near_x - ox_drop) / 0.1)
    near_world_r_d = int((near_y - oy_drop) / 0.1)
    near_pgm_row_d = h_drop - 1 - near_world_r_d
    assert grid_drop[near_pgm_row_d * w_drop + near_col_d] != 100

    # The far cluster must still be marked occupied; the ignore radius must
    # not silently drop real obstacles.
    far_col = int((far_x - ox_drop) / 0.1)
    far_world_r = int((far_y - oy_drop) / 0.1)
    far_pgm_row = h_drop - 1 - far_world_r
    assert grid_drop[far_pgm_row * w_drop + far_col] == 100


def test_clear_radius_around_origin_marks_disk_free():
    # Build a tiny scene with a single far-away obstacle, then ask the
    # projector to clear a disk around the world origin. Cells inside the disk
    # must become free, including ones that started as unknown.
    # Two obstacles: one near the origin (will get cleared) and one far away
    # (must stay occupied). The near obstacle also makes sure the projector's
    # bounding box includes the origin cell.
    xs = [0.0, 5.0]
    ys = [0.0, 0.0]
    zs = [0.5, 0.5]

    grid, origin_x, origin_y, width, height, cleared = pcd_to_2d_map.project(
        xs, ys, zs,
        resolution=0.1,
        ground_threshold=0.08,
        ceiling_threshold=2.0,
        min_obstacle_points=1,
        min_ground_points=1,
        border_padding_m=1.0,
        dilate_radius_cells=0,
        ignore_obstacles_within_radius=0.0,
        clear_radius_around_origin=0.45,
    )
    assert cleared > 0
    # The world (0, 0) cell must be free.
    col = int((0.0 - origin_x) / 0.1)
    world_r = int((0.0 - origin_y) / 0.1)
    pgm_row = height - 1 - world_r
    assert grid[pgm_row * width + col] == 0


def test_clear_radius_with_ignore_keeps_origin_in_grid_and_free():
    # This is the live-command shape we actually recommend: a tight near-origin
    # self-shell that gets ignored, plus a single real obstacle far away. The
    # origin must still be inside the projected map and explicitly free.
    near_radius = 0.05  # well inside ignore radius
    far_x = 5.0
    z_obstacle = 0.5
    xs = [near_radius] * 20 + [far_x]
    ys = [0.0] * 20 + [0.0]
    zs = [z_obstacle] * 20 + [z_obstacle]

    grid, origin_x, origin_y, width, height, cleared = pcd_to_2d_map.project(
        xs, ys, zs,
        resolution=0.1,
        ground_threshold=0.08,
        ceiling_threshold=2.0,
        min_obstacle_points=2,
        min_ground_points=1,
        border_padding_m=0.5,
        dilate_radius_cells=0,
        ignore_obstacles_within_radius=0.45,
        clear_radius_around_origin=0.45,
    )

    # Origin must be inside the grid bounds.
    col = int((0.0 - origin_x) / 0.1)
    world_r = int((0.0 - origin_y) / 0.1)
    assert 0 <= col < width
    assert 0 <= world_r < height

    # And the origin cell must be free, with at least one cell cleared.
    pgm_row = height - 1 - world_r
    assert grid[pgm_row * width + col] == 0
    assert cleared > 0


def test_clear_world_point_marks_ndt_base_link_pose_free():
    # The navigation start pose is not necessarily the map origin after NDT
    # relocalization. Clearing only (0, 0) must not be mistaken for clearing the
    # robot's current map-frame base_link pose.
    start_x, start_y = 1.14, -2.76
    far_x, far_y = 5.0, 0.0
    xs = [start_x] * 4 + [far_x] * 4
    ys = [start_y] * 4 + [far_y] * 4
    zs = [0.5] * 8

    origin_only, ox, oy, width, height, _ = pcd_to_2d_map.project(
        xs, ys, zs,
        resolution=0.1,
        ground_threshold=0.08,
        ceiling_threshold=2.0,
        min_obstacle_points=2,
        min_ground_points=1,
        border_padding_m=0.5,
        dilate_radius_cells=0,
        ignore_obstacles_within_radius=0.0,
        clear_radius_around_origin=0.45,
    )
    col = int((start_x - ox) / 0.1)
    world_r = int((start_y - oy) / 0.1)
    pgm_row = height - 1 - world_r
    assert origin_only[pgm_row * width + col] == 100

    cleared_grid, ox2, oy2, width2, height2, cleared = pcd_to_2d_map.project(
        xs, ys, zs,
        resolution=0.1,
        ground_threshold=0.08,
        ceiling_threshold=2.0,
        min_obstacle_points=2,
        min_ground_points=1,
        border_padding_m=0.5,
        dilate_radius_cells=0,
        ignore_obstacles_within_radius=0.0,
        clear_radius_around_origin=0.0,
        clear_world_points=[(start_x, start_y, 0.45)],
    )
    col2 = int((start_x - ox2) / 0.1)
    world_r2 = int((start_y - oy2) / 0.1)
    pgm_row2 = height2 - 1 - world_r2
    assert cleared_grid[pgm_row2 * width2 + col2] == 0
    assert cleared > 0

    far_col = int((far_x - ox2) / 0.1)
    far_world_r = int((far_y - oy2) / 0.1)
    far_pgm_row = height2 - 1 - far_world_r
    assert cleared_grid[far_pgm_row * width2 + far_col] == 100


def test_cli_dilate_default_is_zero():
    # The CLI default must stay 0 so we do not reintroduce projection-stage
    # inflation on top of Nav2 costmap inflation.
    saved_argv = sys.argv
    try:
        sys.argv = ["pcd_to_2d_map.py", "/tmp/does_not_matter.pcd"]
        args = pcd_to_2d_map._cli_args()
    finally:
        sys.argv = saved_argv
    assert args.dilate == 0
    assert args.ignore_obstacles_within_radius == 0.0
    assert args.clear_radius_around_origin == 0.0
