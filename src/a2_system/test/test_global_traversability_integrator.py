#!/usr/bin/env python3
"""Unit tests for GlobalTraversabilityMemory — temporal cooldown logic.

Run without ROS:
  python3 -m pytest src/a2_system/test/test_global_traversability_integrator.py -q
"""

import time
import math
import struct
import unittest

import numpy as np
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


def _make_grid(
    data_2d,
    resolution=0.1,
    origin_x=-0.5,
    origin_y=-0.5,
    frame_id="map",
):
    """Build an OccupancyGrid from a 2D list-of-lists."""
    rows = len(data_2d)
    cols = len(data_2d[0]) if rows else 0
    flat = []
    for row in data_2d:
        flat.extend(row)
    grid = OccupancyGrid()
    grid.header = Header(frame_id=frame_id)
    origin_pose = Pose()
    origin_pose.position.x = float(origin_x)
    origin_pose.position.y = float(origin_y)
    origin_pose.position.z = 0.0
    origin_pose.orientation.w = 1.0
    grid.info = MapMetaData(
        resolution=float(resolution),
        width=cols,
        height=rows,
        origin=origin_pose,
    )
    grid.data = flat
    return grid


# Import the pure-Python class under test (does not require ROS).
import sys
sys.path.insert(0, "src/a2_system/scripts")
from global_traversability_integrator import (
    GlobalTraversabilityMemory,
    validate_frame,
    should_update_with_tf,
)


class TestGlobalTraversabilityMemory(unittest.TestCase):
    def setUp(self):
        self.memory = GlobalTraversabilityMemory(
            high_cost_threshold=90,
            lethal_threshold=70,
            min_observations=3,
            min_confidence=0.6,
            confidence_increment=0.25,
            confidence_decay=0.10,
            observation_decay_sec=20.0,
            stale_clear_sec=60.0,
            unknown_policy="ignore",
            local_update_window_enabled=False,
        )

    def test_single_frame_high_cost_does_not_produce_obstacle(self):
        """A high-cost cell in a single frame must not be forwarded."""
        grid = _make_grid([[100]])
        self.memory.update(grid)
        self.memory.apply_decay()
        pts = self.memory.get_stable_obstacle_points()
        self.assertEqual(len(pts), 0)

    def test_consecutive_high_cost_produces_obstacle(self):
        """After min_observations (3) consecutive high-cost frames, obstacle is emitted."""
        grid = _make_grid([[100]])
        for _ in range(3):
            self.memory.update(grid)
        self.memory.apply_decay()
        pts = self.memory.get_stable_obstacle_points()
        self.assertGreaterEqual(len(pts), 1)

    def test_unknown_cells_ignored_by_default(self):
        """Unknown cells (value == -1) are ignored with unknown_policy=ignore."""
        grid = _make_grid([[-1, 100], [100, 100]])
        for _ in range(3):
            self.memory.update(grid)
        self.memory.apply_decay()
        pts = self.memory.get_stable_obstacle_points()
        # Should have at most 3 cells (excluding the unknown)
        self.assertLessEqual(len(pts), 3)

    def test_stale_cells_cleared(self):
        """Cells older than stale_clear_sec are removed."""
        memory = GlobalTraversabilityMemory(
            stale_clear_sec=0.01,
        )
        grid = _make_grid([[100]])
        for _ in range(5):
            memory.update(grid)
        memory.apply_decay()
        pts = memory.get_stable_obstacle_points()
        self.assertGreater(len(pts), 0)

        time.sleep(0.02)
        memory.apply_decay()
        pts = memory.get_stable_obstacle_points()
        self.assertEqual(len(pts), 0)

    def test_output_points_are_provided(self):
        """Points are emitted; frame_id is set by ROS node layer."""
        memory = GlobalTraversabilityMemory()
        grid = _make_grid([[100], [100]], frame_id="map")
        for _ in range(5):
            memory.update(grid)
        memory.apply_decay()
        pts = memory.get_stable_obstacle_points()
        self.assertGreater(len(pts), 0)
        self.assertGreater(memory.stats.stable_obstacle_cells, 0)

    def test_max_points_limit(self):
        """The max_points parameter caps the number of output points."""
        memory = GlobalTraversabilityMemory(
            local_update_window_enabled=False,
            max_points=2,
            min_observations=1,
            min_confidence=0.0,
            confidence_increment=1.0,
        )
        data = [[100] * 10 for _ in range(10)]
        grid = _make_grid(data, resolution=0.1)
        for _ in range(3):
            memory.update(grid)
        memory.apply_decay()
        pts = memory.get_stable_obstacle_points()
        self.assertLessEqual(len(pts), 2)

    def test_status_contains_expected_fields(self):
        """Status string includes required diagnostic fields."""
        memory = GlobalTraversabilityMemory()
        grid = _make_grid([[100]])
        for _ in range(5):
            memory.update(grid)
        memory.apply_decay()
        memory.get_stable_obstacle_points()
        status = memory.stats.status_string()
        self.assertIn("state=", status)
        self.assertIn("ready=", status)
        self.assertIn("reason=", status)
        self.assertIn("stable_obstacle_cells=", status)
        self.assertIn("dropped_unknown_cells=", status)

    def test_confidence_decay_over_time(self):
        """Confidence decays for cells not observed as high-cost."""
        memory = GlobalTraversabilityMemory(
            observation_decay_sec=0.01,
            min_observations=2,
            min_confidence=0.5,
            confidence_increment=0.5,
            confidence_decay=0.3,
        )
        grid = _make_grid([[100]])
        memory.update(grid)
        memory.update(grid)
        memory.apply_decay()
        pts = memory.get_stable_obstacle_points()
        self.assertGreater(len(pts), 0)

        # Now feed two frames with low cost
        grid_low = _make_grid([[0]])
        memory.update(grid_low)
        time.sleep(0.02)
        memory.apply_decay()
        memory.update(grid_low)
        time.sleep(0.02)
        memory.apply_decay()
        pts_after = memory.get_stable_obstacle_points()
        self.assertEqual(len(pts_after), 0)

    def test_local_update_window_filters_remote_cells(self):
        """Only cells within local_update_radius_m are processed."""
        memory = GlobalTraversabilityMemory(
            local_update_window_enabled=True,
            local_update_radius_m=1.0,
            min_observations=1,
            min_confidence=0.0,
            confidence_increment=1.0,
        )
        data = [[100] * 20 for _ in range(20)]
        grid = _make_grid(data, resolution=0.1, origin_x=-1.0, origin_y=-1.0)
        for _ in range(3):
            memory.update(grid, robot_x=0.0, robot_y=0.0)
        memory.apply_decay()
        pts = memory.get_stable_obstacle_points()
        # Only points within 1m of origin (robot) should be present
        for pt in pts:
            dist = math.sqrt(pt[0] ** 2 + pt[1] ** 2)
            self.assertLessEqual(dist, 1.0 + 0.2)  # tolerance

    def test_empty_grid_no_crash(self):
        """An empty occupancy grid should not cause an error."""
        grid = _make_grid([[]])
        self.memory.update(grid)
        self.memory.apply_decay()
        pts = self.memory.get_stable_obstacle_points()
        self.assertEqual(len(pts), 0)

    def test_unknown_policy_lethal_treats_unknown_as_obstacle(self):
        """With unknown_policy=lethal, unknown cells become obstacles."""
        memory = GlobalTraversabilityMemory(
            unknown_policy="lethal",
            min_observations=3,
            min_confidence=0.5,
            confidence_increment=0.5,
            local_update_window_enabled=False,
        )
        grid = _make_grid([[-1]])
        for _ in range(3):
            memory.update(grid)
        memory.apply_decay()
        pts = memory.get_stable_obstacle_points()
        self.assertGreater(len(pts), 0)

    def test_get_stable_obstacle_cells_returns_cellstate(self):
        """get_stable_obstacle_cells returns CellState objects with col, row."""
        memory = GlobalTraversabilityMemory(
            min_observations=2,
            min_confidence=0.5,
            confidence_increment=0.6,
            local_update_window_enabled=False,
        )
        grid = _make_grid([[100]])
        for _ in range(2):
            memory.update(grid)
        memory.apply_decay()
        cells = memory.get_stable_obstacle_cells()
        self.assertGreaterEqual(len(cells), 1)
        cell = cells[0]
        self.assertEqual(cell.col, 0)
        self.assertEqual(cell.row, 0)
        self.assertGreater(cell.high_cost_count, 0)

    def test_build_stable_costmap_output(self):
        """build_stable_costmap sets stable cells to 100, others to 0."""
        memory = GlobalTraversabilityMemory(
            min_observations=2,
            min_confidence=0.5,
            confidence_increment=0.6,
            local_update_window_enabled=False,
        )
        grid = _make_grid([[0, 0], [0, 100]])
        for _ in range(3):
            memory.update(grid)
        memory.apply_decay()
        costmap = memory.build_stable_costmap()
        self.assertIsNotNone(costmap)
        self.assertEqual(costmap.header.frame_id, "map")
        data = list(costmap.data)
        # Cell (1,0)=100 is index 1*2+0=2, Cell (1,1)=100 is index 1*2+1=3
        # Wait - the 100 cell is at col 1, row 1 (since python list: grid[1][1]=100)
        # Flattened row-major: row=1, col=1 → index=1*2+1=3
        # The grid was: [[0,0],[0,100]] i.e. row0=[0,0], row1=[0,100]
        # So cell (row=1, col=1) with value 100
        self.assertEqual(data[3], 100)
        # Other cells should be 0
        self.assertEqual(data[0], 0)
        self.assertEqual(data[1], 0)
        self.assertEqual(data[2], 0)

    def test_build_stable_costmap_no_grid(self):
        """build_stable_costmap returns None when no grid received."""
        memory = GlobalTraversabilityMemory()
        self.assertIsNone(memory.build_stable_costmap())


class TestValidateFrame(unittest.TestCase):
    def test_empty_frame(self):
        ok, reason = validate_frame("", "map")
        self.assertFalse(ok)
        self.assertIn("empty", reason)

    def test_match(self):
        ok, reason = validate_frame("map", "map")
        self.assertTrue(ok)

    def test_mismatch(self):
        ok, reason = validate_frame("base_link", "map")
        self.assertFalse(ok)
        self.assertIn("mismatch", reason)

    def test_whitespace_tolerance(self):
        ok, _ = validate_frame("  map  ", "map")
        self.assertTrue(ok)


class TestShouldUpdateWithTf(unittest.TestCase):
    def test_local_window_off_always_updates(self):
        should, _ = should_update_with_tf(False, False)
        self.assertTrue(should)

    def test_local_window_on_tf_ok_updates(self):
        should, _ = should_update_with_tf(True, True)
        self.assertTrue(should)

    def test_local_window_on_tf_fail_blocks(self):
        should, reason = should_update_with_tf(True, False)
        self.assertFalse(should)
        self.assertIn("waiting_tf", reason)


if __name__ == "__main__":
    unittest.main()
