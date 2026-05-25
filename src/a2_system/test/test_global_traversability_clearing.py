#!/usr/bin/env python3
"""Verify stale obstacle clearing behaviour in GlobalTraversabilityMemory.

Run without ROS:
  python3 -m pytest test_global_traversability_clearing.py -v
"""

import sys
import time
import unittest
from pathlib import Path

_WS = Path("/home/unitree/ws/device-navigation")
sys.path.insert(0, str(_WS / "src" / "a2_system" / "scripts"))

from global_traversability_integrator import (
    GlobalTraversabilityMemory,
    _make_empty_costmap,
    _make_empty_pointcloud,
)

# Force stale_clear_sec to a tiny value so tests run fast
class FastClearingMemory(GlobalTraversabilityMemory):
    def __init__(self, **kwargs):
        kwargs.setdefault("stale_clear_sec", 0.01)
        kwargs.setdefault("observation_decay_sec", 0.005)
        kwargs.setdefault("min_observations", 1)
        kwargs.setdefault("min_confidence", 0.1)
        kwargs.setdefault("confidence_increment", 0.5)
        kwargs.setdefault("confidence_decay", 0.5)
        super().__init__(**kwargs)


class TestStaleClearing(unittest.TestCase):
    def test_apply_decay_removes_stale_cells(self):
        mem = FastClearingMemory()
        from nav_msgs.msg import OccupancyGrid, MapMetaData
        from geometry_msgs.msg import Pose
        import numpy as np

        info = MapMetaData(resolution=0.1, width=3, height=3)
        info.origin = Pose()
        info.origin.orientation.w = 1.0
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info = info
        grid.data = np.array([0, 0, 0, 0, 99, 0, 0, 0, 0], dtype=np.int8).tolist()
        mem.update(grid, 0.0, 0.0)
        mem.apply_decay()

        stable = mem.get_stable_obstacle_cells()
        self.assertGreater(len(stable), 0, "fresh high-cost cell must be stable")

        time.sleep(0.02)
        mem.apply_decay()
        stable2 = mem.get_stable_obstacle_cells()
        self.assertEqual(len(stable2), 0, "stale cells must be cleared after stale_clear_sec")

    def test_empty_costmap_when_no_stable_cells(self):
        mem = FastClearingMemory()
        from nav_msgs.msg import OccupancyGrid, MapMetaData
        from geometry_msgs.msg import Pose
        import numpy as np

        info = MapMetaData(resolution=0.1, width=3, height=3)
        info.origin = Pose()
        info.origin.orientation.w = 1.0
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info = info
        grid.data = np.array([0, 0, 0, 0, 99, 0, 0, 0, 0], dtype=np.int8).tolist()
        mem.update(grid, 0.0, 0.0)

        # Before staleness: should have stable cells
        cm_before = mem.build_stable_costmap()
        self.assertIsNotNone(cm_before)
        self.assertTrue(any(v > 0 for v in cm_before.data), "costmap must have obstacles")

        # After staleness: costmap must be all zeros
        time.sleep(0.02)
        mem.apply_decay()
        cm_after = mem.build_stable_costmap()
        self.assertIsNotNone(cm_after)
        self.assertTrue(all(v == 0 for v in cm_after.data), "stale costmap must be all zeros")


class TestErrorRecoveryPublishesEmptyOutput(unittest.TestCase):
    def test_empty_pointcloud_is_sized_zero(self):
        import rclpy
        rclpy.init(args=[])
        try:
            node = rclpy.create_node("_test_empty_pub")
            stamp = node.get_clock().now().to_msg()
            cloud = _make_empty_pointcloud(stamp, "map")
            self.assertEqual(cloud.width, 0, "empty cloud must have width=0")
            self.assertEqual(cloud.height, 1)
            self.assertTrue(cloud.is_dense)
        finally:
            rclpy.shutdown()

    def test_empty_costmap_has_one_cell(self):
        cm = _make_empty_costmap("map")
        self.assertEqual(len(cm.data), 1, "empty costmap must have exactly 1 cell")
        self.assertEqual(cm.data[0], 0, "empty costmap single cell must be zero")
        self.assertEqual(cm.header.frame_id, "map")


class TestConfigLayerIsStaticLayer(unittest.TestCase):
    def test_global_traversability_layer_is_opt_in_static_layer(self):
        import yaml
        path = _WS / "src" / "a2_system" / "config" / "nav2_3d.yaml"
        self.assertTrue(path.exists(), f"nav2_3d.yaml not found: {path}")
        config = yaml.safe_load(path.read_text())
        gc = config.get("global_costmap", {}).get("global_costmap", {}).get("ros__parameters", {})
        plugins = gc.get("plugins", [])
        self.assertNotIn(
            "global_traversability_layer",
            plugins,
            "default global_costmap plugins must not instantiate optional traversability layer",
        )
        tl = gc.get("global_traversability_layer", {})
        self.assertEqual(
            tl.get("plugin"),
            "nav2_costmap_2d::StaticLayer",
            "global_traversability_layer must use StaticLayer when enabled",
        )
        self.assertIn(
            "subscribe_to_updates",
            tl,
            "StaticLayer must have subscribe_to_updates",
        )
        self.assertEqual(
            tl.get("topic"),
            "/a2/global_traversability/costmap",
            "StaticLayer must subscribe to costmap topic not PointCloud2",
        )

    def test_no_obstacle_layer_observation_sources(self):
        import yaml
        path = _WS / "src" / "a2_system" / "config" / "nav2_3d.yaml"
        config = yaml.safe_load(path.read_text())
        tl = config.get("global_costmap", {}).get("global_costmap", {}).get("ros__parameters", {}).get("global_traversability_layer", {})
        self.assertNotIn("observation_sources", tl,
                         "StaticLayer must not have observation_sources (ObstacleLayer remnant)")


if __name__ == "__main__":
    unittest.main()
