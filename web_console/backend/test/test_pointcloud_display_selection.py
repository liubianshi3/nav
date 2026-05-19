from __future__ import annotations

import sys
import types
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def _install_stub_modules() -> None:
    if "rclpy" not in sys.modules:
        rclpy = types.ModuleType("rclpy")
        rclpy.init = lambda *args, **kwargs: None
        rclpy.shutdown = lambda *args, **kwargs: None
        rclpy.spin_until_future_complete = lambda *args, **kwargs: None
        sys.modules["rclpy"] = rclpy

    if "rclpy.action" not in sys.modules:
        action_mod = types.ModuleType("rclpy.action")
        action_mod.ActionClient = type("ActionClient", (), {})
        sys.modules["rclpy.action"] = action_mod

    if "rclpy.executors" not in sys.modules:
        executors_mod = types.ModuleType("rclpy.executors")
        executors_mod.MultiThreadedExecutor = type("MultiThreadedExecutor", (), {})
        sys.modules["rclpy.executors"] = executors_mod

    if "rclpy.node" not in sys.modules:
        node_mod = types.ModuleType("rclpy.node")
        node_mod.Node = type("Node", (), {})
        sys.modules["rclpy.node"] = node_mod

    if "rclpy.qos" not in sys.modules:
        qos_mod = types.ModuleType("rclpy.qos")
        qos_mod.DurabilityPolicy = type("DurabilityPolicy", (), {"TRANSIENT_LOCAL": object(), "VOLATILE": object()})
        qos_mod.ReliabilityPolicy = type("ReliabilityPolicy", (), {"RELIABLE": object(), "BEST_EFFORT": object()})
        qos_mod.QoSProfile = type("QoSProfile", (), {"__init__": lambda self, *args, **kwargs: None})
        sys.modules["rclpy.qos"] = qos_mod

    def _register_message_module(module_name: str, names: list[str]) -> None:
        module = sys.modules.get(module_name)
        if module is None:
            module = types.ModuleType(module_name)
        for name in names:
            if not hasattr(module, name):
                setattr(module, name, type(name, (), {}))
        sys.modules[module_name] = module

    _register_message_module("action_msgs.msg", ["GoalStatus"])
    _register_message_module("geometry_msgs.msg", ["PoseStamped", "PoseWithCovarianceStamped", "Twist"])
    _register_message_module("nav_msgs.msg", ["OccupancyGrid", "Odometry"])
    _register_message_module("std_msgs.msg", ["Bool", "Float32", "Int32", "String"])
    _register_message_module("tf2_msgs.msg", ["TFMessage"])
    _register_message_module("sensor_msgs.msg", ["BatteryState", "CompressedImage", "Image", "PointCloud2"])


_install_stub_modules()

from backend.models import PointCloudSnapshot
from backend.ros_bridge import _select_display_pointcloud_snapshot


def _snapshot(topic: str, stamp: str) -> PointCloudSnapshot:
    return PointCloudSnapshot(
        loaded=True,
        source_topic=topic,
        stamp=stamp,
        points=[[1.0, 2.0, 0.0]],
        points_total=1,
        points_sampled=1,
        sample_stride=1,
    )


def test_display_pointcloud_prefers_fresh_preview_map_topic_over_fallback() -> None:
    selected = _select_display_pointcloud_snapshot(
        snapshots_by_topic={
            "/jt128/dlio/map_points_preview": _snapshot("/jt128/dlio/map_points_preview", "t1"),
            "/jt128/front/points_preview": _snapshot("/jt128/front/points_preview", "t3"),
        },
        timestamps_by_topic={
            "/jt128/dlio/map_points_preview": 1.0,
            "/jt128/front/points_preview": 6.0,
        },
        primary_topic="/jt128/dlio/map_points_preview",
        map_topics=["/jt128/dlio/map_points_preview"],
        fallback_topic="/jt128/front/points_preview",
        stale_sec=2.0,
        now=2.0,
    )

    assert selected is not None
    assert selected.source_topic == "/jt128/dlio/map_points_preview"


def test_display_pointcloud_ignores_non_preview_topics_for_web() -> None:
    selected = _select_display_pointcloud_snapshot(
        snapshots_by_topic={
            "/jt128/dlio/map_points": _snapshot("/jt128/dlio/map_points", "raw-map"),
            "/a2/map/pointcloud_3d": _snapshot("/a2/map/pointcloud_3d", "raw-web-map"),
            "/jt128/front/points": _snapshot("/jt128/front/points", "raw-fallback"),
        },
        timestamps_by_topic={
            "/jt128/dlio/map_points": 5.0,
            "/a2/map/pointcloud_3d": 5.0,
            "/jt128/front/points": 5.0,
        },
        primary_topic="/jt128/dlio/map_points",
        map_topics=["/jt128/dlio/map_points", "/a2/map/pointcloud_3d"],
        fallback_topic="/jt128/front/points",
        stale_sec=2.0,
        now=5.0,
    )

    assert selected is None


def test_display_pointcloud_uses_raw_fallback_only_until_a_map_topic_is_available() -> None:
    selected = _select_display_pointcloud_snapshot(
        snapshots_by_topic={
            "/jt128/front/points_preview": _snapshot("/jt128/front/points_preview", "t1"),
        },
        timestamps_by_topic={
            "/jt128/front/points_preview": 1.0,
        },
        primary_topic="/jt128/dlio/map_points_preview",
        map_topics=["/jt128/dlio/map_points_preview"],
        fallback_topic="/jt128/front/points_preview",
        stale_sec=2.0,
        now=1.5,
    )

    assert selected is not None
    assert selected.source_topic == "/jt128/front/points_preview"


def test_display_pointcloud_returns_to_standby_preview_when_map_topics_go_stale() -> None:
    selected = _select_display_pointcloud_snapshot(
        snapshots_by_topic={
            "/jt128/dlio/map_points_preview": _snapshot("/jt128/dlio/map_points_preview", "mapping-old"),
            "/jt128/front/points_preview": _snapshot("/jt128/front/points_preview", "standby-live"),
        },
        timestamps_by_topic={
            "/jt128/dlio/map_points_preview": 1.0,
            "/jt128/front/points_preview": 8.0,
        },
        primary_topic="/jt128/dlio/map_points_preview",
        map_topics=["/jt128/dlio/map_points_preview"],
        fallback_topic="/jt128/front/points_preview",
        stale_sec=2.0,
        now=8.5,
    )

    assert selected is not None
    assert selected.source_topic == "/jt128/front/points_preview"
