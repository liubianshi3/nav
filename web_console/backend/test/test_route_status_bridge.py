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
        qos_mod.DurabilityPolicy = type("DurabilityPolicy", (), {"TRANSIENT_LOCAL": object()})
        qos_mod.ReliabilityPolicy = type("ReliabilityPolicy", (), {"RELIABLE": object()})
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
    _register_message_module("std_msgs.msg", ["Bool", "String"])
    _register_message_module("tf2_msgs.msg", ["TFMessage"])
    _register_message_module("sensor_msgs.msg", ["CompressedImage", "Image", "PointCloud2"])


_install_stub_modules()

from backend.models import TaskRouteStatus, TextStatus
from backend.ros_bridge import RosBridgeNode


class _DummyLock:
    def __enter__(self):
        return None

    def __exit__(self, exc_type, exc, tb):
        return False


def test_task_route_status_falls_back_to_service_response_for_missing_fields():
    bridge = object.__new__(RosBridgeNode)
    bridge.status = types.SimpleNamespace(
        task_manager_status=TextStatus(
            raw="mode=real;state=ready;ready=true;reason=idle;route_state=running",
            mode="real",
            state="ready",
            ready=True,
            reason="idle",
            fields={"route_state": "running"},
        )
    )
    bridge._lock = _DummyLock()
    bridge._call_task_command = lambda **kwargs: types.SimpleNamespace(
        current_mode="navigation",
        active_map="plant_a",
        route_id="night_shift",
        route_path="/tmp/night_shift.yaml",
        report_path="/tmp/night_shift.md",
        mission_state="running",
    )

    status = RosBridgeNode.task_route_status(bridge)

    assert isinstance(status, TaskRouteStatus)
    assert status.current_mode == "navigation"
    assert status.active_map == "plant_a"
    assert status.route_id == "night_shift"
    assert status.route_path == "/tmp/night_shift.yaml"
    assert status.report_path == "/tmp/night_shift.md"
    assert status.route_state == "running"
