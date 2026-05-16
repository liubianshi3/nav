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

    sensor_msgs = sys.modules.get("sensor_msgs.msg")
    if sensor_msgs is None:
        sensor_msgs = types.ModuleType("sensor_msgs.msg")
        sys.modules["sensor_msgs.msg"] = sensor_msgs
    if not hasattr(sensor_msgs, "BatteryState"):
        sensor_msgs.BatteryState = type("BatteryState", (), {})
    sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_CHARGING = 1


_install_stub_modules()

from backend.models import BatterySnapshot
from backend.ros_bridge import RosBridgeNode


class _DummyLock:
    def __enter__(self):
        return None

    def __exit__(self, exc_type, exc, tb):
        return False


def _msg(*, percentage: float, present: bool, charging: bool, voltage: float = 29.4, health: int = 1):
    status = 1 if charging else 2
    return types.SimpleNamespace(
        percentage=percentage,
        present=present,
        voltage=voltage,
        power_supply_status=status,
        power_supply_health=health,
        header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=1, nanosec=0),
        ),
    )


def test_battery_percentage_normalizes_fraction_to_0_100():
    bridge = object.__new__(RosBridgeNode)
    bridge._lock = _DummyLock()
    bridge.battery = BatterySnapshot()
    bridge._publish = lambda *args, **kwargs: None

    RosBridgeNode._on_battery(bridge, _msg(percentage=0.85, present=True, charging=True))

    assert bridge.battery.available is True
    assert bridge.battery.percentage == 85.0
    assert bridge.battery.charging is True
    assert bridge.battery.health == 1


def test_battery_percentage_keeps_0_100_value():
    bridge = object.__new__(RosBridgeNode)
    bridge._lock = _DummyLock()
    bridge.battery = BatterySnapshot()
    bridge._publish = lambda *args, **kwargs: None

    RosBridgeNode._on_battery(bridge, _msg(percentage=85.0, present=True, charging=False))

    assert bridge.battery.available is True
    assert bridge.battery.percentage == 85.0
    assert bridge.battery.charging is False
