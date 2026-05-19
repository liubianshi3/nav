from __future__ import annotations

import asyncio
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

    class _Twist:
        def __init__(self) -> None:
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

    def _register_message_module(module_name: str, names: list[str]) -> None:
        module = sys.modules.get(module_name)
        if module is None:
            module = types.ModuleType(module_name)
        for name in names:
            if not hasattr(module, name):
                setattr(module, name, type(name, (), {}))
        sys.modules[module_name] = module

    geometry_msgs = types.ModuleType("geometry_msgs.msg")
    geometry_msgs.PoseStamped = type("PoseStamped", (), {})
    geometry_msgs.PoseWithCovarianceStamped = type("PoseWithCovarianceStamped", (), {})
    geometry_msgs.Twist = _Twist
    sys.modules["geometry_msgs.msg"] = geometry_msgs

    _register_message_module("action_msgs.msg", ["GoalStatus"])
    _register_message_module("nav_msgs.msg", ["OccupancyGrid", "Odometry"])
    _register_message_module("std_msgs.msg", ["Bool", "Float32", "Int32", "String"])
    _register_message_module("tf2_msgs.msg", ["TFMessage"])
    _register_message_module("sensor_msgs.msg", ["BatteryState", "CompressedImage", "Image", "PointCloud2"])


_install_stub_modules()

from backend.grpc_server import A2GrpcServices


class _LaserNavigationPb2:
    class PositionResponse:
        def __init__(self, **kwargs) -> None:
            self.__dict__.update(kwargs)

    class ScanDataResponse:
        def __init__(self, **kwargs) -> None:
            self.__dict__.update(kwargs)

    class StartNavigationResponse:
        def __init__(self, **kwargs) -> None:
            self.__dict__.update(kwargs)

    class SetInitialPoseResponse:
        def __init__(self, **kwargs) -> None:
            self.__dict__.update(kwargs)


class _FakeRuntime:
    def __init__(self, node) -> None:
        self.node = node
        self.thread = types.SimpleNamespace(is_alive=lambda: True)


class _FakeParent:
    def __init__(self, node, stack_controller=None) -> None:
        self.ros_runtime = _FakeRuntime(node)
        self.laser_navigation_pb2 = _LaserNavigationPb2
        self.stack_controller = stack_controller or _FakeStackController()

    async def _node_or_abort(self, context):
        if self.ros_runtime.node is None:
            await context.abort(None, "ROS runtime is not started")
        return self.ros_runtime.node


class _Context:
    async def abort(self, code, details):
        raise AssertionError(f"unexpected grpc abort: {code} {details}")


class _FakeStackController:
    def __init__(self) -> None:
        self.start_navigation_request = None

    def start_navigation_from_request(self, request):
        self.start_navigation_request = request
        return {"message": "navigation stack started"}


def _pointcloud(points, *, topic="/jt128/dlio/map_points", stamp="2026-05-18T08:00:00+00:00"):
    return types.SimpleNamespace(
        loaded=bool(points),
        source_topic=topic,
        stamp=stamp,
        points=points,
        points_total=len(points),
        points_sampled=len(points),
        sample_stride=1,
    )


class _FakeNode:
    def __init__(self) -> None:
        self.initial_pose_request = None
        self.raw_scan = _pointcloud([[9.0, 0.0, 0.0]], topic="/jt128/front/points")
        self.navigation_map = _pointcloud(
            [
                [1.0, 2.0, 0.5],
                [2.5, 3.5, 0.7],
                [4.0, 5.0, 1.2],
            ]
        )

    def build_snapshot(self, ros_thread_alive: bool = True):
        return types.SimpleNamespace(pointcloud=self.raw_scan)

    def get_navigation_pointcloud_snapshot(self):
        return self.navigation_map

    def set_initial_pose(self, request):
        self.initial_pose_request = request
        return {
            "message": "initial pose accepted",
            "pose": {"x": request.pose.x, "y": request.pose.y, "yaw": request.pose.yaw},
        }


def _service(node: _FakeNode):
    return A2GrpcServices._LaserNavigationService(_FakeParent(node))


def test_start_navigation_uses_full_stack_request_from_grpc() -> None:
    node = _FakeNode()
    stack_controller = _FakeStackController()
    service = A2GrpcServices._LaserNavigationService(_FakeParent(node, stack_controller))

    response = asyncio.run(
        service.StartNavigation(
            types.SimpleNamespace(
                device_id="laser",
                map_id="factory-floor",
                localization_mode="ndt",
                motion_mode="live_motion",
                enable_nav2_3d=True,
                collision_monitor_profile="strict",
            ),
            _Context(),
        )
    )

    assert response.success is True
    assert response.current_map_id == "factory-floor"
    request = stack_controller.start_navigation_request
    assert request.map_id == "factory-floor"
    assert request.localization_mode == "ndt"
    assert request.motion_mode == "live_motion"
    assert request.enable_nav2_3d is True
    assert request.collision_monitor_profile == "strict"


def test_set_initial_pose_forwards_pose_and_map_id_to_ros_bridge() -> None:
    node = _FakeNode()
    response = asyncio.run(
        _service(node).SetInitialPose(
            types.SimpleNamespace(
                device_id="laser",
                map_id="factory-floor",
                x=1.25,
                y=-2.5,
                theta=0.75,
                frame_id="map",
            ),
            _Context(),
        )
    )

    assert response.success is True
    assert node.initial_pose_request.map_id == "factory-floor"
    assert node.initial_pose_request.pose.x == 1.25
    assert node.initial_pose_request.pose.y == -2.5
    assert node.initial_pose_request.pose.yaw == 0.75
    assert node.initial_pose_request.pose.frame_id == "map"


def test_get_scan_data_include_pcd_returns_accumulated_map_pcd() -> None:
    response = asyncio.run(
        _service(_FakeNode()).GetScanData(
            types.SimpleNamespace(device_id="laser", angle_min=-180, angle_max=180, include_pcd=True),
            _Context(),
        )
    )

    pcd = response.pcd_data.decode("ascii")
    assert response.points_count == 3
    assert response.ranges == []
    assert "FIELDS x y z" in pcd
    assert "POINTS 3" in pcd
    assert "1.000000 2.000000 0.500000" in pcd
    assert "4.000000 5.000000 1.200000" in pcd


def test_watch_scan_data_streams_accumulated_map_pcd() -> None:
    async def first_frame():
        stream = _service(_FakeNode()).WatchScanData(
            types.SimpleNamespace(device_id="laser", angle_min=-180, angle_max=180, include_pcd=True, interval_ms=100),
            _Context(),
        )
        return await anext(stream)

    response = asyncio.run(first_frame())

    pcd = response.pcd_data.decode("ascii")
    assert response.points_count == 3
    assert "FIELDS x y z" in pcd
    assert "POINTS 3" in pcd
