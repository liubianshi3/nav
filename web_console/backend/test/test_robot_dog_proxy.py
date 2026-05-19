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


class _Response:
    def __init__(self, **kwargs) -> None:
        self.__dict__.update(kwargs)


class _RobotDogPb2:
    POSTURE_TYPE_STAND = 1
    POSTURE_TYPE_SIT = 2
    POSTURE_TYPE_LIE = 3
    POSTURE_TYPE_STRETCH = 4
    MOTION_AUTHORIZATION_STATE_UNKNOWN = 1
    MOTION_AUTHORIZATION_STATE_STAND_DOWN = 2
    MOTION_AUTHORIZATION_STATE_STANDING_NOT_AUTHORIZED = 3
    MOTION_AUTHORIZATION_STATE_MANUAL_START_REQUIRED = 4
    MOTION_AUTHORIZATION_STATE_AUTHORIZED = 5
    MOTION_AUTHORIZATION_ACTION_NONE = 1
    MOTION_AUTHORIZATION_ACTION_STAND_UP = 2
    MOTION_AUTHORIZATION_ACTION_PRESS_REMOTE_START = 3
    MOTION_AUTHORIZATION_ACTION_STOP = 4

    StopResponse = _Response
    MoveResponse = _Response
    WalkResponse = _Response
    PostureResponse = _Response
    BalanceStandResponse = _Response
    StandUpResponse = _Response
    StandDownResponse = _Response
    RecoveryStandResponse = _Response
    DampResponse = _Response
    SetAutoRecoveryResponse = _Response
    SwitchGaitResponse = _Response
    SetSpeedLevelResponse = _Response
    SetBodyHeightResponse = _Response
    GetControlStateResponse = _Response
    AuthorizeMotionResponse = _Response
    ReleaseMotionAuthorizationResponse = _Response
    GetMotionAuthorizationResponse = _Response


class _Publisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


class _FakeNode:
    def __init__(self) -> None:
        self.cancel_stop_publisher = _Publisher()
        self.direct_cmd_publisher = _Publisher()
        self.manual_velocity_commands = []
        self.motion_commands = []
        self.control_state = types.SimpleNamespace(
            stamp="2026-05-14T08:00:00+00:00",
            runtime_mode="mock",
            state="ready",
            ready=True,
            reason="ok",
            interface_name="lo",
            gait_control_enabled=True,
            gait_type=2,
            speed_level=1,
            body_height=0.03,
            auto_recovery=True,
            last_command="stand_up",
            last_sdk_code=0,
            last_error_code="",
            last_error_reason="",
        )
        self.snapshot = types.SimpleNamespace(
            status=types.SimpleNamespace(
                raw_state=types.SimpleNamespace(
                    connected=True,
                    position=[0.0, 0.0, 0.55],
                    body_height=0.55,
                    motion_mode=0,
                    gait_type=1,
                )
            ),
            control_state=self.control_state,
        )

    def call_motion_command(self, command: str, int_value: int = 0, float_value: float = 0.0, bool_value: bool = False):
        self.motion_commands.append((command, int_value, float_value, bool_value))
        self.control_state.last_command = command
        if command == "stand_up":
            self.snapshot.status.raw_state.motion_mode = 2
        elif command == "balance_stand":
            self.snapshot.status.raw_state.motion_mode = 3
        elif command in {"stand_down", "damp"}:
            self.snapshot.status.raw_state.motion_mode = 5
        return types.SimpleNamespace(
            success=True,
            message=f"{command} ok",
            sdk_code=0,
            error_code="",
            runtime_mode="mock",
            state="ready",
        )

    def publish_manual_velocity(self, command):
        self.manual_velocity_commands.append(command)
        return types.SimpleNamespace(message="manual velocity ok")

    def build_snapshot(self, ros_thread_alive: bool = True):
        return self.snapshot


class _FakeRuntime:
    def __init__(self, node: _FakeNode) -> None:
        self.node = node
        self.thread = types.SimpleNamespace(is_alive=lambda: True)


class _FakeStackController:
    def __init__(self) -> None:
        self.manual_standby_calls = 0

    def ensure_manual_control_standby(self) -> None:
        self.manual_standby_calls += 1


class _Context:
    async def abort(self, code, details):
        raise AssertionError(f"unexpected grpc abort: {code} {details}")


class _FakeParent:
    def __init__(self, node: _FakeNode) -> None:
        self.ros_runtime = _FakeRuntime(node)
        self.stack_controller = _FakeStackController()
        self.robot_dog_pb2 = _RobotDogPb2
        self.robot_mode = {}

    async def _node_or_abort(self, context):
        if self.ros_runtime.node is None:
            await context.abort(None, "ROS runtime is not started")
        return self.ros_runtime.node


def _service(node: _FakeNode):
    return A2GrpcServices._RobotDogService(_FakeParent(node))


def test_robot_dog_proto_adds_explicit_motion_rpc_methods_without_generic_command() -> None:
    proto = Path(__file__).resolve().parents[3] / "proto/device/robot_dog.proto"
    source = proto.read_text(encoding="utf-8")

    for rpc_name in [
        "BalanceStand",
        "StandUp",
        "StandDown",
        "RecoveryStand",
        "Damp",
        "SetAutoRecovery",
        "SwitchGait",
        "SetSpeedLevel",
        "SetBodyHeight",
        "GetControlState",
        "AuthorizeMotion",
        "ReleaseMotionAuthorization",
        "GetMotionAuthorization",
    ]:
        assert f"rpc {rpc_name}(" in source
    assert "ExecuteMotionCommand" not in source


def test_robot_dog_proto_keeps_mode_separate_from_motion_authorization() -> None:
    proto = Path(__file__).resolve().parents[3] / "proto/device/robot_dog.proto"
    source = proto.read_text(encoding="utf-8")

    assert "enum RobotDogMode" in source
    assert "ROBOT_DOG_MODE_REMOTE_CONTROL" in source
    assert "ROBOT_DOG_MODE_API" in source
    assert "enum MotionAuthorizationState" in source
    assert "MOTION_AUTHORIZATION_STATE_MANUAL_START_REQUIRED" in source
    assert "MOTION_AUTHORIZATION_ACTION_PRESS_REMOTE_START" in source


def test_stop_publishes_zero_manual_velocity_without_motion_stop() -> None:
    node = _FakeNode()
    service = _service(node)

    response = asyncio.run(service.Stop(types.SimpleNamespace(device_id="a2", type=1), _Context()))

    assert response.success is True
    assert node.motion_commands == []
    assert len(node.cancel_stop_publisher.messages) == 0
    assert len(node.direct_cmd_publisher.messages) == 0
    assert len(node.manual_velocity_commands) == 1
    zero = node.manual_velocity_commands[0]
    assert zero.linear_x == 0.0
    assert zero.linear_y == 0.0
    assert zero.angular_z == 0.0
    assert service.p.stack_controller.manual_standby_calls == 0


def test_move_authorizes_motion_before_publishing_manual_velocity() -> None:
    node = _FakeNode()
    node.control_state.last_command = "stand_up"
    node.snapshot.status.raw_state.motion_mode = 2
    service = _service(node)

    response = asyncio.run(
        service.Move(
            types.SimpleNamespace(device_id="a2", velocity_x=0.2, velocity_y=0.0, angular_velocity=0.0),
            _Context(),
        )
    )

    assert response.success is True
    assert node.motion_commands == [("balance_stand", 0, 0.0, False)]
    assert len(node.manual_velocity_commands) == 1
    assert node.manual_velocity_commands[0].linear_x == 0.2


def test_move_requires_stand_up_before_authorizing_motion() -> None:
    node = _FakeNode()
    node.control_state.last_command = "stand_down"
    node.snapshot.status.raw_state.motion_mode = 5
    service = _service(node)

    response = asyncio.run(
        service.Move(
            types.SimpleNamespace(device_id="a2", velocity_x=0.2, velocity_y=0.0, angular_velocity=0.0),
            _Context(),
        )
    )

    assert response.success is False
    assert response.message == "stand up before requesting motion authorization"
    assert node.motion_commands == []
    assert node.manual_velocity_commands == []


def test_move_and_walk_publish_through_manual_control_contract() -> None:
    node = _FakeNode()
    service = _service(node)

    move = asyncio.run(
        service.Move(
            types.SimpleNamespace(device_id="a2", velocity_x=0.12, velocity_y=0.01, angular_velocity=0.03),
            _Context(),
        )
    )
    walk = asyncio.run(
        service.Walk(
            types.SimpleNamespace(device_id="a2", x=-0.05, y=0.02, theta=-0.04),
            _Context(),
        )
    )

    assert move.success is True
    assert walk.success is True
    assert len(node.cancel_stop_publisher.messages) == 0
    assert len(node.direct_cmd_publisher.messages) == 0
    assert len(node.manual_velocity_commands) == 2
    assert node.manual_velocity_commands[0].linear_x == 0.12
    assert node.manual_velocity_commands[0].linear_y == 0.01
    assert node.manual_velocity_commands[0].angular_z == 0.03
    assert node.manual_velocity_commands[1].linear_x == -0.05
    assert node.manual_velocity_commands[1].linear_y == 0.02
    assert node.manual_velocity_commands[1].angular_z == -0.04
    assert node.motion_commands == [("balance_stand", 0, 0.0, False)]
    assert service.p.stack_controller.manual_standby_calls == 2


def test_posture_maps_stand_and_lie_to_motion_service() -> None:
    node = _FakeNode()
    service = _service(node)

    stand = asyncio.run(
        service.SetPosture(
            types.SimpleNamespace(device_id="a2", posture=_RobotDogPb2.POSTURE_TYPE_STAND, duration_ms=0),
            _Context(),
        )
    )
    lie = asyncio.run(
        service.SetPosture(
            types.SimpleNamespace(device_id="a2", posture=_RobotDogPb2.POSTURE_TYPE_LIE, duration_ms=0),
            _Context(),
        )
    )

    assert stand.success is True
    assert lie.success is True
    assert node.motion_commands == [
        ("stand_up", 0, 0.0, False),
        ("stand_down", 0, 0.0, False),
    ]


def test_explicit_motion_rpcs_call_named_motion_commands() -> None:
    node = _FakeNode()
    service = _service(node)
    cases = [
        ("BalanceStand", types.SimpleNamespace(device_id="a2"), ("balance_stand", 0, 0.0, False)),
        ("StandUp", types.SimpleNamespace(device_id="a2"), ("stand_up", 0, 0.0, False)),
        ("StandDown", types.SimpleNamespace(device_id="a2"), ("stand_down", 0, 0.0, False)),
        ("RecoveryStand", types.SimpleNamespace(device_id="a2"), ("recovery_stand", 0, 0.0, False)),
        ("Damp", types.SimpleNamespace(device_id="a2"), ("damp", 0, 0.0, False)),
        ("SetAutoRecovery", types.SimpleNamespace(device_id="a2", enabled=True), ("set_auto_recovery", 0, 0.0, True)),
        ("SwitchGait", types.SimpleNamespace(device_id="a2", gait_type=3), ("switch_gait", 3, 0.0, False)),
        ("SetSpeedLevel", types.SimpleNamespace(device_id="a2", level=2), ("speed_level", 2, 0.0, False)),
        ("SetBodyHeight", types.SimpleNamespace(device_id="a2", height=0.04), ("body_height", 0, 0.04, False)),
    ]

    for method_name, request, expected in cases:
        response = asyncio.run(getattr(service, method_name)(request, _Context()))
        assert response.success is True

    assert node.motion_commands == [expected for _, _, expected in cases]


def test_get_control_state_returns_latest_structured_state() -> None:
    node = _FakeNode()
    service = _service(node)

    response = asyncio.run(service.GetControlState(types.SimpleNamespace(device_id="a2"), _Context()))

    assert response.device_id == "a2"
    assert response.runtime_mode == "mock"
    assert response.state == "ready"
    assert response.ready is True
    assert response.gait_type == 2
    assert response.speed_level == 1
    assert response.body_height == 0.03
    assert response.auto_recovery is True
    assert response.last_command == "stand_up"


def test_get_motion_authorization_reports_software_authorization_required_when_standing() -> None:
    node = _FakeNode()
    node.control_state.last_command = "stand_up"
    node.snapshot.status.raw_state.motion_mode = 2
    service = _service(node)

    response = asyncio.run(service.GetMotionAuthorization(types.SimpleNamespace(device_id="a2"), _Context()))

    assert response.success is False
    assert response.state == _RobotDogPb2.MOTION_AUTHORIZATION_STATE_STANDING_NOT_AUTHORIZED
    assert response.required_action == _RobotDogPb2.MOTION_AUTHORIZATION_ACTION_NONE
    assert response.standing is True
    assert response.motion_authorized is False
    assert response.manual_start_required is False
    assert response.error_code == "motion_authorization_required"


def test_authorize_motion_requires_stand_up_before_software_authorization() -> None:
    node = _FakeNode()
    node.control_state.last_command = "stand_down"
    node.snapshot.status.raw_state.position = [0.0, 0.0, 0.08]
    node.snapshot.status.raw_state.body_height = 0.08
    node.snapshot.status.raw_state.motion_mode = 5
    service = _service(node)

    response = asyncio.run(service.AuthorizeMotion(types.SimpleNamespace(device_id="a2"), _Context()))

    assert response.success is False
    assert response.state == _RobotDogPb2.MOTION_AUTHORIZATION_STATE_STAND_DOWN
    assert response.required_action == _RobotDogPb2.MOTION_AUTHORIZATION_ACTION_STAND_UP
    assert response.error_code == "stand_up_required"
    assert node.motion_commands == []
    assert service.p.stack_controller.manual_standby_calls == 0


def test_motion_authorization_uses_successful_stand_command_before_unitree_body_height_offset() -> None:
    node = _FakeNode()
    node.control_state.last_command = "stand_up"
    node.snapshot.status.raw_state.position = [2.94, -5.78, 0.44]
    node.snapshot.status.raw_state.body_height = 0.0
    node.snapshot.status.raw_state.motion_mode = 2
    service = _service(node)

    response = asyncio.run(service.GetMotionAuthorization(types.SimpleNamespace(device_id="a2"), _Context()))

    assert response.success is False
    assert response.state == _RobotDogPb2.MOTION_AUTHORIZATION_STATE_STANDING_NOT_AUTHORIZED
    assert response.required_action == _RobotDogPb2.MOTION_AUTHORIZATION_ACTION_NONE
    assert response.standing is True
    assert response.error_code == "motion_authorization_required"


def test_motion_authorization_uses_unitree_motion_mode_as_primary_posture_signal() -> None:
    node = _FakeNode()
    node.control_state.last_command = "none"
    node.snapshot.status.raw_state.position = [2.94, -5.78, 0.44]
    node.snapshot.status.raw_state.body_height = 0.0
    node.snapshot.status.raw_state.motion_mode = 2
    service = _service(node)

    response = asyncio.run(service.GetMotionAuthorization(types.SimpleNamespace(device_id="a2"), _Context()))

    assert response.success is False
    assert response.state == _RobotDogPb2.MOTION_AUTHORIZATION_STATE_STANDING_NOT_AUTHORIZED
    assert response.required_action == _RobotDogPb2.MOTION_AUTHORIZATION_ACTION_NONE
    assert response.error_code == "motion_authorization_required"


def test_get_motion_authorization_treats_unitree_locomotion_mode_as_authorized() -> None:
    node = _FakeNode()
    node.control_state.last_command = "none"
    node.snapshot.status.raw_state.body_height = 0.0
    node.snapshot.status.raw_state.motion_mode = 3
    service = _service(node)

    response = asyncio.run(service.GetMotionAuthorization(types.SimpleNamespace(device_id="a2"), _Context()))

    assert response.success is True
    assert response.state == _RobotDogPb2.MOTION_AUTHORIZATION_STATE_AUTHORIZED
    assert response.required_action == _RobotDogPb2.MOTION_AUTHORIZATION_ACTION_NONE
    assert response.motion_authorized is True
    assert response.manual_start_required is False


def test_authorize_motion_calls_balance_stand_as_software_authorization() -> None:
    node = _FakeNode()
    node.control_state.last_command = "stand_up"
    node.snapshot.status.raw_state.motion_mode = 2
    service = _service(node)

    response = asyncio.run(service.AuthorizeMotion(types.SimpleNamespace(device_id="a2"), _Context()))

    assert response.success is True
    assert response.state == _RobotDogPb2.MOTION_AUTHORIZATION_STATE_AUTHORIZED
    assert response.required_action == _RobotDogPb2.MOTION_AUTHORIZATION_ACTION_NONE
    assert response.motion_authorized is True
    assert response.manual_start_required is False
    assert node.motion_commands == [("balance_stand", 0, 0.0, False)]
    assert service.p.stack_controller.manual_standby_calls == 1


def test_motion_authorization_reports_stand_down_from_unitree_lie_down_mode() -> None:
    node = _FakeNode()
    node.control_state.last_command = "none"
    node.snapshot.status.raw_state.body_height = 0.0
    node.snapshot.status.raw_state.motion_mode = 5
    service = _service(node)

    response = asyncio.run(service.GetMotionAuthorization(types.SimpleNamespace(device_id="a2"), _Context()))

    assert response.success is False
    assert response.state == _RobotDogPb2.MOTION_AUTHORIZATION_STATE_STAND_DOWN
    assert response.required_action == _RobotDogPb2.MOTION_AUTHORIZATION_ACTION_STAND_UP
    assert response.error_code == "stand_up_required"


def test_release_motion_authorization_stops_motion_and_returns_stop_action() -> None:
    node = _FakeNode()
    node.control_state.last_command = "move"
    service = _service(node)

    response = asyncio.run(service.ReleaseMotionAuthorization(types.SimpleNamespace(device_id="a2"), _Context()))

    assert response.success is True
    assert response.required_action == _RobotDogPb2.MOTION_AUTHORIZATION_ACTION_STOP
    assert response.error_code == "ok"
    assert node.motion_commands == [("stop", 0, 0.0, False)]
