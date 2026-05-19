from __future__ import annotations

import importlib.util
import sys
from dataclasses import dataclass, field
from pathlib import Path


def load_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "imu_to_si_converter.py"
    spec = importlib.util.spec_from_file_location("imu_to_si_converter_under_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@dataclass
class FakeVector:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class FakeQuaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class FakeHeader:
    frame_id: str = "jt128_front_link"


@dataclass
class FakeImu:
    header: FakeHeader = field(default_factory=FakeHeader)
    orientation: FakeQuaternion = field(default_factory=FakeQuaternion)
    orientation_covariance: list[float] = field(default_factory=lambda: [0.0] * 9)
    angular_velocity: FakeVector = field(default_factory=FakeVector)
    angular_velocity_covariance: list[float] = field(default_factory=lambda: [0.0] * 9)
    linear_acceleration: FakeVector = field(default_factory=FakeVector)
    linear_acceleration_covariance: list[float] = field(default_factory=lambda: [0.0] * 9)


def test_convert_imu_to_si_scales_acceleration_and_covariance():
    converter = load_module()
    msg = FakeImu()
    msg.linear_acceleration = FakeVector(x=0.0, y=1.0, z=-0.5)
    msg.linear_acceleration_covariance = [1.0] + [0.0] * 8
    msg.angular_velocity = FakeVector(x=180.0, y=90.0, z=-45.0)
    msg.angular_velocity_covariance = [4.0] + [0.0] * 8

    converted = converter.convert_imu_to_si(msg, imu_cls=FakeImu)

    assert converted.header.frame_id == "jt128_front_link"
    assert converted.linear_acceleration.y == 9.80665
    assert converted.linear_acceleration.z == -4.903325
    assert abs(converted.linear_acceleration_covariance[0] - 96.1703842225) < 1.0e-9
    assert converted.angular_velocity.x == 180.0
    assert converted.angular_velocity.y == 90.0
    assert converted.angular_velocity.z == -45.0
    assert converted.angular_velocity_covariance[0] == 4.0


def test_convert_imu_to_si_preserves_unknown_covariance_sentinel():
    converter = load_module()
    msg = FakeImu()
    msg.linear_acceleration_covariance = [-1.0] + [0.0] * 8

    converted = converter.convert_imu_to_si(msg, imu_cls=FakeImu)

    assert converted.linear_acceleration_covariance == [-1.0] + [0.0] * 8
