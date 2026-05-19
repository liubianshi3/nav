#!/usr/bin/env python3
from __future__ import annotations

from typing import Any


DEFAULT_ACCELERATION_SCALE = 9.80665
DEFAULT_ANGULAR_VELOCITY_SCALE = 1.0


def _copy_vector(dst: Any, src: Any, scale: float) -> None:
    dst.x = float(src.x) * scale
    dst.y = float(src.y) * scale
    dst.z = float(src.z) * scale


def _copy_quaternion(dst: Any, src: Any) -> None:
    dst.x = float(src.x)
    dst.y = float(src.y)
    dst.z = float(src.z)
    dst.w = float(src.w)


def _scaled_covariance(values: Any, scale: float) -> list[float]:
    cov = [float(value) for value in values]
    if cov and cov[0] < 0.0:
        return cov
    factor = scale * scale
    return [value * factor for value in cov]


def convert_imu_to_si(
    msg: Any,
    *,
    imu_cls: type[Any] | None = None,
    acceleration_scale: float = DEFAULT_ACCELERATION_SCALE,
    angular_velocity_scale: float = DEFAULT_ANGULAR_VELOCITY_SCALE,
) -> Any:
    if imu_cls is None:
        from sensor_msgs.msg import Imu

        imu_cls = Imu

    converted = imu_cls()
    converted.header = msg.header
    _copy_quaternion(converted.orientation, msg.orientation)
    converted.orientation_covariance = [float(value) for value in msg.orientation_covariance]
    _copy_vector(converted.angular_velocity, msg.angular_velocity, angular_velocity_scale)
    converted.angular_velocity_covariance = _scaled_covariance(
        msg.angular_velocity_covariance,
        angular_velocity_scale,
    )
    _copy_vector(converted.linear_acceleration, msg.linear_acceleration, acceleration_scale)
    converted.linear_acceleration_covariance = _scaled_covariance(
        msg.linear_acceleration_covariance,
        acceleration_scale,
    )
    return converted


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    class ImuToSiConverter(Node):
        def __init__(self) -> None:
            super().__init__("imu_to_si_converter")
            input_topic = str(self.declare_parameter("input_topic", "/jt128/front/imu").value)
            output_topic = str(self.declare_parameter("output_topic", "/jt128/front/imu_si").value)
            self.acceleration_scale = float(
                self.declare_parameter("acceleration_scale", DEFAULT_ACCELERATION_SCALE).value
            )
            self.angular_velocity_scale = float(
                self.declare_parameter("angular_velocity_scale", DEFAULT_ANGULAR_VELOCITY_SCALE).value
            )
            self.publisher = self.create_publisher(Imu, output_topic, 20)
            self.create_subscription(Imu, input_topic, self._on_imu, 20)
            self.get_logger().info(
                "Converting IMU to SI units: "
                f"{input_topic} -> {output_topic}; "
                f"acceleration_scale={self.acceleration_scale}; "
                f"angular_velocity_scale={self.angular_velocity_scale}"
            )

        def _on_imu(self, msg: Imu) -> None:
            converted = convert_imu_to_si(
                msg,
                imu_cls=Imu,
                acceleration_scale=self.acceleration_scale,
                angular_velocity_scale=self.angular_velocity_scale,
            )
            self.publisher.publish(converted)

    rclpy.init()
    node = ImuToSiConverter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
