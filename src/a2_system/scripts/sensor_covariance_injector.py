#!/usr/bin/env python3
"""Inject conservative covariance into Odometry or Imu messages.

Some third-party sensor sources publish all-zero covariance, which makes
robot_localization treat the measurement as unrealistically certain. This relay
keeps the original message content and only replaces covariance blocks.
"""

from __future__ import annotations

from typing import Iterable

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def _diag36(values: Iterable[float]) -> list[float]:
    diagonal = list(values)
    covariance = [0.0] * 36
    for index, value in enumerate(diagonal[:6]):
        covariance[index * 6 + index] = float(value)
    return covariance


def _diag9(values: Iterable[float]) -> list[float]:
    diagonal = list(values)
    covariance = [0.0] * 9
    for index, value in enumerate(diagonal[:3]):
        covariance[index * 3 + index] = float(value)
    return covariance


class SensorCovarianceInjector(Node):
    def __init__(self) -> None:
        super().__init__("sensor_covariance_injector")
        self.message_type = str(self.declare_parameter("message_type", "odometry").value)
        self.input_topic = str(self.declare_parameter("input_topic", "").value)
        self.output_topic = str(self.declare_parameter("output_topic", "").value)
        self.replace_existing = bool(self.declare_parameter("replace_existing", True).value)
        self.pose_diagonal = self.declare_parameter(
            "pose_covariance_diagonal", [0.05, 0.05, 5.0, 2.0, 2.0, 0.1]
        ).value
        self.twist_diagonal = self.declare_parameter(
            "twist_covariance_diagonal", [0.04, 0.04, 5.0, 2.0, 2.0, 0.2]
        ).value
        self.orientation_diagonal = self.declare_parameter(
            "orientation_covariance_diagonal", [10.0, 10.0, 10.0]
        ).value
        self.angular_velocity_diagonal = self.declare_parameter(
            "angular_velocity_covariance_diagonal", [0.1, 0.1, 0.03]
        ).value
        self.linear_acceleration_diagonal = self.declare_parameter(
            "linear_acceleration_covariance_diagonal", [5.0, 5.0, 5.0]
        ).value

        if not self.input_topic or not self.output_topic:
            raise ValueError("input_topic and output_topic are required")

        if self.message_type == "odometry":
            self.publisher = self.create_publisher(Odometry, self.output_topic, 20)
            self.subscription = self.create_subscription(Odometry, self.input_topic, self._on_odom, 20)
        elif self.message_type == "imu":
            self.publisher = self.create_publisher(Imu, self.output_topic, 50)
            self.subscription = self.create_subscription(Imu, self.input_topic, self._on_imu, 50)
        else:
            raise ValueError("message_type must be 'odometry' or 'imu'")

        self.get_logger().info(
            f"Injecting {self.message_type} covariance: {self.input_topic} -> {self.output_topic}"
        )

    def _should_replace(self, covariance: Iterable[float]) -> bool:
        return self.replace_existing or all(float(value) == 0.0 for value in covariance)

    def _on_odom(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        if self._should_replace(out.pose.covariance):
            out.pose.covariance = _diag36(self.pose_diagonal)
        if self._should_replace(out.twist.covariance):
            out.twist.covariance = _diag36(self.twist_diagonal)
        self.publisher.publish(out)

    def _on_imu(self, msg: Imu) -> None:
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration
        if self._should_replace(out.orientation_covariance):
            out.orientation_covariance = _diag9(self.orientation_diagonal)
        if self._should_replace(out.angular_velocity_covariance):
            out.angular_velocity_covariance = _diag9(self.angular_velocity_diagonal)
        if self._should_replace(out.linear_acceleration_covariance):
            out.linear_acceleration_covariance = _diag9(self.linear_acceleration_diagonal)
        self.publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = SensorCovarianceInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
