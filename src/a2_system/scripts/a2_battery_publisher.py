#!/usr/bin/env python3
"""
a2_battery_publisher — publishes sensor_msgs/BatteryState for the Unitree A2.

Real mode: queries Unitree SportClient for battery level.
Mock mode: publishes a static battery level for development.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

try:
    from unitree.robot.a2.sport import SportClient as A2SportClient
except ImportError:  # pragma: no cover — dev machine, no Unitree SDK
    A2SportClient = None


def _query_unitree_sdk(client) -> tuple[float, bool, float]:
    """Query Unitree A2 SportClient for battery state.

    Returns (percentage_0_100, is_charging, voltage_volts).
    Defaults to (-1.0, False, -1.0) on failure.
    """
    try:
        state = client.RobotState()
        if state is not None:
            percent = float(getattr(state, "power", -1.0))
            charging = bool(getattr(state, "charging", False))
            voltage = float(getattr(state, "battery_voltage", -1.0))
            return percent, charging, voltage
    except Exception:
        pass
    return -1.0, False, -1.0


class A2BatteryPublisher(Node):
    def __init__(self):
        super().__init__("a2_battery_publisher")

        self.use_mock = self.declare_parameter("use_mock", not _sdk_available()).value
        self.simulate_battery = self.declare_parameter("simulate_battery", False).value
        self.battery_topic = self.declare_parameter(
            "battery_topic", "/a2/battery"
        ).value
        self.publish_hz = max(0.1, float(self.declare_parameter("publish_hz", 1.0).value))
        self.low_threshold = float(self.declare_parameter("low_threshold_percent", 20.0).value)

        self._client = None
        if not self.use_mock:
            self._init_sdk_client()

        self._battery_pub = self.create_publisher(BatteryState, self.battery_topic, 10)
        self.create_timer(1.0 / self.publish_hz, self._publish)

        self._mock_percent = 85.0
        self._mock_charging = False
        self._mock_voltage = 29.4

        self.get_logger().info(
            f"a2_battery_publisher started: topic={self.battery_topic} "
            f"mock={self.use_mock} simulate={self.simulate_battery} hz={self.publish_hz}"
        )

    def _init_sdk_client(self):
        try:
            self._client = A2SportClient()
            self._client.SetTimeout(5.0)
            self._client.Init()
            self.get_logger().info("Unitree SportClient initialized for battery queries")
        except Exception as exc:
            self.get_logger().warn(f"SportClient init failed: {exc}; falling back to mock")
            self.use_mock = True

    def _publish(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self.use_mock or self._client is None:
            if self.simulate_battery:
                msg.percentage = float(self._mock_percent)
                msg.power_supply_status = (
                    BatteryState.POWER_SUPPLY_STATUS_CHARGING
                    if self._mock_charging
                    else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                )
                msg.voltage = float(self._mock_voltage)
                msg.present = True
            else:
                msg.percentage = float("nan")
                msg.voltage = float("nan")
                msg.present = False
        else:
            pct, charging, voltage = _query_unitree_sdk(self._client)
            msg.percentage = float(pct) if math.isfinite(pct) else -1.0
            msg.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_CHARGING
                if charging
                else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            )
            msg.voltage = float(voltage) if math.isfinite(voltage) else float("nan")
            msg.present = pct >= 0.0

        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self._battery_pub.publish(msg)


def _sdk_available():
    return A2SportClient is not None


def main():
    rclpy.init()
    node = A2BatteryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
