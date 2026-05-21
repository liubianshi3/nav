#!/usr/bin/env python3
"""
a2_battery_publisher — publishes sensor_msgs/BatteryState for the Unitree A2.

Compatibility publisher for development and tests.

Real robot battery data is published by a2_sdk_bridge from unitree_agent state.
This node deliberately does not import Unitree SDK2 so ROS processes do not load
the robot-side DDS stack.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class A2BatteryPublisher(Node):
    def __init__(self):
        super().__init__("a2_battery_publisher")

        self.use_mock = self.declare_parameter("use_mock", True).value
        self.simulate_battery = self.declare_parameter("simulate_battery", False).value
        self.battery_topic = self.declare_parameter(
            "battery_topic", "/a2/battery"
        ).value
        self.publish_hz = max(0.1, float(self.declare_parameter("publish_hz", 1.0).value))
        self.low_threshold = float(self.declare_parameter("low_threshold_percent", 20.0).value)

        self._battery_pub = self.create_publisher(BatteryState, self.battery_topic, 10)
        self.create_timer(1.0 / self.publish_hz, self._publish)

        self._mock_percent = 85.0
        self._mock_charging = False
        self._mock_voltage = 29.4

        self.get_logger().info(
            f"a2_battery_publisher started: topic={self.battery_topic} "
            f"mock={self.use_mock} simulate={self.simulate_battery} hz={self.publish_hz}; "
            "real battery state is owned by a2_sdk_bridge/unitree_agent"
        )

    def _publish(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()

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

        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self._battery_pub.publish(msg)


def main():
    rclpy.init()
    node = A2BatteryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
