#!/usr/bin/env python3
"""
Bridge a single a2 status String topic to standard diagnostic_msgs/DiagnosticStatus.

This node subscribes to one of the existing ``mode=...;state=...;ready=...``
String topics, parses it, and publishes a proper :class:`DiagnosticArray`
on ``/diagnostics``.  Use one adapter per existing status topic you want
to surface in standard ROS diagnostic tools.
"""

from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from std_msgs.msg import String

from a2_diagnostics.status_parser import build_diagnostic_status, parse_status_string


class StatusAdapterNode(Node):
    """Bridge ``std_msgs/String`` status → ``diagnostic_msgs/DiagnosticArray``."""

    def __init__(self) -> None:
        super().__init__("status_adapter")
        self.status_topic = self.declare_parameter(
            "status_topic", ""
        ).value
        self.diag_name = self.declare_parameter(
            "diag_name", self.status_topic.lstrip("/").replace("/", "_").strip("_")
        ).value
        self.hardware_id = self.declare_parameter(
            "hardware_id", "a2_robot"
        ).value

        if not self.status_topic:
            self.get_logger().error("status_topic parameter is required")
            raise RuntimeError("status_topic parameter is required")

        self._last_status: DiagnosticStatus | None = None

        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._sub = self.create_subscription(
            String, self.status_topic, self._on_status, 10
        )

        self.get_logger().info(
            f"Status adapter: {self.status_topic} → /diagnostics [{self.diag_name}]"
        )

    def _on_status(self, msg: String) -> None:
        parsed = parse_status_string(msg.data)
        ds = build_diagnostic_status(self.diag_name, self.hardware_id, parsed)
        self._last_status = ds

        da = DiagnosticArray()
        da.header.stamp = self.get_clock().now().to_msg()
        da.status = [ds]
        self._diag_pub.publish(da)


def main() -> None:
    import rclpy
    rclpy.init()
    node = StatusAdapterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
