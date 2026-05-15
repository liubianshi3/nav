#!/usr/bin/env python3
"""
Central diagnostic aggregator for a2_system_ws.

Collects status from all registered a2 components (either via inline String
parsing or via standard DiagnosticArray fusion), applies worst-level-wins
aggregation, and publishes:

  /diagnostics_agg   — DiagnosticArray with every sub-status + global summary
  /a2/health         — Bool (ready / degraded / error)
  /a2/health/status  — String with human-readable global health report
"""

from __future__ import annotations

from typing import Dict, List

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from std_msgs.msg import Bool, String

from a2_diagnostics.status_parser import (
    build_aggregated_status,
    build_diagnostic_status,
    parse_status_string,
)


class DiagnosticAggregator(Node):
    """Aggregate many diagnostic sources into a unified system health view."""

    def __init__(self) -> None:
        super().__init__("diagnostic_aggregator")

        # List of {topic, name, hardware_id} entries for String status topics
        self._sources: List[dict] = []
        source_list = self.declare_parameter("status_sources", []).value
        for entry in source_list:
            self._add_string_source(entry)

        self._diag_sources: Dict[str, List[DiagnosticStatus]] = {}
        self.hardware_id = self.declare_parameter("hardware_id", "a2_robot").value
        self._system_name = self.declare_parameter("system_name", "a2_system").value

        # Publishers
        self._agg_pub = self.create_publisher(DiagnosticArray, "/diagnostics_agg", 10)
        self._health_pub = self.create_publisher(Bool, "/a2/health", 10)
        self._health_status_pub = self.create_publisher(
            String, "/a2/health/status", 10
        )

        # Also subscribe to any pre-existing DiagnosticArray on /diagnostics
        # (for nodes that natively publish DiagnosticStatus, or other adapters)
        self._diag_sub = self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostic_array, 10
        )

        # Publish aggregate at 1 Hz
        self.create_timer(1.0, self._publish_aggregate)

        self.get_logger().info(
            f"Diagnostic aggregator active: {len(self._sources)} string sources"
            + (", " + ", ".join(s["topic"] for s in self._sources) if self._sources else "")
        )

    def _add_string_source(self, entry) -> None:
        """Register a String status topic to watch."""
        topic = entry.get("topic", "")
        name = entry.get("name", "")
        hw_id = entry.get("hardware_id", self.hardware_id)
        if not topic:
            self.get_logger().warn(f"Skipping source with empty topic: {entry}")
            return
        if not name:
            name = topic.lstrip("/").replace("/", "_").strip("_")
        self._sources.append({"topic": topic, "name": name, "hardware_id": hw_id})
        self._diag_sources[name] = []
        self.create_subscription(
            String, topic, lambda msg, n=name, h=hw_id: self._on_string_status(msg, n, h), 10
        )
        self.get_logger().info(f"  watching {topic} → diag[{name}]")

    def _on_string_status(self, msg: String, name: str, hw_id: str) -> None:
        parsed = parse_status_string(msg.data)
        ds = build_diagnostic_status(name, hw_id, parsed)
        self._diag_sources[name] = [ds]

    def _on_diagnostic_array(self, msg: DiagnosticArray) -> None:
        for ds in msg.status:
            self._diag_sources[ds.name] = [ds]

    def _collect_all(self) -> List[DiagnosticStatus]:
        statuses: List[DiagnosticStatus] = []
        for name in sorted(self._diag_sources.keys()):
            items = self._diag_sources[name]
            if items:
                statuses.append(items[-1])  # latest
        return statuses

    def _publish_aggregate(self) -> None:
        now = self.get_clock().now().to_msg()
        all_status = self._collect_all()

        # Global aggregated status
        global_ds = build_aggregated_status(
            all_status, name=self._system_name, hardware_id=self.hardware_id
        )

        # Publish DiagnosticArray with every sub-status + global at front
        da = DiagnosticArray()
        da.header.stamp = now
        da.status = [global_ds] + sorted(all_status, key=lambda s: s.name)
        self._agg_pub.publish(da)

        # Publish simple ready flag
        ready = global_ds.level == DiagnosticStatus.OK
        self._health_pub.publish(Bool(data=ready))

        # Publish human-readable global status
        self._health_status_pub.publish(String(data=global_ds.message))


def main() -> None:
    rclpy.init()
    node = DiagnosticAggregator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
