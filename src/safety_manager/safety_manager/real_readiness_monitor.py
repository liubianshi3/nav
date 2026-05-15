#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class RealReadinessMonitor(Node):
    def __init__(self):
        super().__init__("real_readiness_monitor")
        self.runtime_mode = self.declare_parameter("runtime_mode", "real").value
        self.lidar_connected_topic = self.declare_parameter(
            "lidar_connected_topic", "/a2/lidar/connected"
        ).value
        self.lidar_label = self.declare_parameter("lidar_label", "lidar").value
        self.sdk_connected = False
        self.lidar_connected = False
        self.localization_ok = False
        self.map_ready = False
        self.slam_status = "unknown"

        self.ready_pub = self.create_publisher(Bool, "/a2/real/ready", 10)
        self.report_pub = self.create_publisher(String, "/a2/real/report", 10)
        self.last_report = ""
        self.create_subscription(Bool, "/a2/sdk/connected", self.on_sdk_connected, 10)
        self.create_subscription(Bool, self.lidar_connected_topic, self.on_lidar_connected, 10)
        self.create_subscription(Bool, "/a2/localization_ok", self.on_localization_ok, 10)
        self.create_subscription(Bool, "/a2/map_ready", self.on_map_ready, 10)
        self.create_subscription(String, "/a2/slam/status", self.on_slam_status, 10)
        self.create_timer(0.5, self.tick)

    def on_sdk_connected(self, msg):
        self.sdk_connected = msg.data

    def on_lidar_connected(self, msg):
        self.lidar_connected = msg.data

    def on_localization_ok(self, msg):
        self.localization_ok = msg.data

    def on_map_ready(self, msg):
        self.map_ready = msg.data

    def on_slam_status(self, msg):
        self.slam_status = msg.data

    @staticmethod
    def parse_status_fields(payload):
        fields = {}
        for item in payload.split(";"):
            if "=" not in item:
                continue
            key, value = item.split("=", 1)
            fields[key] = value
        return fields

    def tick(self):
        ready = self.sdk_connected and self.lidar_connected and self.map_ready and self.localization_ok
        reason = []
        if not self.sdk_connected:
            reason.append("sdk_down")
        if not self.lidar_connected:
            reason.append(f"{self.lidar_label}_down")
        if not self.map_ready:
            reason.append("map_down")
        if not self.localization_ok:
            reason.append("localization_down")
        report = self.build_report(
            ready,
            "ready" if ready else "degraded",
            ",".join(reason) if reason else "ok",
        )
        self.ready_pub.publish(Bool(data=ready))
        self.report_pub.publish(String(data=report))
        if report != self.last_report:
            self.get_logger().info(f"Real readiness changed: {report}")
            self.last_report = report

    def build_report(self, ready, state, reason):
        mode = self.runtime_mode
        slam_fields = self.parse_status_fields(self.slam_status)
        return (
            f"mode={mode};state={state};ready={str(bool(ready)).lower()};reason={reason};"
            f"sdk={str(self.sdk_connected).lower()};{self.lidar_label}={str(self.lidar_connected).lower()};"
            f"map={str(self.map_ready).lower()};localization={str(self.localization_ok).lower()};"
            f"slam_state={slam_fields.get('state', 'unknown')};"
            f"slam_ready={slam_fields.get('ready', 'false')};"
            f"slam_reason={slam_fields.get('reason', self.slam_status or 'unknown')}"
        )


def main():
    rclpy.init()
    node = RealReadinessMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
