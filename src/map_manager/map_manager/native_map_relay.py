#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class NativeMapRelay(Node):
    def __init__(self) -> None:
        super().__init__("native_map_relay")
        self.input_topic = self.declare_parameter("input_topic", "/global_map").value
        self.output_topic = self.declare_parameter("output_topic", "/map").value
        self.output_frame_id = self.declare_parameter("output_frame_id", "map").value
        self.status_topic = self.declare_parameter("status_topic", "/a2/native_map/status").value
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 1.0).value)
        self.last_status = ""
        self.latest_map = None

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, self.output_topic, latched_qos)
        self.status_pub = self.create_publisher(String, self.status_topic, latched_qos)
        self.create_subscription(OccupancyGrid, self.input_topic, self.on_map, latched_qos)
        self.create_subscription(OccupancyGrid, self.input_topic, self.on_map, 10)
        self.create_timer(1.0 / max(self.publish_rate_hz, 0.1), self.republish_latest_map)
        self.publish_status("waiting_source", "no_map_yet")

    def on_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        self.republish_latest_map()

    def republish_latest_map(self) -> None:
        if self.latest_map is None:
            return
        msg = self.latest_map
        if self.output_frame_id:
            msg.header.frame_id = self.output_frame_id
        self.map_pub.publish(msg)
        self.publish_status(
            "ready",
            (
                f"bridged:{self.input_topic}->{self.output_topic},"
                f"size={msg.info.width}x{msg.info.height},"
                f"resolution={msg.info.resolution:.3f}"
            ),
        )

    def publish_status(self, state: str, reason: str) -> None:
        status = f"mode=real;state={state};ready={str(state == 'ready').lower()};reason={reason}"
        self.status_pub.publish(String(data=status))
        if status != self.last_status:
            self.get_logger().info(f"Native map relay status changed: {status}")
            self.last_status = status


def main() -> None:
    rclpy.init()
    node = NativeMapRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
