#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class PointCloudRelay(Node):
    def __init__(self):
        super().__init__("pointcloud_relay")
        input_topic = self.declare_parameter("input_topic", "/jt128/front/points").value
        output_topic = self.declare_parameter("output_topic", "/jt128/front/points").value
        self.frame_id = self.declare_parameter("frame_id", "jt128_front_link").value
        self.restamp_on_receive = bool(self.declare_parameter("restamp_on_receive", False).value)
        self.same_topic = str(input_topic).strip() == str(output_topic).strip()
        if self.same_topic and self.restamp_on_receive:
            self.restamp_on_receive = False
            self.get_logger().warn("restamp_on_receive disabled because input_topic == output_topic")
        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)
        self.create_subscription(PointCloud2, input_topic, self.on_cloud, 10)
        self.get_logger().info(
            f"Relaying PointCloud2 {input_topic} -> {output_topic} "
            f"frame={self.frame_id} restamp_on_receive={self.restamp_on_receive}"
        )

    def on_cloud(self, msg):
        if self.same_topic and not self.restamp_on_receive:
            desired = self.frame_id or msg.header.frame_id or "jt128_front_link"
            if (msg.header.frame_id or "") == desired:
                return
        relayed = PointCloud2()
        relayed.header = msg.header
        if self.restamp_on_receive:
            relayed.header.stamp = self.get_clock().now().to_msg()
        relayed.header.frame_id = self.frame_id or msg.header.frame_id or "jt128_front_link"
        relayed.height = msg.height
        relayed.width = msg.width
        relayed.fields = msg.fields
        relayed.is_bigendian = msg.is_bigendian
        relayed.point_step = msg.point_step
        relayed.row_step = msg.row_step
        relayed.data = msg.data
        relayed.is_dense = msg.is_dense
        self.publisher.publish(relayed)


def main():
    rclpy.init()
    node = PointCloudRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
