#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudToLaserScan(Node):
    def __init__(self):
        super().__init__("pointcloud_to_laserscan")
        self.input_topic = self.declare_parameter("input_topic", "/jt128/front/points").value
        self.output_topic = self.declare_parameter("output_topic", "/scan").value
        self.frame_id = self.declare_parameter("frame_id", "").value
        self.min_height = float(self.declare_parameter("min_height", -0.2).value)
        self.max_height = float(self.declare_parameter("max_height", 1.2).value)
        self.min_range = float(self.declare_parameter("min_range", 0.2).value)
        self.max_range = float(self.declare_parameter("max_range", 12.0).value)
        self.angle_min = float(self.declare_parameter("angle_min", -math.pi).value)
        self.angle_max = float(self.declare_parameter("angle_max", math.pi).value)
        self.angle_increment = float(
            self.declare_parameter("angle_increment", math.radians(1.0)).value
        )
        self.use_inf = bool(self.declare_parameter("use_inf", True).value)

        self.publisher = self.create_publisher(LaserScan, self.output_topic, 10)
        self.create_subscription(PointCloud2, self.input_topic, self.on_cloud, 10)

    def on_cloud(self, msg):
        beam_count = max(1, int(math.ceil((self.angle_max - self.angle_min) / self.angle_increment)))
        default_value = math.inf if self.use_inf else (self.max_range + 1.0)
        ranges = [default_value] * beam_count

        for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if z < self.min_height or z > self.max_height:
                continue
            distance = math.hypot(x, y)
            if distance < self.min_range or distance > self.max_range:
                continue
            angle = math.atan2(y, x)
            if angle < self.angle_min or angle > self.angle_max:
                continue
            index = int((angle - self.angle_min) / self.angle_increment)
            index = max(0, min(beam_count - 1, index))
            if distance < ranges[index]:
                ranges[index] = distance

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.frame_id or msg.header.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.min_range
        scan.range_max = self.max_range
        scan.ranges = ranges
        self.publisher.publish(scan)


def main():
    rclpy.init()
    node = PointCloudToLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
