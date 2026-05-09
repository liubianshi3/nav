#!/usr/bin/env python3
"""Send a properly timestamped initial pose for NDT."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys


def main():
    rclpy.init()
    node = Node('initial_pose_sender')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    x = float(sys.argv[1]) if len(sys.argv) > 1 else 3.94
    y = float(sys.argv[2]) if len(sys.argv) > 2 else -7.42

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance = [
        0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    ]

    node.get_logger().info(f'Sending 50 initial poses: ({x:.2f}, {y:.2f})')

    for i in range(50):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)

    node.get_logger().info('Done.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
