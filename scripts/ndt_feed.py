#!/usr/bin/env python3
"""Feed the NDT directly by publishing to its open_loop_pose topic with fresh timestamps."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys
import time

def main():
    rclpy.init()
    node = Node('ndt_feed')
    # Publish directly to NDT's subscribed topic
    pub = node.create_publisher(PoseWithCovarianceStamped, '/a2/ndt/open_loop_pose', 10)
    time.sleep(0.5)

    x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

    node.get_logger().info(f'Feeding NDT at ({x:.2f}, {y:.2f}) for 10s...')

    for i in range(100):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0.1,0,0,0,0,0,0,0.1,0,0,0,0,0,0,0.1]
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    node.get_logger().info('Done feeding.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
