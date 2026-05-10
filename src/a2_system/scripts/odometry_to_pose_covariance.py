#!/usr/bin/env python3
"""
Bridge robot_localization ekf_node output (nav_msgs/Odometry) to the
PoseWithCovarianceStamped format the Autoware NDT scan matcher expects
on its input_initial_pose_topic (ekf_pose_with_covariance).

Subscribes: /odometry/filtered  (nav_msgs/Odometry, map→base_link)
Publishes:  ekf_pose_with_covariance (PoseWithCovarianceStamped, map frame)
"""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdometryToPoseCovariance(Node):
    def __init__(self) -> None:
        super().__init__("odometry_to_pose_covariance")

        self._input_topic = self.declare_parameter(
            "input_topic", "/odometry/filtered"
        ).value
        self._output_topic = self.declare_parameter(
            "output_topic", "ekf_pose_with_covariance"
        ).value

        self._sub = self.create_subscription(
            Odometry, self._input_topic, self._on_odom, 10
        )
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, self._output_topic, 10
        )

        self.get_logger().info(
            f"Odom→PoseWithCovariance bridge: {self._input_topic} → {self._output_topic}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.pose = msg.pose
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = OdometryToPoseCovariance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
