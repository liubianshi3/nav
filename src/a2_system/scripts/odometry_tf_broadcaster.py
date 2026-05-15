#!/usr/bin/env python3

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class OdometryTfBroadcaster(Node):
    """Publish odometry pose as an odom->base_link TF for Nav2 consumers."""

    def __init__(self) -> None:
        super().__init__("odometry_tf_broadcaster")

        self._odom_topic = str(self.declare_parameter("odom_topic", "/jt128/dlio/odom").value)
        self._parent_frame = str(self.declare_parameter("parent_frame", "odom").value)
        self._child_frame = str(self.declare_parameter("child_frame", "base_link").value)
        self._use_msg_frame_ids = bool(self.declare_parameter("use_msg_frame_ids", False).value)
        self._flatten_z = bool(self.declare_parameter("flatten_z", True).value)
        self._planarize_orientation = bool(
            self.declare_parameter("planarize_orientation", True).value
        )

        self._tf_broadcaster = TransformBroadcaster(self)
        self._sub = self.create_subscription(Odometry, self._odom_topic, self._on_odom, 50)

        self.get_logger().info(
            "Odometry TF broadcaster started: "
            f"odom_topic={self._odom_topic} "
            f"parent={self._parent_frame} child={self._child_frame} "
            f"flatten_z={self._flatten_z} planarize_orientation={self._planarize_orientation}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        parent_frame = msg.header.frame_id if self._use_msg_frame_ids and msg.header.frame_id else self._parent_frame
        child_frame = msg.child_frame_id if self._use_msg_frame_ids and msg.child_frame_id else self._child_frame
        if not parent_frame or not child_frame:
            self.get_logger().warning("Skipping odom TF publish because frame id is empty.")
            return
        if parent_frame == child_frame:
            self.get_logger().warning(
                f"Skipping odom TF publish because parent and child are both '{parent_frame}'."
            )
            return

        pose = msg.pose.pose
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = float(pose.position.x)
        transform.transform.translation.y = float(pose.position.y)
        transform.transform.translation.z = 0.0 if self._flatten_z else float(pose.position.z)

        if self._planarize_orientation:
            q = pose.orientation
            qx, qy, qz, qw = _quaternion_from_yaw(_yaw_from_quaternion(q.x, q.y, q.z, q.w))
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
        else:
            transform.transform.rotation = pose.orientation

        self._tf_broadcaster.sendTransform(transform)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdometryTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
