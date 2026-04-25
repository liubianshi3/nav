#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class LocalizationGate(Node):
    def __init__(self):
        super().__init__("localization_gate")
        self.use_mock = bool(self.declare_parameter("use_mock", True).value)
        self.runtime_mode = self.declare_parameter(
            "runtime_mode", "mock" if self.use_mock else "real"
        ).value
        pose_topic = self.declare_parameter("input_pose_topic", "/amcl_pose").value
        self.status_topic = self.declare_parameter("status_topic", "/a2/localization_ok").value
        self.status_report_topic = self.declare_parameter(
            "status_report_topic", "/a2/localization/status"
        ).value
        self.max_pose_age_sec = float(self.declare_parameter("max_pose_age_sec", 0.5).value)
        self.latch_valid_pose = bool(self.declare_parameter("latch_valid_pose", False).value)
        self.latched_pose_timeout_sec = float(
            self.declare_parameter("latched_pose_timeout_sec", 300.0).value
        )
        self.pose_transient_local = bool(self.declare_parameter("pose_transient_local", True).value)
        self.allow_zero_stamp_as_now = bool(self.declare_parameter("allow_zero_stamp_as_now", True).value)
        self.max_xy_variance = float(self.declare_parameter("max_xy_variance", 0.25).value)
        self.max_yaw_variance = float(self.declare_parameter("max_yaw_variance", 0.2).value)
        self.last_pose = None
        self.last_valid_pose_time = None
        self.last_status_text = ""

        self.status_pub = self.create_publisher(Bool, self.status_topic, 10)
        self.status_report_pub = self.create_publisher(String, self.status_report_topic, 10)
        pose_qos = (
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            if self.pose_transient_local
            else 20
        )
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.on_pose, pose_qos)
        self.create_timer(0.2, self.evaluate)

    def on_pose(self, msg):
        self.last_pose = msg

    def evaluate(self):
        if self.last_pose is None:
            self.status_pub.publish(Bool(data=False))
            self.publish_status(False, "waiting_pose", "no_pose")
            return
        now = self.get_clock().now()
        stamp = self.last_pose.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0 and self.allow_zero_stamp_as_now:
            age = 0.0
        else:
            pose_time = rclpy.time.Time.from_msg(stamp)
            age = (now - pose_time).nanoseconds * 1e-9
        covariance = self.last_pose.pose.covariance
        xy_ok = covariance[0] <= self.max_xy_variance and covariance[7] <= self.max_xy_variance
        yaw_ok = covariance[35] <= self.max_yaw_variance
        pose_ok = bool(xy_ok and yaw_ok)
        if pose_ok and age <= self.max_pose_age_sec:
            self.last_valid_pose_time = now
            self.status_pub.publish(Bool(data=True))
            self.publish_status(True, "ready", f"pose_ok,age={age:.2f}")
            return
        if (
            pose_ok
            and self.latch_valid_pose
            and self.last_valid_pose_time is not None
            and (now - self.last_valid_pose_time).nanoseconds * 1e-9
            <= self.latched_pose_timeout_sec
        ):
            self.status_pub.publish(Bool(data=True))
            self.publish_status(True, "ready", f"pose_latched,age={age:.2f}")
            return
        self.status_pub.publish(Bool(data=False))
        if age > self.max_pose_age_sec:
            self.publish_status(False, "stale_pose", f"pose_timeout,age={age:.2f}")
            return
        self.publish_status(
            False,
            "covariance_rejected",
            (
                f"xy_ok={str(xy_ok).lower()},yaw_ok={str(yaw_ok).lower()},"
                f"cov_x={covariance[0]:.4f},cov_y={covariance[7]:.4f},cov_yaw={covariance[35]:.4f}"
            ),
        )

    def publish_status(self, ready, state, reason):
        mode = self.runtime_mode
        status = f"mode={mode};state={state};ready={str(bool(ready)).lower()};reason={reason}"
        self.status_report_pub.publish(String(data=status))
        if status != self.last_status_text:
            self.get_logger().info(f"Localization status changed: {status}")
            self.last_status_text = status


def main():
    rclpy.init()
    node = LocalizationGate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
