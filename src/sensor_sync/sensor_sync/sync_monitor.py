#!/usr/bin/env python3

from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool, String


class SyncMonitor(Node):
    def __init__(self):
        super().__init__("sync_monitor")
        self.runtime_mode = self.declare_parameter("runtime_mode", "real").value
        imu_topic = self.declare_parameter("imu_topic", "/imu/data").value
        pointcloud_topic = self.declare_parameter("pointcloud_topic", "/jt128/front/points").value
        self.status_report_topic = self.declare_parameter(
            "status_report_topic", "/a2/sensor_sync/status"
        ).value
        self.max_age_sec = float(self.declare_parameter("max_age_sec", 0.25).value)
        self.warn_skew_sec = float(self.declare_parameter("warn_skew_sec", 0.05).value)
        self.history_size = int(self.declare_parameter("history_size", 256).value)

        self.last_imu_stamp = None
        self.last_cloud_stamp = None
        self.last_imu_rx_time = None
        self.last_cloud_rx_time = None
        self.imu_history = deque(maxlen=max(4, self.history_size))
        self.cloud_history = deque(maxlen=max(4, self.history_size))
        self.status_pub = self.create_publisher(Bool, "/a2/sensor_sync/ok", 10)
        self.status_report_pub = self.create_publisher(String, self.status_report_topic, 10)
        self.last_status_text = ""

        self.create_subscription(Imu, imu_topic, self.on_imu, 20)
        self.create_subscription(PointCloud2, pointcloud_topic, self.on_cloud, 10)
        self.create_timer(0.5, self.check_status)

    def on_imu(self, msg):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        self.last_imu_stamp = stamp
        self.last_imu_rx_time = self.get_clock().now()
        self.imu_history.append(stamp)

    def on_cloud(self, msg):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        self.last_cloud_stamp = stamp
        self.last_cloud_rx_time = self.get_clock().now()
        self.cloud_history.append(stamp)

    def check_status(self):
        now = self.get_clock().now()
        imu_ok = (
            self.last_imu_rx_time is not None
            and (now - self.last_imu_rx_time).nanoseconds * 1e-9 <= self.max_age_sec
        )
        cloud_ok = (
            self.last_cloud_rx_time is not None
            and (now - self.last_cloud_rx_time).nanoseconds * 1e-9 <= self.max_age_sec
        )
        skew_ok = True
        skew = 0.0
        if self.last_imu_stamp is not None and self.last_cloud_stamp is not None:
            skew = self.compute_nearest_pair_skew()
            skew_ok = skew <= self.warn_skew_sec
            if not skew_ok:
                self.get_logger().warn(f"IMU / point cloud skew too large: {skew:.3f}s")
        ready = imu_ok and cloud_ok and skew_ok
        self.status_pub.publish(Bool(data=ready))

        reason = "ok"
        state = "ready"
        if not imu_ok and not cloud_ok:
            state = "waiting_inputs"
            reason = "imu_stale,pointcloud_stale"
        elif not imu_ok:
            state = "imu_stale"
            reason = "imu_stale"
        elif not cloud_ok:
            state = "pointcloud_stale"
            reason = "pointcloud_stale"
        elif not skew_ok:
            state = "skew_too_large"
            reason = f"skew={skew:.3f}s"
        self.publish_status(ready, state, reason)

    def publish_status(self, ready, state, reason):
        mode = self.runtime_mode
        status = f"mode={mode};state={state};ready={str(bool(ready)).lower()};reason={reason}"
        self.status_report_pub.publish(String(data=status))
        if status != self.last_status_text:
            self.get_logger().info(f"Sensor sync status changed: {status}")
            self.last_status_text = status

    def compute_nearest_pair_skew(self):
        if not self.cloud_history or not self.imu_history:
            return 0.0
        latest_cloud = self.cloud_history[-1]
        nearest_imu = min(
            self.imu_history,
            key=lambda stamp: abs((stamp - latest_cloud).nanoseconds),
        )
        return abs((nearest_imu - latest_cloud).nanoseconds) * 1e-9


def main():
    rclpy.init()
    node = SyncMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
