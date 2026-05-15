#!/usr/bin/env python3

import rclpy
from a2_interfaces.srv import SetMode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String


class SlamOrchestrator(Node):
    def __init__(self):
        super().__init__("slam_orchestrator")
        self.runtime_mode = self.declare_parameter("runtime_mode", "real").value
        self.default_mode = self.declare_parameter("default_mode", "mapping").value
        self.stack_profile = self.declare_parameter("stack_profile", "fast_lio").value
        self.stack_available = bool(self.declare_parameter("stack_available", False).value)
        self.stack_blocked_reason = self.declare_parameter("stack_blocked_reason", "").value
        self.status_timeout_sec = float(
            self.declare_parameter("status_timeout_sec", 1.0).value
        )
        self.external_odom_topics = list(
            self.declare_parameter("external_odom_topics", ["/Odometry"]).value
        )
        self.mode = self.default_mode
        self.last_external_odom_time = None
        self.last_external_odom_topic = ""

        self.status_pub = self.create_publisher(String, "/a2/slam/status", 10)
        self.mode_pub = self.create_publisher(String, "/a2/slam/mode", 10)
        self.mapping_pub = self.create_publisher(Bool, "/a2/slam/mapping_active", 10)
        self.create_service(SetMode, "/slam_manager/set_mode", self.handle_set_mode)
        self.odom_subscriptions = []
        for topic in self.external_odom_topics:
            if not topic:
                continue
            self.odom_subscriptions.append(
                self.create_subscription(
                    Odometry,
                    topic,
                    lambda msg, topic_name=topic: self.on_external_odom(msg, topic_name),
                    10,
                )
            )
        self.create_timer(0.2, self.tick)

    def handle_set_mode(self, request, response):
        allowed = {"mapping", "localization", "navigation", "idle"}
        if request.mode not in allowed:
            response.success = False
            response.message = f"unsupported slam mode: {request.mode}"
            return response
        self.mode = request.mode
        response.success = True
        response.message = f"slam mode set to {self.mode}"
        return response

    def on_external_odom(self, msg, topic_name):
        del msg
        self.last_external_odom_time = self.get_clock().now()
        self.last_external_odom_topic = topic_name

    def tick(self):
        self.mode_pub.publish(String(data=self.mode))
        self.mapping_pub.publish(Bool(data=self.mode == "mapping"))
        self.status_pub.publish(String(data=self.build_status()))

    def build_status(self):
        if self.stack_blocked_reason:
            return (
                f"mode=real;state=blocked;ready=false;reason={self.stack_blocked_reason};"
                f"slam_mode={self.mode};profile={self.stack_profile}"
            )
        if not self.stack_available:
            return (
                f"mode=real;state=waiting_stack;ready=false;reason=external_stack_missing;"
                f"slam_mode={self.mode};profile={self.stack_profile}"
            )
        if self.last_external_odom_time is None:
            return (
                f"mode=real;state=waiting_odometry;ready=false;reason=external_stack_waiting_for_odometry;"
                f"slam_mode={self.mode};profile={self.stack_profile}"
            )

        age = (self.get_clock().now() - self.last_external_odom_time).nanoseconds / 1e9
        if age > self.status_timeout_sec:
            return (
                f"mode=real;state=stale;ready=false;reason=external_odometry_stale;"
                f"slam_mode={self.mode};profile={self.stack_profile};"
                f"topic={self.last_external_odom_topic};age_sec={age:.2f}"
            )
        return (
            f"mode=real;state=ready;ready=true;reason=external_stack_ready;"
            f"slam_mode={self.mode};profile={self.stack_profile};topic={self.last_external_odom_topic}"
        )


def main():
    rclpy.init()
    node = SlamOrchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
