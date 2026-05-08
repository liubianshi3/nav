#!/usr/bin/env python3
"""
ndt_health_monitor — watches NDT scan-matching quality and publishes a health signal.

Subscribes to /a2/relocalization/status (key=value;... string from the NDT adapter).
Parses the score field, counts consecutive low-score readings, and when the count
exceeds a threshold, declares NDT unhealthy so the safety supervisor can gate motion.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


def parse_ndt_status(text):
    """Parse semicolon-separated key=value pairs into a dict.

    Returns empty dict on parse failure — callers treat missing fields as defaults.
    """
    out = {}
    for part in text.split(";"):
        part = part.strip()
        if "=" not in part:
            continue
        k, v = part.split("=", 1)
        out[k.strip()] = v.strip()
    return out


class NdtHealthMonitor(Node):
    def __init__(self):
        super().__init__("ndt_health_monitor")

        self.status_topic = self.declare_parameter(
            "ndt_status_topic", "/a2/relocalization/status"
        ).value
        self.health_pub_topic = self.declare_parameter(
            "health_pub_topic", "/a2/ndt/healthy"
        ).value
        self.health_status_topic = self.declare_parameter(
            "health_status_topic", "/a2/ndt/health_status"
        ).value
        self.min_score = float(self.declare_parameter("min_score", 0.5).value)
        self.consecutive_failures_threshold = int(
            self.declare_parameter("consecutive_failures_threshold", 5).value
        )
        self.eval_frequency = float(self.declare_parameter("eval_frequency", 5.0).value)

        self.last_score = None
        self.last_ready = False
        self.failure_count = 0
        self.healthy = False
        self.last_status_text = ""

        self.health_pub = self.create_publisher(Bool, self.health_pub_topic, 10)
        self.health_status_pub = self.create_publisher(String, self.health_status_topic, 10)

        self.create_subscription(String, self.status_topic, self.on_status, 10)
        self.create_timer(1.0 / self.eval_frequency, self.evaluate)

        self.get_logger().info(
            f"NDT health monitor started: min_score={self.min_score:.3f}, "
            f"failures_threshold={self.consecutive_failures_threshold}"
        )

    def on_status(self, msg):
        self.last_status_text = msg.data
        parsed = parse_ndt_status(msg.data)

        ready_str = parsed.get("ready", "false")
        self.last_ready = ready_str.lower() == "true"

        try:
            self.last_score = float(parsed.get("score", -1.0))
        except (ValueError, TypeError):
            self.last_score = None

    def evaluate(self):
        score_ok = (
            self.last_score is not None
            and self.last_ready
            and self.last_score >= self.min_score
        )

        if score_ok:
            self.failure_count = 0
            self.healthy = True
        else:
            self.failure_count += 1
            if self.failure_count >= self.consecutive_failures_threshold:
                self.healthy = False

        self.health_pub.publish(Bool(data=self.healthy))
        self.publish_health_status(score_ok)

    def publish_health_status(self, score_ok):
        score_str = f"{self.last_score:.3f}" if self.last_score is not None else "none"
        if not self.last_ready:
            state = "ndt_not_ready"
        elif self.failure_count == 0:
            state = "healthy"
        elif self.failure_count < self.consecutive_failures_threshold:
            state = "degrading"
        else:
            state = "failed"

        status = (
            f"state={state};healthy={str(self.healthy).lower()};"
            f"score={score_str};min_score={self.min_score:.3f};"
            f"failure_count={self.failure_count}/{self.consecutive_failures_threshold};"
            f"ndt_ready={str(self.last_ready).lower()}"
        )
        self.health_status_pub.publish(String(data=status))

        if not score_ok and self.failure_count == 1:
            self.get_logger().warn(
                f"NDT score degraded: score={score_str}, threshold={self.min_score:.3f}"
            )
        elif not self.healthy and self.failure_count == self.consecutive_failures_threshold:
            self.get_logger().error(
                f"NDT localization FAILED after {self.failure_count} consecutive low scores"
            )
        elif score_ok and self.failure_count == 0 and self.healthy:
            pass  # healthy — silent


def main():
    rclpy.init()
    node = NdtHealthMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
