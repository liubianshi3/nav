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


def _parse_bool(value, default=False):
    if value is None:
        return default
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "t", "yes", "y", "on"):
        return True
    if normalized in ("0", "false", "f", "no", "n", "off"):
        return False
    return default


def _parse_float(value):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed


def _parse_int(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def classify_ndt_health_status(
    parsed,
    *,
    min_score: float,
    initial_guess_timeout_sec: float = 1.0,
):
    """Return (healthy, state, reason) for an adapter status payload."""
    if not parsed:
        return False, "waiting_initial_guess", "initial_guess_missing"

    ready = _parse_bool(parsed.get("ready"), False)
    adapter_state = parsed.get("state", "")
    adapter_reason = parsed.get("reason", "")
    score = _parse_float(parsed.get("score"))
    score_fresh = _parse_bool(parsed.get("score_fresh"), None)
    initial_guess_count = _parse_int(parsed.get("initial_guess_count"))
    last_initial_guess_age = _parse_float(parsed.get("last_initial_guess_age"))

    reason_text = f"{adapter_state} {adapter_reason}".lower()
    if "out_of_map" in reason_text or "map_range" in reason_text:
        return False, "out_of_map", adapter_reason or adapter_state or "map_range"

    if (
        adapter_state in ("waiting_seed", "waiting_first_score")
        or adapter_reason in ("send_initialpose", "initialpose_without_odom", "ndt_not_scored_yet")
        or initial_guess_count == 0
    ):
        return False, "waiting_initial_guess", "initial_guess_missing"

    if (
        last_initial_guess_age is not None
        and last_initial_guess_age >= 0.0
        and initial_guess_timeout_sec > 0.0
        and last_initial_guess_age > initial_guess_timeout_sec
    ):
        return False, "pose_buffer_insufficient", "initial_guess_stale"

    if (
        adapter_state == "waiting_score"
        or adapter_reason in ("score_stale", "no_recent_ndt_score")
        or score_fresh is False
    ):
        if score_fresh is False:
            return False, "no_recent_ndt_score", "score_stale"
        return False, "no_recent_ndt_score", adapter_reason or "score_stale"

    if score is None:
        return False, "waiting_initial_guess", "score_missing"

    if score < min_score:
        return False, "score_low", "score_below_threshold"

    if ready:
        return True, "healthy", adapter_reason or "score_ok"

    return False, adapter_state or "ndt_not_ready", adapter_reason or "adapter_not_ready"


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
        self.initial_guess_timeout_sec = float(
            self.declare_parameter("initial_guess_timeout_sec", 1.0).value
        )

        self.last_score = None
        self.last_ready = False
        self.last_parsed_status = {}
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
        self.last_parsed_status = parsed

        ready_str = parsed.get("ready", "false")
        self.last_ready = ready_str.lower() == "true"

        try:
            self.last_score = float(parsed.get("score", -1.0))
        except (ValueError, TypeError):
            self.last_score = None

    def evaluate(self):
        score_ok, state, reason = classify_ndt_health_status(
            self.last_parsed_status,
            min_score=self.min_score,
            initial_guess_timeout_sec=self.initial_guess_timeout_sec,
        )

        if score_ok:
            self.failure_count = 0
            self.healthy = True
        else:
            self.failure_count += 1
            if self.failure_count >= self.consecutive_failures_threshold:
                self.healthy = False

        self.health_pub.publish(Bool(data=self.healthy))
        self.publish_health_status(score_ok, state, reason)

    def publish_health_status(self, score_ok, state, reason):
        score_str = f"{self.last_score:.3f}" if self.last_score is not None else "none"
        parsed = self.last_parsed_status
        last_score_age = parsed.get("last_score_age", "-1.000")
        last_initial_guess_age = parsed.get("last_initial_guess_age", "-1.000")
        initial_guess_count = parsed.get("initial_guess_count", "0")
        adapter_state = parsed.get("state", "unknown")
        adapter_reason = parsed.get("reason", "unknown")

        status = (
            f"state={state};healthy={str(self.healthy).lower()};"
            f"reason={reason};"
            f"score={score_str};min_score={self.min_score:.3f};"
            f"failure_count={self.failure_count}/{self.consecutive_failures_threshold};"
            f"ndt_ready={str(self.last_ready).lower()};"
            f"adapter_state={adapter_state};adapter_reason={adapter_reason};"
            f"last_score_age={last_score_age};"
            f"last_initial_guess_age={last_initial_guess_age};"
            f"initial_guess_count={initial_guess_count}"
        )
        self.health_status_pub.publish(String(data=status))

        if not score_ok and self.failure_count == 1:
            self.get_logger().warn(
                f"NDT health degraded: state={state}, reason={reason}, "
                f"score={score_str}, threshold={self.min_score:.3f}"
            )
        elif not self.healthy and self.failure_count == self.consecutive_failures_threshold:
            self.get_logger().error(
                f"NDT localization FAILED: state={state}, reason={reason}, "
                f"failures={self.failure_count}"
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
