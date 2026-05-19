#!/usr/bin/env python3

from __future__ import annotations

import math
import os
import subprocess
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class Jt128DlioWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("jt128_dlio_watchdog")
        self.odom_topic = self.declare_parameter("odom_topic", "/jt128/dlio/odom").value
        self.status_topic = self.declare_parameter("status_topic", "/a2/jt128/dlio_watchdog").value
        self.max_position_norm = float(self.declare_parameter("max_position_norm", 50.0).value)
        self.max_abs_z = float(self.declare_parameter("max_abs_z", 5.0).value)
        self.max_linear_speed = float(self.declare_parameter("max_linear_speed", 2.0).value)
        self.fault_sample_count = int(self.declare_parameter("fault_sample_count", 10).value)
        self.startup_grace_sec = float(self.declare_parameter("startup_grace_sec", 8.0).value)
        self.stop_on_fault = bool(self.declare_parameter("stop_on_fault", True).value)
        raw_stop_script = self.declare_parameter(
            "stop_script", "${A2_WORKSPACE}/install/a2_system/share/a2_system/stop_jt128_stack.sh"
        ).value
        self.stop_script = Path(os.path.expandvars(os.path.expanduser(raw_stop_script)))
        self.started_at = time.monotonic()
        self.last_odom_at: float | None = None
        self.faulted = False
        self.pending_fault_count = 0
        self.last_status = ""
        self.last_logged_state = ""
        self.last_ok_log_at = 0.0
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        self.create_timer(1.0, self.publish_heartbeat)

    def on_odom(self, msg: Odometry) -> None:
        if self.faulted:
            return
        self.last_odom_at = time.monotonic()
        age = time.monotonic() - self.started_at
        if age < self.startup_grace_sec:
            self.publish_status("warming_up", True, f"age={age:.1f}")
            return
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        position_norm = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
        speed_norm = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        reason = None
        if not all(math.isfinite(value) for value in [p.x, p.y, p.z, v.x, v.y, v.z]):
            reason = "nonfinite_odom"
        elif position_norm > self.max_position_norm:
            reason = f"position_norm={position_norm:.2f}>{self.max_position_norm:.2f}"
        elif abs(p.z) > self.max_abs_z:
            reason = f"abs_z={abs(p.z):.2f}>{self.max_abs_z:.2f}"
        elif speed_norm > self.max_linear_speed:
            reason = f"speed_norm={speed_norm:.2f}>{self.max_linear_speed:.2f}"
        if reason is None:
            self.pending_fault_count = 0
            self.publish_status(
                "ok",
                True,
                f"position_norm={position_norm:.3f};z={p.z:.3f};speed={speed_norm:.3f}",
            )
            return
        self.pending_fault_count += 1
        if self.pending_fault_count < self.fault_sample_count:
            self.publish_status(
                state="suspect",
                ready=True,
                reason=f"{reason};sample={self.pending_fault_count}/{self.fault_sample_count}",
            )
            return
        self.faulted = True
        self.publish_status("fault", False, reason)
        self.get_logger().error(f"JT128 DLIO watchdog fault: {reason}")
        if self.stop_on_fault:
            self.stop_stack()

    def stop_stack(self) -> None:
        if not self.stop_script.exists():
            self.get_logger().error(f"stop script not found: {self.stop_script}")
            return
        try:
            subprocess.Popen(
                [str(self.stop_script)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            self.get_logger().error(f"Triggered JT128 stack stop via {self.stop_script}")
        except Exception as exc:
            self.get_logger().error(f"failed to trigger stop script: {exc}")

    def publish_heartbeat(self) -> None:
        if not self.faulted:
            if self.last_odom_at is None:
                self.publish_status("waiting_odom", False, f"topic={self.odom_topic}")
                return
            odom_age = time.monotonic() - self.last_odom_at
            if odom_age > 1.0:
                self.publish_status("stale_odom", False, f"age={odom_age:.2f};topic={self.odom_topic}")

    def publish_status(self, state: str, ready: bool, reason: str) -> None:
        status = f"state={state};ready={str(bool(ready)).lower()};reason={reason};odom_topic={self.odom_topic}"
        self.status_pub.publish(String(data=status))
        now = time.monotonic()
        should_log = state != self.last_logged_state or state != "ok" or now - self.last_ok_log_at >= 5.0
        if should_log:
            self.get_logger().info(f"JT128 DLIO watchdog status changed: {status}")
            self.last_logged_state = state
            if state == "ok":
                self.last_ok_log_at = now
        self.last_status = status


def main() -> None:
    rclpy.init()
    node = Jt128DlioWatchdog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
