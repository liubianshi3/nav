#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32, String


class NavHealthMonitor(Node):
    """Consume /diagnostics_agg and drive navigation degradation levels.

    State machine:
        OK (scale=1.0) → WARN (scale=0.5, persistent 3s) → ERROR (scale=0.0, cancel goal)
        → STALE (scale=0.0, estop)
    Recovery: OK must hold for recovery_hold_sec before leaving degraded state.
    """

    def __init__(self) -> None:
        super().__init__("nav_health_monitor")

        # --- parameters ---
        self.warn_speed_scale = float(
            self.declare_parameter("warn_speed_scale", 0.5).value)
        self.warn_persistence_sec = float(
            self.declare_parameter("warn_persistence_sec", 3.0).value)
        self.recovery_hold_sec = float(
            self.declare_parameter("recovery_hold_sec", 5.0).value)
        self.estop_on_stale = bool(
            self.declare_parameter("estop_on_stale", True).value)
        self.cancel_goal_on_error = bool(
            self.declare_parameter("cancel_goal_on_error", True).value)
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel_safe").value
        self.zero_cmd_on_degraded = bool(
            self.declare_parameter("zero_cmd_on_degraded", False).value)

        # --- state ---
        self._state = "OK"
        self._ok_since: rclpy.time.Time | None = None
        self._degraded_since: rclpy.time.Time | None = None

        # --- subscriptions ---
        self._diag_sub = self.create_subscription(
            DiagnosticArray, "/diagnostics_agg", self._on_diagnostics, 10)

        # --- publishers ---
        self._level_pub = self.create_publisher(String, "/a2/nav/health_level", 10)
        self._speed_pub = self.create_publisher(Float32, "/a2/nav/max_speed_scale", 10)
        self._cancel_pub = self.create_publisher(Empty, "/a2/nav/cancel_goal", 10)
        self._estop_pub = self.create_publisher(Bool, "/a2/nav/estop", 10)
        self._status_pub = self.create_publisher(String, "/a2/nav/status", 10)
        self._cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # --- control tick (10 Hz) ---
        self._control_timer = self.create_timer(0.1, self._control_tick)

        self.get_logger().info(
            "NavHealthMonitor started: warn_scale=%.2f persist=%.1fs recovery=%.1fs",
            self.warn_speed_scale, self.warn_persistence_sec, self.recovery_hold_sec)

    # ------------------------------------------------------------------
    # diagnostics callback
    # ------------------------------------------------------------------
    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        if not msg.status:
            self._transition(DiagnosticStatus.STALE)
            return

        global_level = msg.status[0].level
        if isinstance(global_level, bytes):
            global_level = global_level[0]
        self._transition(global_level)

    # ------------------------------------------------------------------
    # state machine
    # ------------------------------------------------------------------
    def _transition(self, new_level: int) -> None:
        now = self.get_clock().now()

        if new_level == DiagnosticStatus.OK:
            if self._ok_since is None:
                self._ok_since = now
            elapsed = (now - self._ok_since).nanoseconds * 1e-9
            if elapsed >= self.recovery_hold_sec and self._state != "OK":
                self._set_state("OK")
                self._degraded_since = None
            return

        # non-OK → clear recovery timer
        self._ok_since = None

        if new_level == DiagnosticStatus.WARN:
            if self._state == "OK":
                self._set_state("WARN")
            elif self._state == "WARN" and self._degraded_since is not None:
                elapsed = (now - self._degraded_since).nanoseconds * 1e-9
                if elapsed >= self.warn_persistence_sec:
                    self.get_logger().warn(
                        "WARN persisted for %.1fs — escalating to ERROR", elapsed)
                    self._set_state("ERROR")

        elif new_level == DiagnosticStatus.ERROR:
            if self._state in ("OK", "WARN"):
                self._set_state("ERROR")

        elif new_level == DiagnosticStatus.STALE:
            self._set_state("STALE")

    def _set_state(self, state: str) -> None:
        prev = self._state
        self._state = state
        now = self.get_clock().now()

        if state == "OK":
            self._publish_speed_scale(1.0)
            self._publish_estop(False)
            self.get_logger().info("NAV HEALTH: Restored to OK")
        elif state == "WARN":
            self._publish_speed_scale(self.warn_speed_scale)
            self._publish_estop(False)
            self.get_logger().warn(
                "NAV HEALTH: Degraded to WARN (speed x%.2f)", self.warn_speed_scale)
        elif state == "ERROR":
            self._publish_speed_scale(0.0)
            self._publish_estop(False)
            if self.cancel_goal_on_error:
                self._cancel_pub.publish(Empty())
            self.get_logger().error(
                "NAV HEALTH: Degraded to ERROR — goal cancelled, velocity zeroed")
        elif state == "STALE":
            self._publish_speed_scale(0.0)
            if self.cancel_goal_on_error:
                self._cancel_pub.publish(Empty())
            if self.estop_on_stale:
                self._publish_estop(True)
            self.get_logger().fatal("NAV HEALTH: STALE — emergency stop engaged")

        self._degraded_since = now if state != "OK" else None
        self._publish_status()

    # ------------------------------------------------------------------
    # control tick (10 Hz)
    # ------------------------------------------------------------------
    def _control_tick(self) -> None:
        if self._state in ("ERROR", "STALE") and self.zero_cmd_on_degraded:
            self._cmd_vel_pub.publish(Twist())
        self._level_pub.publish(String(data=self._state))

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _publish_speed_scale(self, scale: float) -> None:
        self._speed_pub.publish(Float32(data=scale))

    def _publish_estop(self, active: bool) -> None:
        self._estop_pub.publish(Bool(data=active))

    def _publish_status(self) -> None:
        status = (
            f"state={self._state.lower()};"
            f"ready={str(self._state == 'OK').lower()};"
            f"reason=diagnostic_level_{self._state.lower()};"
            f"source=/diagnostics_agg"
        )
        self._status_pub.publish(String(data=status))


def main() -> None:
    rclpy.init()
    node = NavHealthMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
