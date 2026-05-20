#!/usr/bin/env python3
"""Simplified safety gate: activate collision_monitor and publish allow_motion=true.

Only runs when safety_supervisor is not active. Waits for lifecycle services
to become available before activating.
"""
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition


class SimplifiedSafetyGate(Node):
    def __init__(self):
        super().__init__("simplified_safety_gate")
        self.allow_motion_pub = self.create_publisher(Bool, "/a2/allow_motion", 10)
        self.map_ready_pub = self.create_publisher(Bool, "/a2/map_ready", 10)
        self.localization_ok_pub = self.create_publisher(Bool, "/a2/localization_ok", 10)
        self.activated = False
        self._activation_thread_started = False
        self.timer = self.create_timer(2.0, self.try_activate)

    def _publish_safety_topics(self):
        self.allow_motion_pub.publish(Bool(data=True))
        self.map_ready_pub.publish(Bool(data=True))
        self.localization_ok_pub.publish(Bool(data=True))

    def _activate_in_thread(self):
        """Run blocking lifecycle service calls in a daemon thread to avoid deadlocking the executor."""
        node = rclpy.create_node("simplified_safety_gate_activation_helper")
        try:
            get_client = node.create_client(GetState, "/collision_monitor/get_state")
            if not get_client.wait_for_service(timeout_sec=10.0):
                node.get_logger().warn("collision_monitor/get_state service not available")
                return
            future = get_client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            result = future.result()
            state_id = result.current_state.id if result else None

            if state_id is None:
                node.get_logger().warn("Failed to get collision_monitor state")
                return

            if state_id == 3:
                node.get_logger().info("collision_monitor already active")
                self.activated = True
                return

            change_client = node.create_client(ChangeState, "/collision_monitor/change_state")
            if not change_client.wait_for_service(timeout_sec=10.0):
                node.get_logger().warn("collision_monitor/change_state service not available")
                return

            if state_id == 1:
                req = ChangeState.Request()
                req.transition = Transition(id=1, label="configure")
                f = change_client.call_async(req)
                rclpy.spin_until_future_complete(node, f, timeout_sec=5.0)
                if not (f.result() and f.result().success):
                    node.get_logger().warn("configure failed")
                    return
                time.sleep(0.3)

            req2 = ChangeState.Request()
            req2.transition = Transition(id=3, label="activate")
            f2 = change_client.call_async(req2)
            rclpy.spin_until_future_complete(node, f2, timeout_sec=5.0)
            if f2.result() and f2.result().success:
                node.get_logger().info("collision_monitor activated successfully")
                self.activated = True
            else:
                node.get_logger().warn("activate failed")
        finally:
            node.destroy_node()

    def try_activate(self):
        if self.activated:
            self._publish_safety_topics()
            return
        if not self._activation_thread_started:
            self._activation_thread_started = True
            t = threading.Thread(target=self._activate_in_thread, daemon=True)
            t.start()

def main():
    rclpy.init()
    node = SimplifiedSafetyGate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
