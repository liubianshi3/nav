#!/usr/bin/env python3
"""Simplified safety gate: activate collision_monitor and publish allow_motion=true.

Only runs when safety_supervisor is not active. Waits for lifecycle services
to become available before activating.
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


class SimplifiedSafetyGate(Node):
    def __init__(self):
        super().__init__("simplified_safety_gate")
        self.allow_motion_pub = self.create_publisher(Bool, "/a2/allow_motion", 10)
        self.activated = False
        self.timer = self.create_timer(2.0, self.try_activate)

    def try_activate(self):
        if self.activated:
            return
        # Activate collision_monitor
        client = self.create_client(ChangeState, "/collision_monitor/change_state")
        if not client.wait_for_service(timeout_sec=1.0):
            return
        req = ChangeState.Request()
        req.transition = Transition(id=1, label="configure")
        client.call(req)
        req2 = ChangeState.Request()
        req2.transition = Transition(id=3, label="activate")
        client.call(req2)
        self.destroy_client(client)
        # Publish allow_motion
        self.allow_motion_pub.publish(Bool(data=True))
        self.activated = True
        self.get_logger().info("Simplified safety gate: collision_monitor activated, allow_motion=true")
        self.timer.cancel()


def main():
    rclpy.init()
    node = SimplifiedSafetyGate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
