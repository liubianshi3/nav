#!/usr/bin/env python3
"""Simplified safety gate: activate collision_monitor and publish allow_motion=true.

Only runs when safety_supervisor is not active. Waits for lifecycle services
to become available before activating.
"""
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
        self.timer = self.create_timer(2.0, self.try_activate)

    def _get_state(self):
        """Return collision_monitor lifecycle state id, or None on failure."""
        get_client = self.create_client(GetState, "/collision_monitor/get_state")
        if not get_client.wait_for_service(timeout_sec=2.0):
            self.destroy_client(get_client)
            return None
        result = get_client.call(GetState.Request())
        self.destroy_client(get_client)
        return result.current_state.id if result else None

    def try_activate(self):
        # Always publish safety topics while active
        if self.activated:
            self.allow_motion_pub.publish(Bool(data=True))
            self.map_ready_pub.publish(Bool(data=True))
            self.localization_ok_pub.publish(Bool(data=True))
            return
        # Query current state first
        state_id = self._get_state()
        if state_id is None:
            self.get_logger().info("collision_monitor not ready, retrying...")
            return
        # State 3 = active: already running, skip lifecycle calls
        if state_id == 3:
            self.get_logger().info("collision_monitor already active, publishing allow_motion=true")
            self.allow_motion_pub.publish(Bool(data=True))
            self.map_ready_pub.publish(Bool(data=True))
            self.localization_ok_pub.publish(Bool(data=True))
            self.activated = True
            return
        client = self.create_client(ChangeState, "/collision_monitor/change_state")
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("collision_monitor change_state not ready, retrying...")
            self.destroy_client(client)
            return
        # State 1 = unconfigured: configure first
        if state_id == 1:
            req = ChangeState.Request()
            req.transition = Transition(id=1, label="configure")
            result = client.call(req)
            if not result.success:
                self.get_logger().warn(f"configure failed: {result}")
            time.sleep(0.5)
        # Activate (state 2 = inactive)
        req2 = ChangeState.Request()
        req2.transition = Transition(id=3, label="activate")
        result2 = client.call(req2)
        self.destroy_client(client)
        if result2.success:
            self.get_logger().info("collision_monitor activated")
        else:
            self.get_logger().warn(f"activate failed, retrying: {result2}")
            return
        # Publish safety gate topics
        self.allow_motion_pub.publish(Bool(data=True))
        self.map_ready_pub.publish(Bool(data=True))
        self.localization_ok_pub.publish(Bool(data=True))
        self.activated = True
        self.get_logger().info("Simplified safety gate: all done, allow_motion=true")


def main():
    rclpy.init()
    node = SimplifiedSafetyGate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
