#!/usr/bin/env python3

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


def quaternion_norm(orientation) -> float:
    return math.sqrt(
        orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
        + orientation.w * orientation.w
    )


def normalize_quaternion(orientation) -> bool:
    norm = quaternion_norm(orientation)
    if not math.isfinite(norm) or norm < 1e-6:
        orientation.x = 0.0
        orientation.y = 0.0
        orientation.z = 0.0
        orientation.w = 1.0
        return False
    orientation.x /= norm
    orientation.y /= norm
    orientation.z /= norm
    orientation.w /= norm
    return True


def validate_goal_contract(goal, *, map_frame: str, require_map_frame: bool, max_goal_distance_from_origin: float):
    if not goal.header.frame_id:
        goal.header.frame_id = map_frame
    if require_map_frame and goal.header.frame_id != map_frame:
        return None, f"bad_frame:{goal.header.frame_id}"
    x = float(goal.pose.position.x)
    y = float(goal.pose.position.y)
    if not math.isfinite(x) or not math.isfinite(y):
        return None, "nonfinite_goal_position"
    if math.hypot(x, y) > max_goal_distance_from_origin:
        return None, "goal_out_of_configured_bounds"
    orientation_ok = normalize_quaternion(goal.pose.orientation)
    return goal, "action_goal_dispatched" if orientation_ok else "action_goal_dispatched_orientation_defaulted"


class GoalBridge(Node):
    def __init__(self):
        super().__init__("goal_bridge")
        self.runtime_mode = self.declare_parameter("runtime_mode", "real").value
        self.goal_topic = self.declare_parameter("exploration_goal_topic", "/a2/exploration/goal").value
        self.navigation_backend = self.declare_parameter("navigation_backend", "pose_topic_3d").value
        self.pose_goal_topic = self.declare_parameter("pose_goal_topic", "/a2/nav3/goal_pose").value
        self.legacy_pose_goal_topic = self.declare_parameter("legacy_pose_goal_topic", "/goal_pose_").value
        self.action_name = self.declare_parameter("navigate_action_name", "navigate_to_pose").value
        self.goal_timeout_sec = float(self.declare_parameter("goal_timeout_sec", 180.0).value)
        self.action_wait_timeout_sec = float(self.declare_parameter("action_wait_timeout_sec", 0.5).value)
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.require_map_frame = bool(self.declare_parameter("require_map_frame", True).value)
        self.max_goal_distance_from_origin = float(
            self.declare_parameter("max_goal_distance_from_origin", 120.0).value
        )
        self.cancel_active_goal_on_new_goal = bool(
            self.declare_parameter("cancel_active_goal_on_new_goal", False).value
        )
        self.status_pub = self.create_publisher(String, "/a2/nav2/status", 10)
        self.pose_goal_pub = self.create_publisher(PoseStamped, self.pose_goal_topic, 10)
        self.legacy_pose_goal_pub = None
        if self.legacy_pose_goal_topic and self.legacy_pose_goal_topic != self.pose_goal_topic:
            self.legacy_pose_goal_pub = self.create_publisher(
                PoseStamped, self.legacy_pose_goal_topic, 10
            )
        self.action_client = None
        self.navigate_type = None
        self.active_goal_handle = None
        self.active_goal_start_time = None
        try:
            from nav2_msgs.action import NavigateToPose

            if self.navigation_backend == "nav2":
                self.navigate_type = NavigateToPose
                self.action_client = ActionClient(self, NavigateToPose, self.action_name)
        except ImportError:
            if self.navigation_backend == "nav2":
                self.get_logger().error("nav2_msgs is not available. Goal bridge will stay idle until Nav2 is installed.")

        self.create_subscription(PoseStamped, self.goal_topic, self.on_goal, 10)
        self.create_timer(0.5, self.watchdog)

    def on_goal(self, msg):
        if self.navigation_backend == "pose_topic_3d":
            sanitized_goal, reason = self.sanitize_goal(msg)
            if sanitized_goal is None:
                self.publish_status(False, "goal_rejected", reason)
                return
            self.pose_goal_pub.publish(sanitized_goal)
            if self.legacy_pose_goal_pub is not None:
                self.legacy_pose_goal_pub.publish(sanitized_goal)
            self.publish_status(
                True,
                "goal_sent",
                f"{reason};backend=pose_topic_3d;topic={self.pose_goal_topic}",
            )
            return
        if self.action_client is None or self.navigate_type is None:
            self.publish_status(False, "bridge_unavailable", "nav2_msgs_missing")
            return
        if not self.action_client.wait_for_server(timeout_sec=self.action_wait_timeout_sec):
            self.get_logger().warn("NavigateToPose action server is not ready.")
            self.publish_status(False, "waiting_server", "navigate_action_not_ready")
            return
        sanitized_goal, reason = self.sanitize_goal(msg)
        if sanitized_goal is None:
            self.publish_status(False, "goal_rejected", reason)
            return
        if self.active_goal_handle is not None and not self.cancel_active_goal_on_new_goal:
            self.publish_status(True, "goal_active", "action_goal_already_running")
            return
        if self.active_goal_handle is not None and self.cancel_active_goal_on_new_goal:
            self.active_goal_handle.cancel_goal_async()

        goal = self.navigate_type.Goal()
        goal.pose = sanitized_goal
        future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        self.publish_status(True, "goal_sent", reason)

    def sanitize_goal(self, msg):
        goal = PoseStamped()
        goal.header = msg.header
        goal.pose = msg.pose
        if not goal.header.frame_id:
            goal.header.frame_id = self.map_frame
        if self.require_map_frame and goal.header.frame_id != self.map_frame:
            return None, f"bad_frame:{goal.header.frame_id}"
        if goal.header.stamp.sec == 0 and goal.header.stamp.nanosec == 0:
            goal.header.stamp = self.get_clock().now().to_msg()
        return validate_goal_contract(
            goal,
            map_frame=self.map_frame,
            require_map_frame=self.require_map_frame,
            max_goal_distance_from_origin=self.max_goal_distance_from_origin,
        )

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.active_goal_handle = None
            self.active_goal_start_time = None
            self.publish_status(False, "goal_error", f"action_goal_exception:{exc}")
            return
        if not goal_handle.accepted:
            self.active_goal_handle = None
            self.active_goal_start_time = None
            self.publish_status(False, "goal_rejected", "action_goal_rejected")
            return
        self.active_goal_handle = goal_handle
        self.active_goal_start_time = self.get_clock().now()
        self.publish_status(True, "goal_accepted", "action_goal_accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance_remaining = getattr(feedback, "distance_remaining", None)
        if distance_remaining is None:
            self.publish_status(True, "goal_running", "feedback")
            return
        self.publish_status(True, "goal_running", f"distance_remaining={float(distance_remaining):.2f}")

    def result_callback(self, future):
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            self.active_goal_handle = None
            self.active_goal_start_time = None
            self.publish_status(False, "goal_error", f"action_result_exception:{exc}")
            return
        self.active_goal_handle = None
        self.active_goal_start_time = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_status(True, "goal_succeeded", "action_goal_succeeded")
        elif status == GoalStatus.STATUS_CANCELED:
            self.publish_status(False, "goal_canceled", "action_goal_canceled")
        elif status == GoalStatus.STATUS_ABORTED:
            self.publish_status(False, "goal_aborted", "action_goal_aborted")
        else:
            self.publish_status(False, "goal_failed", f"action_status={status}")

    def watchdog(self):
        if self.active_goal_handle is None or self.active_goal_start_time is None:
            return
        age = (self.get_clock().now() - self.active_goal_start_time).nanoseconds * 1e-9
        if age <= self.goal_timeout_sec:
            return
        self.active_goal_handle.cancel_goal_async()
        self.publish_status(False, "goal_timeout", f"cancel_requested_after={age:.1f}s")

    def publish_status(self, ready, state, reason):
        status = (
            f"mode={self.runtime_mode};state={state};ready={str(bool(ready)).lower()};"
            f"reason={reason}"
        )
        self.status_pub.publish(String(data=status))


def main():
    rclpy.init()
    node = GoalBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
