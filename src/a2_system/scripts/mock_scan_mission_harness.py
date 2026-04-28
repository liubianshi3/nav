#!/usr/bin/env python3

from __future__ import annotations

import math
import time

import rclpy
from a2_interfaces.srv import ManageMap, SetMode
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover - depends on ROS environment
    NavigateToPose = None


def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class MockScanMissionHarness(Node):
    def __init__(self) -> None:
        super().__init__("mock_scan_mission_harness")
        self.map_topic = self.declare_parameter("map_topic", "/map").value
        self.pose_topic = self.declare_parameter("pose_topic", "/amcl_pose").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.localization_ok_topic = self.declare_parameter("localization_ok_topic", "/a2/localization_ok").value
        self.localization_status_topic = self.declare_parameter(
            "localization_status_topic", "/a2/localization/status"
        ).value
        self.real_report_topic = self.declare_parameter("real_report_topic", "/a2/real/report").value
        self.map_manager_status_topic = self.declare_parameter(
            "map_manager_status_topic", "/a2/map_manager/status"
        ).value
        self.active_map_topic = self.declare_parameter("active_map_topic", "/a2/map_manager/active_map").value
        self.nav_status_topic = self.declare_parameter("nav_status_topic", "/a2/nav2/status").value
        self.action_name = self.declare_parameter("navigate_action_name", "/navigate_to_pose").value
        self.manage_map_service = self.declare_parameter("manage_map_service", "/map_manager/manage_map").value
        self.set_mode_service = self.declare_parameter("set_mode_service", "/map_manager/set_mode").value
        self.result_mode = self.declare_parameter("result_mode", "succeeded").value
        self.result_delay_sec = float(self.declare_parameter("result_delay_sec", 0.2).value)
        self.publish_localization_ok = bool(self.declare_parameter("publish_localization_ok", True).value)
        self.map_width = int(self.declare_parameter("map_width", 20).value)
        self.map_height = int(self.declare_parameter("map_height", 20).value)
        self.map_resolution = float(self.declare_parameter("map_resolution", 0.1).value)
        self.robot_x = 0.2
        self.robot_y = 0.2
        self.robot_yaw = 0.0
        self.saved_maps: list[str] = []

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, latched_qos)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, latched_qos)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.localization_ok_pub = self.create_publisher(Bool, self.localization_ok_topic, 10)
        self.localization_status_pub = self.create_publisher(String, self.localization_status_topic, 10)
        self.real_report_pub = self.create_publisher(String, self.real_report_topic, 10)
        self.map_manager_status_pub = self.create_publisher(String, self.map_manager_status_topic, 10)
        self.active_map_pub = self.create_publisher(String, self.active_map_topic, 10)
        self.nav_status_pub = self.create_publisher(String, self.nav_status_topic, 10)

        self.create_service(ManageMap, self.manage_map_service, self.on_manage_map)
        self.create_service(SetMode, self.set_mode_service, self.on_set_mode)
        self.action_server = None
        if NavigateToPose is not None:
            self.action_server = ActionServer(
                self,
                NavigateToPose,
                self.action_name,
                execute_callback=self.execute_goal,
                goal_callback=self.on_goal,
                cancel_callback=self.on_cancel,
            )
        else:
            self.get_logger().error("nav2_msgs NavigateToPose is unavailable; mock action server disabled")
        self.create_timer(0.2, self.publish_state)

    def make_map(self) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.resolution = self.map_resolution
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = [0] * (self.map_width * self.map_height)
        return msg

    def make_pose(self) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.robot_x
        msg.pose.pose.position.y = self.robot_y
        z, w = yaw_to_quaternion(self.robot_yaw)
        msg.pose.pose.orientation.z = z
        msg.pose.pose.orientation.w = w
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.02
        return msg

    def publish_state(self) -> None:
        self.map_pub.publish(self.make_map())
        self.pose_pub.publish(self.make_pose())
        self.odom_pub.publish(Odometry())
        self.localization_ok_pub.publish(Bool(data=self.publish_localization_ok))
        localization_state = "ready" if self.publish_localization_ok else "lost"
        self.localization_status_pub.publish(
            String(data=f"mode=mock;state={localization_state};ready={str(self.publish_localization_ok).lower()};reason=mock")
        )
        self.real_report_pub.publish(String(data="mode=mock;state=ready;ready=true;reason=mock_harness"))
        self.map_manager_status_pub.publish(String(data="mode=mapping;state=ready;ready=true;reason=mock_harness"))
        self.active_map_pub.publish(String(data="mock_map"))

    def on_goal(self, goal_request):
        del goal_request
        if self.result_mode == "reject":
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        del goal_handle
        return CancelResponse.ACCEPT

    def execute_goal(self, goal_handle):
        start_time = time.monotonic()
        while time.monotonic() - start_time < self.result_delay_sec:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.nav_status_pub.publish(String(data="mode=mock;state=goal_canceled;ready=false;reason=cancel_requested"))
                return NavigateToPose.Result()
            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = max(0.0, self.result_delay_sec - (time.monotonic() - start_time))
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)

        if self.result_mode == "timeout":
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.nav_status_pub.publish(String(data="mode=mock;state=goal_canceled;ready=false;reason=timeout_cancel"))
                    return NavigateToPose.Result()
                time.sleep(0.1)
        if self.result_mode == "aborted":
            goal_handle.abort()
            self.nav_status_pub.publish(String(data="mode=mock;state=goal_aborted;ready=false;reason=mock_aborted"))
            return NavigateToPose.Result()

        self.robot_x = goal_handle.request.pose.pose.position.x
        self.robot_y = goal_handle.request.pose.pose.position.y
        orientation = goal_handle.request.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        goal_handle.succeed()
        self.nav_status_pub.publish(String(data="mode=mock;state=goal_succeeded;ready=true;reason=mock_succeeded"))
        return NavigateToPose.Result()

    def on_manage_map(self, request, response):
        if request.command == "save":
            self.saved_maps.append(request.map_id)
            response.success = True
            response.message = f"saved:{request.map_id}"
            response.map_ids = self.saved_maps
            return response
        if request.command == "list":
            response.success = True
            response.message = "ok"
            response.map_ids = self.saved_maps
            return response
        response.success = False
        response.message = f"unsupported command: {request.command}"
        response.map_ids = self.saved_maps
        return response

    def on_set_mode(self, request, response):
        response.success = request.mode in {"mapping", "navigation", "localization"}
        response.message = f"mode={request.mode}"
        return response


def main() -> None:
    rclpy.init()
    node = MockScanMissionHarness()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
