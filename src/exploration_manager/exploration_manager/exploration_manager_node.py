#!/usr/bin/env python3

from enum import Enum

import rclpy
from a2_interfaces.srv import ManageMap
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class ExploreState(str, Enum):
    IDLE = "idle"
    WAIT_LOCALIZATION = "wait_localization"
    EXPLORING = "exploring"
    STUCK_RECOVERY = "stuck_recovery"
    SAVING = "saving"
    COMPLETE = "complete"


class ExplorationManagerNode(Node):
    def __init__(self):
        super().__init__("exploration_manager")
        map_topic = self.declare_parameter("map_topic", "/map").value
        localization_ok_topic = self.declare_parameter("localization_ok_topic", "/a2/localization_ok").value
        self.goal_topic = self.declare_parameter("goal_topic", "/a2/exploration/goal").value
        self.state_topic = self.declare_parameter("state_topic", "/a2/exploration/state").value
        self.coverage_target = float(self.declare_parameter("coverage_target", 0.82).value)
        self.goal_republish_sec = float(self.declare_parameter("goal_republish_sec", 15.0).value)
        self.frontier_min_cluster = int(self.declare_parameter("frontier_min_cluster", 6).value)
        self.auto_start = bool(self.declare_parameter("auto_start", False).value)
        self.stuck_timeout_sec = float(self.declare_parameter("stuck_timeout_sec", 8.0).value)
        self.stuck_distance_m = float(self.declare_parameter("stuck_distance_m", 0.08).value)
        self.command_active_threshold = float(self.declare_parameter("command_active_threshold", 0.05).value)
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.command_topic = self.declare_parameter("command_topic", "/a2/command_limited").value
        self.latest_map = None
        self.localization_ok = False
        self.state = ExploreState.IDLE
        self.last_goal_time = self.get_clock().now()
        self.last_progress_time = self.get_clock().now()
        self.last_progress_position = None
        self.current_position = None
        self.command_active = False
        self.save_client = self.create_client(ManageMap, "/map_manager/manage_map")

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.coverage_pub = self.create_publisher(Float32, "/a2/exploration/coverage", 10)
        self.reason_pub = self.create_publisher(String, "/a2/exploration/reason", 10)
        self.create_subscription(OccupancyGrid, map_topic, self.on_map, 10)
        self.create_subscription(Bool, localization_ok_topic, self.on_localization, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        self.create_subscription(TwistStamped, self.command_topic, self.on_command, 20)
        self.create_timer(1.0, self.tick)
        self.publish_state()

    def on_map(self, msg):
        self.latest_map = msg

    def on_localization(self, msg):
        self.localization_ok = msg.data

    def on_odom(self, msg):
        self.current_position = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        if self.last_progress_position is None:
            self.last_progress_position = self.current_position
            self.last_progress_time = self.get_clock().now()
            return

        dx = self.current_position[0] - self.last_progress_position[0]
        dy = self.current_position[1] - self.last_progress_position[1]
        if (dx * dx + dy * dy) ** 0.5 >= self.stuck_distance_m:
            self.last_progress_position = self.current_position
            self.last_progress_time = self.get_clock().now()

    def on_command(self, msg):
        self.command_active = abs(msg.twist.linear.x) > self.command_active_threshold or abs(msg.twist.angular.z) > self.command_active_threshold

    def publish_state(self):
        self.state_pub.publish(String(data=self.state.value))

    def coverage(self):
        if self.latest_map is None or not self.latest_map.data:
            return 0.0
        known = sum(1 for value in self.latest_map.data if value >= 0)
        return known / float(len(self.latest_map.data))

    def tick(self):
        self.coverage_pub.publish(Float32(data=float(self.coverage())))
        if not self.auto_start and self.state == ExploreState.IDLE:
            self.publish_state()
            return

        if self.latest_map is None or not self.localization_ok:
            self.state = ExploreState.WAIT_LOCALIZATION
            self.reason_pub.publish(String(data="waiting_for_localization_or_map"))
            self.publish_state()
            return

        if self.coverage() >= self.coverage_target:
            self.state = ExploreState.SAVING
            self.reason_pub.publish(String(data="coverage_target_reached"))
            self.publish_state()
            self.request_save()
            self.state = ExploreState.COMPLETE
            self.publish_state()
            self.auto_start = False
            return

        now = self.get_clock().now()
        if self.command_active and self.current_position is not None:
            if (now - self.last_progress_time).nanoseconds * 1e-9 > self.stuck_timeout_sec:
                self.state = ExploreState.STUCK_RECOVERY
                self.reason_pub.publish(String(data="stuck_detected_replanning_frontier"))
                self.last_goal_time = now - Duration(seconds=self.goal_republish_sec + 1.0)
                self.last_progress_time = now

        if self.state in (ExploreState.IDLE, ExploreState.WAIT_LOCALIZATION):
            self.state = ExploreState.EXPLORING
            self.publish_state()
        elif self.state == ExploreState.STUCK_RECOVERY:
            self.state = ExploreState.EXPLORING
            self.publish_state()

        if (now - self.last_goal_time).nanoseconds * 1e-9 < self.goal_republish_sec:
            return

        frontier = self.find_frontier_goal()
        if frontier is None:
            self.get_logger().warn("No frontier found yet. Waiting for more map growth.")
            self.reason_pub.publish(String(data="no_frontier_available"))
            return
        self.goal_pub.publish(frontier)
        self.last_goal_time = now
        self.reason_pub.publish(String(data="frontier_goal_published"))

    def find_frontier_goal(self):
        msg = self.latest_map
        width = msg.info.width
        height = msg.info.height
        data = list(msg.data)

        def index_of(x, y):
            return y * width + x

        best = None
        best_score = -1
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                value = data[index_of(x, y)]
                if value != 0:
                    continue
                unknown_neighbors = 0
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    if data[index_of(x + dx, y + dy)] < 0:
                        unknown_neighbors += 1
                if unknown_neighbors >= self.frontier_min_cluster and unknown_neighbors > best_score:
                    best_score = unknown_neighbors
                    best = (x, y)

        if best is None:
            return None

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = msg.header.frame_id or "map"
        goal.pose.position.x = msg.info.origin.position.x + best[0] * msg.info.resolution
        goal.pose.position.y = msg.info.origin.position.y + best[1] * msg.info.resolution
        goal.pose.orientation.w = 1.0
        return goal

    def request_save(self):
        if not self.save_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Map save service unavailable.")
            return
        request = ManageMap.Request()
        request.command = "save"
        request.map_id = ""
        self.save_client.call_async(request)


def main():
    rclpy.init()
    node = ExplorationManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
