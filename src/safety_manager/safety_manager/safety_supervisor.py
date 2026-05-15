#!/usr/bin/env python3

import rclpy
from a2_interfaces.msg import RobotState
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__("safety_supervisor")
        self.runtime_mode = self.declare_parameter("runtime_mode", "real").value
        lidar_topic = self.declare_parameter("lidar_topic", "/jt128/front/points").value
        robot_state_topic = self.declare_parameter("robot_state_topic", "/robot_state").value
        map_topic = self.declare_parameter("map_topic", "/map").value
        localization_status_topic = self.declare_parameter("localization_status_topic", "/a2/localization_ok").value
        self.allow_motion_topic = self.declare_parameter("allow_motion_topic", "/a2/allow_motion").value
        self.map_ready_topic = self.declare_parameter("map_ready_topic", "/a2/map_ready").value
        self.lidar_ready_topic = self.declare_parameter("lidar_ready_topic", "/a2/lidar_ready").value
        self.map_asset_ready_topic = self.declare_parameter("map_asset_ready_topic", "/a2/map_asset_ready").value
        self.localization_ready_topic = self.declare_parameter(
            "localization_ready_topic", "/a2/safety/localization_ok"
        ).value
        self.estop_topic = self.declare_parameter("estop_topic", "/a2/estop").value
        self.lidar_timeout_sec = float(self.declare_parameter("lidar_timeout_sec", 0.5).value)
        self.state_timeout_sec = float(self.declare_parameter("state_timeout_sec", 0.5).value)
        self.map_timeout_sec = float(self.declare_parameter("map_timeout_sec", 5.0).value)
        self.latch_map_ready = bool(self.declare_parameter("latch_map_ready", False).value)
        self.map_transient_local = bool(
            self.declare_parameter("map_transient_local", self.latch_map_ready).value
        )
        self.map_representation = self.declare_parameter("map_representation", "occupancy_grid_2d").value
        self.localization_mode = self.declare_parameter("localization_mode", "ndt").value
        self.require_map = bool(self.declare_parameter("require_map", True).value)
        self.require_localization = bool(self.declare_parameter("require_localization", True).value)
        ndt_health_topic = self.declare_parameter("ndt_health_topic", "/a2/ndt/healthy").value
        self.require_ndt_health = bool(self.declare_parameter("require_ndt_health", True).value)

        self.last_lidar = None
        self.last_state = None
        self.last_map = None
        self.localization_ok = False
        self.ndt_healthy = not self.require_ndt_health

        self.allow_motion_pub = self.create_publisher(Bool, self.allow_motion_topic, 10)
        self.map_ready_pub = self.create_publisher(Bool, self.map_ready_topic, 10)
        self.lidar_ready_pub = self.create_publisher(Bool, self.lidar_ready_topic, 10)
        self.map_asset_ready_pub = self.create_publisher(Bool, self.map_asset_ready_topic, 10)
        self.localization_ready_pub = self.create_publisher(Bool, self.localization_ready_topic, 10)
        self.estop_pub = self.create_publisher(Bool, self.estop_topic, 10)
        self.reason_pub = self.create_publisher(String, "/a2/safety/reason", 10)
        self.status_pub = self.create_publisher(String, "/a2/safety/status", 10)
        self.last_status_text = ""
        map_qos = (
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            if self.map_transient_local
            else 10
        )

        self.create_subscription(PointCloud2, lidar_topic, self.on_lidar, 10)
        self.create_subscription(RobotState, robot_state_topic, self.on_state, 20)
        if self.map_representation == "pointcloud_map_3d":
            self.create_subscription(PointCloud2, map_topic, self.on_map, 10)
        else:
            self.create_subscription(OccupancyGrid, map_topic, self.on_map, map_qos)
        self.create_subscription(Bool, localization_status_topic, self.on_localization, 10)
        if ndt_health_topic:
            self.create_subscription(Bool, ndt_health_topic, self.on_ndt_health, 10)
            self.get_logger().info(f"NDT health check enabled: {ndt_health_topic}")
        self.create_timer(0.2, self.evaluate)

    def on_lidar(self, _msg):
        self.last_lidar = self.get_clock().now()

    def on_state(self, msg):
        self.last_state = self.get_clock().now()
        del msg

    def on_map(self, _msg):
        self.last_map = self.get_clock().now()

    def on_localization(self, msg):
        self.localization_ok = msg.data

    def on_ndt_health(self, msg):
        self.ndt_healthy = msg.data

    def fresh(self, stamp, timeout):
        return stamp is not None and (self.get_clock().now() - stamp).nanoseconds * 1e-9 <= timeout

    def evaluate(self):
        lidar_ok = self.fresh(self.last_lidar, self.lidar_timeout_sec)
        state_ok = self.fresh(self.last_state, self.state_timeout_sec)
        map_asset_ready = self.last_map is not None if self.latch_map_ready else self.fresh(
            self.last_map, self.map_timeout_sec
        )
        map_ready = map_asset_ready if self.require_map else True
        localization_ready = self.localization_ok or not self.require_localization
        ndt_ready = self.ndt_healthy or not self.require_ndt_health
        allow_motion = lidar_ok and state_ok and map_ready and localization_ready and ndt_ready
        estop = not lidar_ok or not state_ok

        reason = []
        if not lidar_ok:
            reason.append("lidar_stale")
        if not state_ok:
            reason.append("robot_state_stale")
        if not map_ready:
            reason.append("map_not_ready")
        elif not self.require_map:
            reason.append("map_not_required")
        if not localization_ready:
            reason.append("localization_not_ready")
        if not ndt_ready:
            reason.append("ndt_unhealthy")

        self.allow_motion_pub.publish(Bool(data=allow_motion))
        self.map_ready_pub.publish(Bool(data=map_ready))
        self.lidar_ready_pub.publish(Bool(data=lidar_ok))
        self.map_asset_ready_pub.publish(Bool(data=map_asset_ready))
        self.localization_ready_pub.publish(Bool(data=localization_ready))
        self.estop_pub.publish(Bool(data=estop))
        reason_text = ",".join(reason) if reason else "ok"
        self.reason_pub.publish(String(data=reason_text))
        self.publish_status(
            allow_motion,
            estop,
            reason_text,
            lidar_ok,
            map_asset_ready,
            map_ready,
            localization_ready,
            ndt_ready,
        )

    def publish_status(
        self,
        allow_motion,
        estop,
        reason,
        lidar_ready,
        map_asset_ready,
        map_ready,
        localization_ready,
        ndt_ready,
    ):
        mode = self.runtime_mode
        state = "allow_motion" if allow_motion else "blocked"
        status = (
            f"mode={mode};state={state};ready={str(bool(allow_motion)).lower()};"
            f"reason={reason};estop={str(bool(estop)).lower()};"
            f"localization_mode={self.localization_mode};"
            f"lidar_ready={str(bool(lidar_ready)).lower()};"
            f"map_asset_ready={str(bool(map_asset_ready)).lower()};"
            f"map_ready={str(bool(map_ready)).lower()};"
            f"localization_ok={str(bool(localization_ready)).lower()};"
            f"ndt_ready={str(bool(ndt_ready)).lower()};"
            f"require_map={str(bool(self.require_map)).lower()};"
            f"require_localization={str(bool(self.require_localization)).lower()};"
            f"require_ndt_health={str(bool(self.require_ndt_health)).lower()}"
        )
        self.status_pub.publish(String(data=status))
        if status != self.last_status_text:
            self.get_logger().info(f"Safety status changed: {status}")
            self.last_status_text = status


def main():
    rclpy.init()
    node = SafetySupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
