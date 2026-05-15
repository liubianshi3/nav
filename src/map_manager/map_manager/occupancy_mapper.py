#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OccupancyMapper(Node):
    def __init__(self):
        super().__init__("occupancy_mapper")
        self.map_topic = self.declare_parameter("map_topic", "/map").value
        self.pointcloud_topic = self.declare_parameter("pointcloud_topic", "/jt128/front/points").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.width = int(self.declare_parameter("width", 300).value)
        self.height = int(self.declare_parameter("height", 300).value)
        self.resolution = float(self.declare_parameter("resolution", 0.1).value)
        self.origin_x = float(self.declare_parameter("origin_x", -15.0).value)
        self.origin_y = float(self.declare_parameter("origin_y", -15.0).value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 2.0).value)
        self.point_stride = max(1, int(self.declare_parameter("point_stride", 3).value))
        self.min_range = float(self.declare_parameter("min_range", 0.3).value)
        self.max_range = float(self.declare_parameter("max_range", 15.0).value)
        self.obstacle_min_z = float(self.declare_parameter("obstacle_min_z", 0.05).value)
        self.obstacle_max_z = float(self.declare_parameter("obstacle_max_z", 1.8).value)
        self.hit_log_odds = float(self.declare_parameter("hit_log_odds", 0.9).value)
        self.miss_log_odds = float(self.declare_parameter("miss_log_odds", 0.2).value)
        self.min_log_odds = float(self.declare_parameter("min_log_odds", -2.0).value)
        self.max_log_odds = float(self.declare_parameter("max_log_odds", 3.5).value)
        self.occupied_log_odds = float(self.declare_parameter("occupied_log_odds", 0.7).value)
        self.clear_robot_radius_m = float(self.declare_parameter("clear_robot_radius_m", 0.35).value)
        self.mark_free_space = bool(self.declare_parameter("mark_free_space", True).value)
        self.lidar_offset_xyz = [
            float(value)
            for value in self.declare_parameter("lidar_offset_xyz", [0.32, 0.0, 0.24]).value
        ]
        self.lidar_offset_yaw = float(self.declare_parameter("lidar_offset_yaw", 0.0).value)

        self.latest_pose = None
        self.latest_stamp = None
        self.log_odds = [0.0] * (self.width * self.height)
        self.observed = [False] * (self.width * self.height)

        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self.on_cloud, 10)
        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self.publish_map)

    def on_odom(self, msg):
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.latest_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
            yaw,
        )
        self.latest_stamp = msg.header.stamp

    def on_cloud(self, msg):
        if self.latest_pose is None:
            return

        base_x, base_y, base_z, base_yaw = self.latest_pose
        cos_yaw = math.cos(base_yaw)
        sin_yaw = math.sin(base_yaw)
        sensor_x = base_x + cos_yaw * self.lidar_offset_xyz[0] - sin_yaw * self.lidar_offset_xyz[1]
        sensor_y = base_y + sin_yaw * self.lidar_offset_xyz[0] + cos_yaw * self.lidar_offset_xyz[1]
        sensor_z = base_z + self.lidar_offset_xyz[2]
        sensor_yaw = base_yaw + self.lidar_offset_yaw

        sensor_cell = self.world_to_cell(sensor_x, sensor_y)
        if sensor_cell is None:
            return

        self.clear_robot_footprint(base_x, base_y)

        cos_sensor = math.cos(sensor_yaw)
        sin_sensor = math.sin(sensor_yaw)

        for index, point in enumerate(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            if index % self.point_stride != 0:
                continue

            px, py, pz = [float(value) for value in point]
            radius = math.sqrt(px * px + py * py + pz * pz)
            if radius < self.min_range or radius > self.max_range:
                continue

            map_x = sensor_x + cos_sensor * px - sin_sensor * py
            map_y = sensor_y + sin_sensor * px + cos_sensor * py
            map_z = sensor_z + pz
            hit_cell = self.world_to_cell(map_x, map_y)
            if hit_cell is None:
                continue

            if self.mark_free_space:
                self.mark_ray_free(sensor_cell, hit_cell)

            if self.obstacle_min_z <= map_z <= self.obstacle_max_z:
                self.update_cell(hit_cell[0], hit_cell[1], self.hit_log_odds)

    def world_to_cell(self, x, y):
        cell_x = int(math.floor((x - self.origin_x) / self.resolution))
        cell_y = int(math.floor((y - self.origin_y) / self.resolution))
        if 0 <= cell_x < self.width and 0 <= cell_y < self.height:
            return cell_x, cell_y
        return None

    def index_of(self, cell_x, cell_y):
        return cell_y * self.width + cell_x

    def update_cell(self, cell_x, cell_y, delta):
        index = self.index_of(cell_x, cell_y)
        self.observed[index] = True
        self.log_odds[index] = max(self.min_log_odds, min(self.max_log_odds, self.log_odds[index] + delta))

    def clear_robot_footprint(self, robot_x, robot_y):
        radius_cells = max(1, int(math.ceil(self.clear_robot_radius_m / self.resolution)))
        robot_cell = self.world_to_cell(robot_x, robot_y)
        if robot_cell is None:
            return
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                cell_x = robot_cell[0] + dx
                cell_y = robot_cell[1] + dy
                if 0 <= cell_x < self.width and 0 <= cell_y < self.height:
                    self.update_cell(cell_x, cell_y, -self.miss_log_odds)

    def mark_ray_free(self, start_cell, end_cell):
        x0, y0 = start_cell
        x1, y1 = end_cell
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        while True:
            if x0 == x1 and y0 == y1:
                break
            if 0 <= x0 < self.width and 0 <= y0 < self.height:
                self.update_cell(x0, y0, -self.miss_log_odds)
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.frame_id
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.orientation.w = 1.0

        data = [-1] * (self.width * self.height)
        for index, seen in enumerate(self.observed):
            if not seen:
                continue
            data[index] = 100 if self.log_odds[index] >= self.occupied_log_odds else 0

        grid.data = data
        self.map_pub.publish(grid)


def main():
    rclpy.init()
    node = OccupancyMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
