import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster
from autoware_map_msgs.srv import GetDifferentialPointCloudMap
from autoware_map_msgs.msg import PointCloudMapCellWithID, PointCloudMapCellMetaData
from autoware_internal_debug_msgs.msg import Float32Stamped

def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm < 1e-9:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return q / norm

def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    x, y, z, w = normalize_quaternion(np.array([x, y, z, w], dtype=np.float64))
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )

def matrix_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            w = (rotation[2, 1] - rotation[1, 2]) / s
            x = 0.25 * s
            y = (rotation[0, 1] + rotation[1, 0]) / s
            z = (rotation[0, 2] + rotation[2, 0]) / s
        elif axis == 1:
            s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            w = (rotation[0, 2] - rotation[2, 0]) / s
            x = (rotation[0, 1] + rotation[1, 0]) / s
            y = 0.25 * s
            z = (rotation[1, 2] + rotation[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            w = (rotation[1, 0] - rotation[0, 1]) / s
            x = (rotation[0, 2] + rotation[2, 0]) / s
            y = (rotation[1, 2] + rotation[2, 1]) / s
            z = 0.25 * s
    q = normalize_quaternion(np.array([x, y, z, w], dtype=np.float64))
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])

def pose_to_matrix(position, orientation) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_to_matrix(
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    matrix[:3, 3] = [float(position.x), float(position.y), float(position.z)]
    return matrix

class A2NdtAdapter(Node):
    def __init__(self):
        super().__init__('a2_ndt_adapter')
        
        # Parameters
        self.declare_parameter('live_cloud_topic', '/jt128/front/points')
        self.declare_parameter('odom_topic', '/jt128/dlio/odom')
        self.declare_parameter('map_topic', '/a2/map/pointcloud_3d')
        self.declare_parameter('pose_topic', '/a2/relocalization/pose')
        self.declare_parameter('status_topic', '/a2/relocalization/status')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        
        # Internal topics (to/from NDT)
        self.declare_parameter('ndt_pose_topic', 'ndt_pose_with_covariance')
        self.declare_parameter('ndt_initial_pose_topic', 'ekf_pose_with_covariance')
        self.declare_parameter('ndt_score_topic', 'transform_probability')
        
        # State
        self.last_odom_to_base = None
        self.map_to_odom = np.eye(4)
        self.has_seed = False
        self.last_score = -1.0
        self.cached_map = None
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.get_parameter('pose_topic').value, 10)
        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        self.ndt_initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.get_parameter('ndt_initial_pose_topic').value, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriptions
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, 20)
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter('ndt_pose_topic').value, self.on_ndt_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter('initial_pose_topic').value, self.on_initial_pose, 10)
        self.create_subscription(PointCloud2, self.get_parameter('map_topic').value, self.on_map, 10)
        self.create_subscription(Float32Stamped, self.get_parameter('ndt_score_topic').value, self.on_score, 10)
        
        # Service Server for Map Loading
        self.map_service = self.create_service(GetDifferentialPointCloudMap, 'pcd_loader_service', self.handle_get_map)
        
        # Timer for status
        self.create_timer(1.0, self.publish_periodic_status)
        
        self.get_logger().info("A2 NDT Adapter initialized.")

    def on_odom(self, msg: Odometry):
        self.last_odom_to_base = pose_to_matrix(msg.pose.pose.position, msg.pose.pose.orientation)
        
        # If we have a seed, provide the initial guess to NDT
        if self.has_seed:
            map_to_base = self.map_to_odom @ self.last_odom_to_base
            
            guess = PoseWithCovarianceStamped()
            guess.header = msg.header
            guess.header.frame_id = self.get_parameter('map_frame').value
            guess.pose.pose.position.x = float(map_to_base[0, 3])
            guess.pose.pose.position.y = float(map_to_base[1, 3])
            guess.pose.pose.position.z = float(map_to_base[2, 3])
            qx, qy, qz, qw = matrix_to_quaternion(map_to_base[:3, :3])
            guess.pose.pose.orientation.x = qx
            guess.pose.pose.orientation.y = qy
            guess.pose.pose.orientation.z = qz
            guess.pose.pose.orientation.w = qw
            # Copy covariance or use fixed
            guess.pose.covariance = msg.pose.covariance
            
            self.ndt_initial_pose_pub.publish(guess)

    def on_ndt_pose(self, msg: PoseWithCovarianceStamped):
        map_to_base = pose_to_matrix(msg.pose.pose.position, msg.pose.pose.orientation)
        
        if self.last_odom_to_base is not None:
            # Update map_to_odom
            # map_to_base = map_to_odom * odom_to_base
            # map_to_odom = map_to_base * odom_to_base.inverse()
            self.map_to_odom = map_to_base @ np.linalg.inv(self.last_odom_to_base)
            self.has_seed = True
            
            # Broadcast TF
            tf_msg = TransformStamped()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.header.frame_id = self.get_parameter('map_frame').value
            tf_msg.child_frame_id = self.get_parameter('odom_frame').value
            tf_msg.transform.translation.x = float(self.map_to_odom[0, 3])
            tf_msg.transform.translation.y = float(self.map_to_odom[1, 3])
            tf_msg.transform.translation.z = float(self.map_to_odom[2, 3])
            qx, qy, qz, qw = matrix_to_quaternion(self.map_to_odom[:3, :3])
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)
            
        # Relay to A2 interface
        self.pose_pub.publish(msg)
        
        # Publish status
        self.publish_status(True, "ready", "converged")

    def on_initial_pose(self, msg: PoseWithCovarianceStamped):
        if self.last_odom_to_base is None:
            self.get_logger().warn("Received initialpose but no odom available yet.")
            return
            
        map_to_base = pose_to_matrix(msg.pose.pose.position, msg.pose.pose.orientation)
        self.map_to_odom = map_to_base @ np.linalg.inv(self.last_odom_to_base)
        self.has_seed = True
        self.get_logger().info("Initial pose set, seeding NDT.")
        
        # Also relay to NDT's initial pose topic
        self.ndt_initial_pose_pub.publish(msg)
        self.publish_status(True, "seeded", "initialpose_received")

    def on_map(self, msg: PointCloud2):
        self.cached_map = msg
        self.get_logger().info("Map cached for NDT service.")

    def on_score(self, msg: Float32Stamped):
        self.last_score = msg.data

    def handle_get_map(self, request, response):
        self.get_logger().info(f"NDT requested map area center=({request.area.center_x}, {request.area.center_y})")
        if self.cached_map is None:
            self.get_logger().warn("Map requested but none cached yet.")
            return response
            
        cell = PointCloudMapCellWithID()
        cell.cell_id = "static_map"
        cell.pointcloud = self.cached_map
        # Fill metadata with large bounds since we return the whole map as one cell
        cell.metadata = PointCloudMapCellMetaData()
        cell.metadata.min_x = -10000.0
        cell.metadata.min_y = -10000.0
        cell.metadata.max_x = 10000.0
        cell.metadata.max_y = 10000.0
        
        response.header = self.cached_map.header
        response.new_pointcloud_with_ids = [cell]
        return response

    def publish_periodic_status(self):
        if not self.has_seed:
            self.publish_status(False, "waiting_seed", "send_initialpose")
        elif self.last_odom_to_base is None:
            self.publish_status(False, "waiting_odom", "no_dlio_odom")

    def publish_status(self, ready, state, reason):
        status_parts = [
            f"state={state}",
            f"ready={str(bool(ready)).lower()}",
            f"reason={reason}",
            f"matcher=autoware_ndt",
            f"score={self.last_score:.3f}",
        ]
        status = ";".join(status_parts)
        self.status_pub.publish(String(data=status))

def main(args=None):
    rclpy.init(args=args)
    node = A2NdtAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
