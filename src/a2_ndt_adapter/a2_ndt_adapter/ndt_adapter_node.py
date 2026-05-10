import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from autoware_internal_debug_msgs.msg import Int32Stamped
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

def score_is_acceptable(score: float | None, threshold: float, min_is_good: bool) -> bool:
    if score is None or not math.isfinite(score):
        return False
    return score >= threshold if min_is_good else score <= threshold

def clamp_map_radius(radius: float, min_radius: float, max_radius: float) -> float:
    if not math.isfinite(radius) or radius <= 0.0:
        return max_radius
    return min(max(radius, min_radius), max_radius)

def select_points_for_area(
    points: np.ndarray,
    center_x: float,
    center_y: float,
    radius: float,
    margin: float,
    max_points: int,
) -> np.ndarray:
    if points.size == 0:
        return points.reshape((0, 3))
    effective_radius = max(0.0, radius + margin)
    dx = points[:, 0] - float(center_x)
    dy = points[:, 1] - float(center_y)
    selected = points[(dx * dx + dy * dy) <= effective_radius * effective_radius]
    if max_points > 0 and selected.shape[0] > max_points:
        step = int(math.ceil(selected.shape[0] / max_points))
        selected = selected[::step][:max_points]
    return selected

def make_map_cell_id(prefix: str, center_x: float, center_y: float, radius: float) -> str:
    return f"{prefix}_{center_x:.1f}_{center_y:.1f}_r{radius:.1f}".replace("-", "m").replace(".", "p")

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
        self.declare_parameter('score_topic', 'transform_probability')
        self.declare_parameter('iteration_topic', 'iteration_num')
        self.declare_parameter('score_threshold', 0.5)
        self.declare_parameter('score_min_is_good', True)
        self.declare_parameter('odom_timeout_sec', 1.0)
        self.declare_parameter('score_timeout_sec', 1.0)
        self.declare_parameter('max_map_to_odom_translation_step', 1.0)
        self.declare_parameter('max_map_to_odom_rotation_step_deg', 20.0)
        self.declare_parameter('map_service_min_radius', 1.0)
        self.declare_parameter('map_service_max_radius', 150.0)
        self.declare_parameter('map_service_margin_m', 5.0)
        self.declare_parameter('map_service_max_points', 200000)
        self.declare_parameter('map_cell_id_prefix', 'a2_map_cell')
        
        # State
        self.last_odom_to_base = None
        self.map_to_odom = np.eye(4)
        self.has_seed = False
        self.last_score = -1.0
        self.last_score_stamp = None
        self.last_odom_stamp = None
        self.last_iteration_num = None
        self.cached_map = None
        self.cached_map_frame = self.get_parameter('map_frame').value
        self.cached_map_points = np.empty((0, 3), dtype=np.float32)
        self.map_parse_error = ''
        self.last_map_request = 'none'
        self.last_map_cell_id = 'none'
        self.last_map_returned_points = 0
        
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
        self.create_subscription(Float32Stamped, self.get_parameter('score_topic').value, self.on_score, 10)
        self.create_subscription(Int32Stamped, self.get_parameter('iteration_topic').value, self.on_iteration, 10)

        # Service Server for Map Loading
        self.map_service = self.create_service(GetDifferentialPointCloudMap, 'pcd_loader_service', self.handle_get_map)

        # Timer for status
        self.create_timer(1.0, self.publish_periodic_status)

        self.get_logger().info("A2 NDT Adapter initialized.")

    def on_odom(self, msg: Odometry):
        self.last_odom_to_base = pose_to_matrix(msg.pose.pose.position, msg.pose.pose.orientation)
        self.last_odom_stamp = self.get_clock().now()

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
        if self.last_odom_to_base is None:
            self.publish_status(False, "rejected", "ndt_pose_without_odom")
            return
        if not self.odom_is_fresh():
            self.publish_status(False, "rejected", "odom_stale")
            return
        if not self.score_is_fresh():
            self.publish_status(False, "rejected", "score_stale")
            return
        if not self.current_score_is_acceptable():
            self.publish_status(False, "rejected", "score_below_threshold")
            return

        map_to_base = pose_to_matrix(msg.pose.pose.position, msg.pose.pose.orientation)
        candidate_map_to_odom = map_to_base @ np.linalg.inv(self.last_odom_to_base)
        if self.has_seed and not self.correction_step_is_bounded(candidate_map_to_odom):
            self.publish_status(False, "rejected", "map_to_odom_jump")
            return

        self.map_to_odom = candidate_map_to_odom
        self.has_seed = True

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
        # self.tf_broadcaster.sendTransform(tf_msg) # EKF now handles TF publishing

        # Relay to A2 interface
        self.pose_pub.publish(msg)

        # Publish status
        self.publish_status(True, "ready", "converged")

    def on_initial_pose(self, msg: PoseWithCovarianceStamped):
        if self.last_odom_to_base is None:
            self.get_logger().warn("Received initialpose but no odom available yet.")
            self.publish_status(False, "waiting_odom", "initialpose_without_odom")
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
        self.cached_map_frame = msg.header.frame_id or self.get_parameter('map_frame').value
        try:
            points = [
                (float(x), float(y), float(z))
                for x, y, z in point_cloud2.read_points(
                    msg,
                    field_names=("x", "y", "z"),
                    skip_nans=True,
                )
                if math.isfinite(float(x)) and math.isfinite(float(y)) and math.isfinite(float(z))
            ]
            self.cached_map_points = np.asarray(points, dtype=np.float32).reshape((-1, 3))
            self.map_parse_error = ''
            self.get_logger().info(f"Map cached for NDT service with {self.cached_map_points.shape[0]} xyz points.")
        except Exception as exc:  # pragma: no cover - defensive for malformed robot maps.
            self.cached_map_points = np.empty((0, 3), dtype=np.float32)
            self.map_parse_error = str(exc)
            self.get_logger().error(f"Failed to parse cached pointcloud map: {exc}")

    def on_score(self, msg: Float32Stamped):
        self.last_score = float(msg.data)
        self.last_score_stamp = self.get_clock().now()

    def on_iteration(self, msg: Int32Stamped):
        self.last_iteration_num = int(msg.data)

    def handle_get_map(self, request, response):
        center_x = float(request.area.center_x)
        center_y = float(request.area.center_y)
        radius = clamp_map_radius(
            float(request.area.radius),
            float(self.get_parameter('map_service_min_radius').value),
            float(self.get_parameter('map_service_max_radius').value),
        )
        prefix = str(self.get_parameter('map_cell_id_prefix').value)
        cell_id = make_map_cell_id(prefix, center_x, center_y, radius)
        self.last_map_request = f"cx:{center_x:.2f},cy:{center_y:.2f},r:{radius:.2f}"
        self.last_map_cell_id = cell_id
        self.get_logger().info(f"NDT requested map area {self.last_map_request}")

        if self.cached_map is None or self.cached_map_points.size == 0:
            self.get_logger().warn("Map requested but none cached yet.")
            response.header.frame_id = self.get_parameter('map_frame').value
            self.last_map_returned_points = 0
            return response

        cached_ids = set(request.cached_ids)
        response.header = self.cached_map.header
        response.header.frame_id = self.cached_map_frame
        if cell_id in cached_ids:
            response.new_pointcloud_with_ids = []
            response.ids_to_remove = []
            self.last_map_returned_points = 0
            return response

        selected = select_points_for_area(
            self.cached_map_points,
            center_x,
            center_y,
            radius,
            float(self.get_parameter('map_service_margin_m').value),
            int(self.get_parameter('map_service_max_points').value),
        )
        if selected.size == 0:
            response.new_pointcloud_with_ids = []
            response.ids_to_remove = list(cached_ids)
            self.last_map_returned_points = 0
            return response

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = point_cloud2.create_cloud(response.header, fields, selected.astype(np.float32))
        cloud.is_dense = True

        cell = PointCloudMapCellWithID()
        cell.cell_id = cell_id
        cell.pointcloud = cloud
        cell.metadata = PointCloudMapCellMetaData()
        cell.metadata.min_x = float(np.min(selected[:, 0]))
        cell.metadata.min_y = float(np.min(selected[:, 1]))
        cell.metadata.max_x = float(np.max(selected[:, 0]))
        cell.metadata.max_y = float(np.max(selected[:, 1]))

        response.new_pointcloud_with_ids = [cell]
        response.ids_to_remove = [cached_id for cached_id in cached_ids if cached_id != cell_id]
        self.last_map_returned_points = int(selected.shape[0])
        return response

    def odom_is_fresh(self) -> bool:
        if self.last_odom_stamp is None:
            return False
        age = (self.get_clock().now() - self.last_odom_stamp).nanoseconds * 1e-9
        return age <= float(self.get_parameter('odom_timeout_sec').value)

    def score_is_fresh(self) -> bool:
        if self.last_score_stamp is None:
            return False
        age = (self.get_clock().now() - self.last_score_stamp).nanoseconds * 1e-9
        return age <= float(self.get_parameter('score_timeout_sec').value)

    def current_score_is_acceptable(self) -> bool:
        return score_is_acceptable(
            self.last_score,
            float(self.get_parameter('score_threshold').value),
            bool(self.get_parameter('score_min_is_good').value),
        )

    def correction_step_is_bounded(self, candidate_map_to_odom: np.ndarray) -> bool:
        delta = candidate_map_to_odom @ np.linalg.inv(self.map_to_odom)
        translation = float(np.linalg.norm(delta[:3, 3]))
        rotation_trace = (float(np.trace(delta[:3, :3])) - 1.0) * 0.5
        rotation = math.acos(max(-1.0, min(1.0, rotation_trace)))
        max_translation = float(self.get_parameter('max_map_to_odom_translation_step').value)
        max_rotation = math.radians(float(self.get_parameter('max_map_to_odom_rotation_step_deg').value))
        return translation <= max_translation and rotation <= max_rotation

    def publish_periodic_status(self):
        if not self.has_seed:
            self.publish_status(False, "waiting_seed", "send_initialpose")
        elif self.last_odom_to_base is None:
            self.publish_status(False, "waiting_odom", "no_dlio_odom")
        elif not self.score_is_fresh():
            self.publish_status(False, "waiting_score", "no_recent_ndt_score")

    def publish_status(self, ready, state, reason):
        score = self.last_score if self.last_score is not None else -1.0
        status_parts = [
            f"state={state}",
            f"ready={str(bool(ready)).lower()}",
            f"reason={reason}",
            f"matcher=autoware_ndt",
            f"score={score:.3f}",
            f"score_threshold={float(self.get_parameter('score_threshold').value):.3f}",
            f"iteration_num={self.last_iteration_num if self.last_iteration_num is not None else -1}",
            f"map_ready={str(bool(self.cached_map_points.size > 0 and not self.map_parse_error)).lower()}",
            f"map_points={int(self.cached_map_points.shape[0])}",
            f"last_map_request={self.last_map_request}",
            f"last_map_cell_id={self.last_map_cell_id}",
            f"last_map_returned_points={self.last_map_returned_points}",
            f"live_cloud_topic={self.get_parameter('live_cloud_topic').value}",
            f"odom_topic={self.get_parameter('odom_topic').value}",
        ]
        if self.map_parse_error:
            status_parts.append(f"map_error={self.map_parse_error}")
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
