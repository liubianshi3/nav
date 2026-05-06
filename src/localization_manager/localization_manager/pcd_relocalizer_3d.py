#!/usr/bin/env python3

from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


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


def xyz_rpy_to_matrix(xyz: list[float] | np.ndarray, rpy: list[float] | np.ndarray) -> np.ndarray:
    roll, pitch, yaw = [float(value) for value in rpy]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rz @ ry @ rx
    matrix[:3, 3] = np.array(xyz, dtype=np.float64)
    return matrix


def xyz_rotation_matrix_to_matrix(xyz: list[float], rotation_matrix: list[float]) -> np.ndarray:
    if len(rotation_matrix) != 9:
        raise ValueError("rotation_matrix must contain 9 values")
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.array(rotation_matrix, dtype=np.float64).reshape((3, 3))
    matrix[:3, 3] = np.array(xyz, dtype=np.float64)
    return matrix


def transform_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def rotation_angle(rotation: np.ndarray) -> float:
    value = (float(np.trace(rotation)) - 1.0) * 0.5
    return math.acos(max(-1.0, min(1.0, value)))


def voxel_downsample(points: np.ndarray, leaf_size: float, max_points: int) -> np.ndarray:
    if points.size == 0:
        return points
    if leaf_size > 0.0:
        keys = np.floor(points / leaf_size).astype(np.int64)
        _, indices = np.unique(keys, axis=0, return_index=True)
        points = points[np.sort(indices)]
    if max_points > 0 and len(points) > max_points:
        step = max(1, len(points) // max_points)
        points = points[::step][:max_points]
    return points.astype(np.float64, copy=False)


class NdtVoxelGrid:
    def __init__(self, points: np.ndarray, resolution: float, min_points_per_voxel: int, cov_reg: float):
        self.resolution = resolution
        self.voxels: dict[tuple[int, int, int], tuple[np.ndarray, np.ndarray]] = {}

        if points.size == 0:
            return

        keys = np.floor(points / resolution).astype(np.int64)
        unique_keys, inverse_indices = np.unique(keys, axis=0, return_inverse=True)
        counts = np.bincount(inverse_indices)
        valid_indices = np.where(counts >= min_points_per_voxel)[0]

        reg_matrix = np.eye(3, dtype=np.float64) * cov_reg

        for idx in valid_indices:
            pts_in_voxel = points[inverse_indices == idx]
            mean = np.mean(pts_in_voxel, axis=0)
            cov = np.cov(pts_in_voxel, rowvar=False)
            if not isinstance(cov, np.ndarray) or cov.ndim != 2:
                continue
            cov += reg_matrix
            try:
                inv_cov = np.linalg.inv(cov)
                self.voxels[tuple(unique_keys[idx])] = (mean, inv_cov)
            except np.linalg.LinAlgError:
                pass

    def query(self, points: np.ndarray, neighbor_search: bool = True) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if points.size == 0:
            return (
                np.zeros(0, dtype=bool),
                np.zeros((0, 3), dtype=np.float64),
                np.zeros((0, 3, 3), dtype=np.float64),
                np.zeros((0, 3), dtype=np.float64),
            )
        keys = np.floor(points / self.resolution).astype(np.int64)

        valid_mask = np.zeros(len(points), dtype=bool)
        residuals = np.zeros((len(points), 3), dtype=np.float64)
        inv_covs = np.zeros((len(points), 3, 3), dtype=np.float64)

        for i, pt in enumerate(points):
            k = tuple(keys[i])
            voxel = self.voxels.get(k)

            if voxel is None and neighbor_search:
                min_dist = float('inf')
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for dz in (-1, 0, 1):
                            nk = (k[0]+dx, k[1]+dy, k[2]+dz)
                            v = self.voxels.get(nk)
                            if v is not None:
                                dist = np.linalg.norm(pt - v[0])
                                if dist < min_dist:
                                    min_dist = float(dist)
                                    voxel = v

            if voxel is not None:
                mean, inv_cov = voxel
                valid_mask[i] = True
                residuals[i] = pt - mean
                inv_covs[i] = inv_cov

        return valid_mask, residuals[valid_mask], inv_covs[valid_mask], points[valid_mask]


class PcdRelocalizer3D(Node):
    def __init__(self) -> None:
        super().__init__("pcd_relocalizer_3d")
        raw_map_root = self.declare_parameter("map_root", "${A2_WORKSPACE}/runtime/maps").value
        self.map_root = Path(os.path.expandvars(os.path.expanduser(raw_map_root)))
        self.map_id = self.declare_parameter("map_id", "").value
        self.pcd_path = self.declare_parameter("pcd_path", "").value
        self.live_cloud_topic = self.declare_parameter("live_cloud_topic", "/jt128/front/points").value
        self.odom_topic = self.declare_parameter("odom_topic", "/jt128/dlio/odom").value
        self.initial_pose_topic = self.declare_parameter("initial_pose_topic", "/initialpose").value
        self.pose_topic = self.declare_parameter("pose_topic", "/a2/relocalization/pose").value
        self.status_topic = self.declare_parameter("status_topic", "/a2/relocalization/status").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.lidar_xyz = list(
            self.declare_parameter("base_to_lidar_xyz", [0.33767, 0.0, 0.08134]).value
        )
        self.use_lidar_rotation_matrix = bool(
            self.declare_parameter("base_to_lidar_use_rotation_matrix", False).value
        )
        self.lidar_rotation_matrix = list(
            self.declare_parameter(
                "base_to_lidar_rotation_matrix",
                [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0],
            ).value
        )
        self.lidar_rpy = list(self.declare_parameter("base_to_lidar_rpy", [0.0, 0.0, 0.0]).value)
        self.publish_tf = bool(self.declare_parameter("publish_tf", True).value)
        self.auto_seed_identity = bool(self.declare_parameter("auto_seed_identity", False).value)

        self.matcher_backend = str(self.declare_parameter("matcher_backend", "ndt").value)
        self.match_interval_sec = max(0.2, float(self.declare_parameter("match_interval_sec", 2.0).value))
        self.voxel_leaf_size = max(0.0, float(self.declare_parameter("voxel_leaf_size", 0.35).value))
        self.max_map_points = int(self.declare_parameter("max_map_points", 200000).value)
        self.max_scan_points = int(self.declare_parameter("max_scan_points", 1200).value)

        self.ndt_resolution = float(self.declare_parameter("ndt_resolution", 1.0).value)
        self.ndt_min_points_per_voxel = int(self.declare_parameter("ndt_min_points_per_voxel", 6).value)
        self.ndt_covariance_regularization = float(self.declare_parameter("ndt_covariance_regularization", 0.05).value)
        self.ndt_neighbor_search = bool(self.declare_parameter("ndt_neighbor_search", True).value)
        self.ndt_outlier_mahalanobis_threshold = float(self.declare_parameter("ndt_outlier_mahalanobis_threshold", 9.0).value)
        self.ndt_max_iterations = int(self.declare_parameter("ndt_max_iterations", 12).value)
        self.ndt_step_translation_limit = float(self.declare_parameter("ndt_step_translation_limit", 0.25).value)
        self.ndt_step_rotation_limit = math.radians(float(self.declare_parameter("ndt_step_rotation_limit_deg", 3.0).value))
        self.ndt_converged_translation_epsilon = float(self.declare_parameter("ndt_converged_translation_epsilon", 0.01).value)
        self.ndt_converged_rotation_epsilon = math.radians(float(self.declare_parameter("ndt_converged_rotation_epsilon_deg", 0.2).value))
        self.ndt_score_threshold = float(self.declare_parameter("ndt_score_threshold", 3.0).value)
        self.ndt_min_effective_correspondences = int(self.declare_parameter("ndt_min_effective_correspondences", 80).value)

        self.max_translation_correction = float(self.declare_parameter("max_translation_correction", 1.2).value)
        self.max_rotation_correction = math.radians(float(self.declare_parameter("max_rotation_correction_deg", 5.0).value))
        self.max_map_to_odom_translation = float(self.declare_parameter("max_map_to_odom_translation", 20.0).value)
        self.max_base_distance_from_origin = float(self.declare_parameter("max_base_distance_from_origin", 120.0).value)

        self.covariance_mode = str(self.declare_parameter("covariance_mode", "score_scaled").value)
        self.xy_variance = float(self.declare_parameter("xy_variance", 0.04).value)
        self.z_variance = float(self.declare_parameter("z_variance", 0.08).value)
        self.rot_variance = float(self.declare_parameter("rot_variance", 0.04).value)

        if self.use_lidar_rotation_matrix:
            self.base_to_lidar = xyz_rotation_matrix_to_matrix(
                self.lidar_xyz, self.lidar_rotation_matrix
            )
        else:
            self.base_to_lidar = xyz_rpy_to_matrix(self.lidar_xyz, self.lidar_rpy)

        self.map_to_odom = np.eye(4, dtype=np.float64)
        self.has_seed = bool(self.auto_seed_identity)
        self.last_odom: Odometry | None = None
        self.last_scan: np.ndarray | None = None
        self.last_status = ""
        self.last_logged_state = ""
        self.last_log_time = self.get_clock().now()
        self.last_score: float | None = None
        self.last_cloud_parse_time = None
        self.cloud_sub = None
        self.match_in_progress = False

        map_points = self._load_pcd()
        if self.voxel_leaf_size > 0.0 and self.matcher_backend != "ndt":
            map_points = voxel_downsample(map_points, self.voxel_leaf_size, self.max_map_points)
        elif self.max_map_points > 0 and len(map_points) > self.max_map_points:
            step = max(1, len(map_points) // self.max_map_points)
            map_points = map_points[::step][:self.max_map_points]

        if self.matcher_backend == "ndt":
            self.ndt_grid = NdtVoxelGrid(
                map_points,
                self.ndt_resolution,
                self.ndt_min_points_per_voxel,
                self.ndt_covariance_regularization,
            )
            self.get_logger().info(f"Built NDT grid with {len(self.ndt_grid.voxels)} voxels")
        else:
            raise ValueError(f"Unsupported matcher_backend: {self.matcher_backend}")

        self.get_logger().info(
            f"Loaded 3D relocalization map points={len(map_points)} source={self._resolve_pcd_path()}"
        )

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        self.create_subscription(PoseWithCovarianceStamped, self.initial_pose_topic, self.on_initial_pose, 10)
        if self.has_seed:
            self.ensure_cloud_subscription()
        self.create_timer(self.match_interval_sec, self.run_matcher)

    def ensure_cloud_subscription(self) -> None:
        if self.cloud_sub is not None:
            return
        self.cloud_sub = self.create_subscription(PointCloud2, self.live_cloud_topic, self.on_cloud, 2)
        self.get_logger().info(f"Subscribed to live 3D cloud for relocalization: {self.live_cloud_topic}")

    def _resolve_pcd_path(self) -> Path:
        if self.pcd_path:
            return Path(os.path.expandvars(os.path.expanduser(self.pcd_path)))
        map_id = self.map_id
        if not map_id:
            current_file = self.map_root / "current_map.txt"
            if current_file.exists():
                map_id = current_file.read_text(encoding="utf-8").strip()
        if not map_id:
            raise RuntimeError("map_id or pcd_path is required")
        return self.map_root / map_id / "pointcloud_map_3d.pcd"

    def _load_pcd(self) -> np.ndarray:
        path = self._resolve_pcd_path()
        if not path.exists():
            raise RuntimeError(f"PCD not found: {path}")
        points: list[tuple[float, float, float]] = []
        fields: list[str] = []
        data_started = False
        with path.open("r", encoding="ascii", errors="strict") as handle:
            for raw_line in handle:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if data_started:
                    parts = line.split()
                    if len(parts) < 3:
                        continue
                    try:
                        if fields and {"x", "y", "z"}.issubset(set(fields)):
                            points.append(
                                (
                                    float(parts[fields.index("x")]),
                                    float(parts[fields.index("y")]),
                                    float(parts[fields.index("z")]),
                                )
                            )
                        else:
                            points.append((float(parts[0]), float(parts[1]), float(parts[2])))
                    except (ValueError, IndexError):
                        continue
                    continue
                key, _, value = line.partition(" ")
                if key.upper() == "FIELDS":
                    fields = value.split()
                if key.upper() == "DATA":
                    if value.strip().lower() != "ascii":
                        raise RuntimeError("pcd_relocalizer_3d currently supports ASCII PCD only")
                    data_started = True
        if not points:
            raise RuntimeError(f"PCD has no readable XYZ points: {path}")
        return np.array(points, dtype=np.float64)

    def on_cloud(self, msg: PointCloud2) -> None:
        if not self.has_seed:
            return
        now = self.get_clock().now()
        if self.last_cloud_parse_time is not None:
            age = (now - self.last_cloud_parse_time).nanoseconds * 1e-9
            if age < self.match_interval_sec:
                return
        self.last_cloud_parse_time = now
        points = []
        try:
            for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                x, y, z = float(point[0]), float(point[1]), float(point[2])
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                    points.append((x, y, z))
        except Exception as exc:
            self.publish_status(False, "cloud_error", f"read_points_failed:{exc}")
            return
        if not points:
            self.publish_status(False, "waiting_scan", "empty_cloud")
            return
        self.last_scan = voxel_downsample(
            np.array(points, dtype=np.float64),
            self.voxel_leaf_size,
            self.max_scan_points,
        )

    def on_odom(self, msg: Odometry) -> None:
        self.last_odom = msg
        if self.has_seed:
            self.publish_pose_and_tf(msg, ready=self.last_score is not None and self.last_score <= self.ndt_score_threshold)

    def on_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if self.last_odom is None:
            self.publish_status(False, "waiting_odom", "initialpose_without_odom")
            return
        map_to_base = pose_to_matrix(msg.pose.pose.position, msg.pose.pose.orientation)
        odom_to_base = pose_to_matrix(
            self.last_odom.pose.pose.position,
            self.last_odom.pose.pose.orientation,
        )
        self.map_to_odom = map_to_base @ np.linalg.inv(odom_to_base)
        self.has_seed = True
        self.last_score = None
        self.last_scan = None
        self.last_cloud_parse_time = None
        self.ensure_cloud_subscription()
        if self.auto_seed_identity:
            self.get_logger().warning(
                "auto_seed_identity is enabled. This is only safe when the loaded PCD map "
                "was built in the same odom origin as the live DLIO session."
            )
        self.publish_pose_and_tf(self.last_odom, ready=False)
        self.publish_status(True, "seeded", "initialpose_anchor_set")

    def run_matcher(self) -> None:
        if self.match_in_progress:
            return
        self.match_in_progress = True
        try:
            self._run_ndt()
        finally:
            self.match_in_progress = False

    def _run_ndt(self) -> None:
        if not self.has_seed:
            self.publish_status(False, "waiting_seed", "send_initialpose_or_enable_auto_seed")
            return
        if self.last_odom is None:
            self.publish_status(False, "waiting_odom", "no_dlio_odom")
            return
        if self.last_scan is None or len(self.last_scan) < self.ndt_min_effective_correspondences:
            count = 0 if self.last_scan is None else len(self.last_scan)
            self.publish_status(False, "waiting_scan", f"scan_points={count}")
            return

        odom_to_base = pose_to_matrix(
            self.last_odom.pose.pose.position,
            self.last_odom.pose.pose.orientation,
        )
        source = transform_points(self.map_to_odom @ odom_to_base @ self.base_to_lidar, self.last_scan)
        correction = np.eye(4, dtype=np.float64)

        score = float("inf")
        effective_correspondences = 0
        iteration = 0

        for iteration in range(max(1, self.ndt_max_iterations)):
            moved = transform_points(correction, source)
            valid_mask, residuals, inv_covs, valid_points = self.ndt_grid.query(moved, self.ndt_neighbor_search)

            if not np.any(valid_mask):
                self.publish_status(False, "ndt_rejected", "no_voxels_found")
                return

            dist_sq = np.einsum('ij,ijk,ik->i', residuals, inv_covs, residuals)
            inlier_mask = dist_sq < self.ndt_outlier_mahalanobis_threshold

            effective_correspondences = int(np.count_nonzero(inlier_mask))
            if effective_correspondences < self.ndt_min_effective_correspondences:
                self.publish_status(
                    False,
                    "ndt_rejected",
                    f"few_effective_correspondences={effective_correspondences}",
                )
                return

            inlier_residuals = residuals[inlier_mask]
            inlier_inv_covs = inv_covs[inlier_mask]
            inlier_points = valid_points[inlier_mask]

            score_sum = float(np.sum(dist_sq[inlier_mask]))
            N = len(inlier_points)
            px = inlier_points[:, 0]
            py = inlier_points[:, 1]
            pz = inlier_points[:, 2]

            J = np.zeros((N, 3, 6), dtype=np.float64)
            J[:, 0, 0] = 1.0; J[:, 1, 1] = 1.0; J[:, 2, 2] = 1.0
            J[:, 0, 4] = pz;  J[:, 0, 5] = -py
            J[:, 1, 3] = -pz; J[:, 1, 5] = px
            J[:, 2, 3] = py;  J[:, 2, 4] = -px

            J_T_inv_cov = np.einsum('nij,nik->nkj', inlier_inv_covs, J)

            H_batch = np.einsum('nji,nik->njk', J_T_inv_cov, J)
            H = np.sum(H_batch, axis=0)

            b_batch = np.einsum('nji,ni->nj', J_T_inv_cov, inlier_residuals)
            b = np.sum(b_batch, axis=0)

            try:
                delta = -np.linalg.solve(H, b)
            except np.linalg.LinAlgError:
                self.publish_status(False, "ndt_error", "singular_hessian")
                return

            if not np.all(np.isfinite(delta)):
                self.publish_status(False, "ndt_error", "non_finite_update")
                return

            d_t = delta[:3]
            d_r = delta[3:]

            t_norm = np.linalg.norm(d_t)
            if t_norm > self.ndt_step_translation_limit:
                d_t *= self.ndt_step_translation_limit / t_norm
            r_norm = np.linalg.norm(d_r)
            if r_norm > self.ndt_step_rotation_limit:
                d_r *= self.ndt_step_rotation_limit / r_norm

            step = xyz_rpy_to_matrix(d_t, d_r)
            correction = step @ correction
            score = score_sum / effective_correspondences

            if np.linalg.norm(d_t) < self.ndt_converged_translation_epsilon and np.linalg.norm(d_r) < self.ndt_converged_rotation_epsilon:
                break

        translation = float(np.linalg.norm(correction[:3, 3]))
        angle = rotation_angle(correction[:3, :3])
        if translation > self.max_translation_correction or angle > self.max_rotation_correction:
            self.publish_status(
                False,
                "ndt_rejected",
                f"correction_too_large:translation={translation:.3f},rotation_deg={math.degrees(angle):.2f}",
            )
            return

        candidate_map_to_odom = correction @ self.map_to_odom
        candidate_map_to_base = candidate_map_to_odom @ odom_to_base
        if np.linalg.norm(candidate_map_to_odom[:3, 3]) > self.max_map_to_odom_translation:
            self.publish_status(
                False,
                "ndt_rejected",
                (
                    "map_to_odom_out_of_bounds:"
                    f"norm={np.linalg.norm(candidate_map_to_odom[:3, 3]):.3f},"
                    f"limit={self.max_map_to_odom_translation:.3f}"
                ),
            )
            return
        if np.linalg.norm(candidate_map_to_base[:3, 3]) > self.max_base_distance_from_origin:
            self.publish_status(
                False,
                "ndt_rejected",
                (
                    "base_pose_out_of_bounds:"
                    f"norm={np.linalg.norm(candidate_map_to_base[:3, 3]):.3f},"
                    f"limit={self.max_base_distance_from_origin:.3f}"
                ),
            )
            return

        if score > self.ndt_score_threshold:
            self.publish_status(
                False,
                "ndt_rejected",
                "score_above_threshold",
                score=score,
                effective_correspondences=effective_correspondences,
                iterations=iteration + 1,
                translation=translation,
                rotation_deg=math.degrees(angle),
            )
            return

        self.map_to_odom = candidate_map_to_odom
        self.last_score = score
        ready = True

        self.publish_pose_and_tf(self.last_odom, ready=ready)
        self.publish_status(
            ready,
            "ready",
            "converged",
            score=score,
            effective_correspondences=effective_correspondences,
            iterations=iteration + 1,
            translation=translation,
            rotation_deg=math.degrees(angle),
        )

    def publish_pose_and_tf(self, odom_msg: Odometry, *, ready: bool) -> None:
        odom_to_base = pose_to_matrix(odom_msg.pose.pose.position, odom_msg.pose.pose.orientation)
        map_to_base = self.map_to_odom @ odom_to_base
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = odom_msg.header.stamp
        pose.header.frame_id = self.map_frame
        pose.pose.pose.position.x = float(map_to_base[0, 3])
        pose.pose.pose.position.y = float(map_to_base[1, 3])
        pose.pose.pose.position.z = float(map_to_base[2, 3])
        qx, qy, qz, qw = matrix_to_quaternion(map_to_base[:3, :3])
        pose.pose.pose.orientation.x = qx
        pose.pose.pose.orientation.y = qy
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw

        covariance_scale = 1.0
        if not ready:
            covariance_scale = 10.0
        elif self.covariance_mode == "score_scaled" and self.last_score is not None:
            covariance_scale = max(1.0, min(5.0, self.last_score / max(0.1, self.ndt_score_threshold)))

        pose.pose.covariance[0] = self.xy_variance * covariance_scale
        pose.pose.covariance[7] = self.xy_variance * covariance_scale
        pose.pose.covariance[14] = self.z_variance * covariance_scale
        pose.pose.covariance[21] = self.rot_variance * covariance_scale
        pose.pose.covariance[28] = self.rot_variance * covariance_scale
        pose.pose.covariance[35] = self.rot_variance * covariance_scale
        self.pose_pub.publish(pose)

        if not self.publish_tf:
            return
        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom_msg.header.stamp
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.odom_frame
        tf_msg.transform.translation.x = float(self.map_to_odom[0, 3])
        tf_msg.transform.translation.y = float(self.map_to_odom[1, 3])
        tf_msg.transform.translation.z = float(self.map_to_odom[2, 3])
        qx, qy, qz, qw = matrix_to_quaternion(self.map_to_odom[:3, :3])
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

    def publish_status(
        self,
        ready: bool,
        state: str,
        reason: str,
        *,
        score: float | None = None,
        effective_correspondences: int | None = None,
        iterations: int | None = None,
        translation: float | None = None,
        rotation_deg: float | None = None,
    ) -> None:
        score_val = score if score is not None else (self.last_score if self.last_score is not None else -1.0)
        status_parts = [
            f"state={state}",
            f"ready={str(bool(ready)).lower()}",
            f"reason={reason}",
            f"matcher={self.matcher_backend}",
            f"score={score_val:.3f}",
        ]
        if effective_correspondences is not None:
            status_parts.append(f"effective_correspondences={effective_correspondences}")
        if iterations is not None:
            status_parts.append(f"iterations={iterations}")
        if translation is not None:
            status_parts.append(f"translation={translation:.3f}")
        if rotation_deg is not None:
            status_parts.append(f"rotation_deg={rotation_deg:.2f}")

        status_parts.extend([
            f"map_id={self.map_id or 'current'}",
            f"live_cloud_topic={self.live_cloud_topic}",
            f"odom_topic={self.odom_topic}"
        ])

        status = ";".join(status_parts)
        self.status_pub.publish(String(data=status))
        now = self.get_clock().now()
        log_age = (now - self.last_log_time).nanoseconds * 1e-9
        should_log = state != self.last_logged_state or log_age >= 5.0
        if should_log:
            self.get_logger().info(f"3D relocalization status changed: {status}")
            self.last_logged_state = state
            self.last_log_time = now
        self.last_status = status


def main() -> None:
    rclpy.init()
    node = PcdRelocalizer3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()