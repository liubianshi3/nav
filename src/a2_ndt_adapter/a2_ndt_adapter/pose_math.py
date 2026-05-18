import math

import numpy as np


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


def choose_ndt_initial_stamp(candidate_stamp, latest_cloud_stamp, align_to_cloud: bool):
    if align_to_cloud and latest_cloud_stamp is not None:
        return latest_cloud_stamp
    return candidate_stamp


def choose_periodic_initial_guess_stamp(candidate_stamp, latest_cloud_stamp, align_to_cloud: bool):
    return choose_ndt_initial_stamp(candidate_stamp, latest_cloud_stamp, align_to_cloud)


def should_publish_periodic_guess(elapsed_sec: float | None, period_sec: float, force: bool = False) -> bool:
    if force or elapsed_sec is None:
        return True
    if not math.isfinite(period_sec) or period_sec <= 0.0:
        return True
    return elapsed_sec >= period_sec


def should_feed_ndt_pose_buffer(
    *,
    has_seed: bool,
    odom_available: bool,
    awaiting_first_ndt_fix: bool,
) -> bool:
    del awaiting_first_ndt_fix
    return bool(has_seed and odom_available)


def compose_map_pose_from_odom(map_to_odom: np.ndarray, odom_to_base: np.ndarray) -> np.ndarray:
    return map_to_odom @ odom_to_base


def seeded_odom_tracking_status(
    *,
    has_seed: bool,
    odom_fresh: bool,
    score: float | None,
    score_threshold: float,
    score_min_is_good: bool,
    map_ready: bool,
) -> tuple[bool, str, str]:
    if not has_seed:
        return False, "waiting_seed", "send_initialpose"
    if not odom_fresh:
        return False, "waiting_odom", "odom_stale"
    if not map_ready:
        return False, "waiting_map", "map_not_ready"
    if not score_is_acceptable(score, score_threshold, score_min_is_good):
        return False, "waiting_first_score", "ndt_not_scored_yet"
    return True, "tracking", "odom_tracking"
