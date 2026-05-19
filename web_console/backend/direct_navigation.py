from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class DirectVelocityCommand:
    linear_x: float
    angular_z: float
    distance_remaining: float
    heading_error_rad: float
    yaw_error_rad: float
    reached: bool


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, limit: float) -> float:
    limit = abs(float(limit))
    return max(-limit, min(limit, float(value)))


def compute_direct_velocity_command(
    *,
    current_x: float,
    current_y: float,
    current_yaw: float,
    goal_x: float,
    goal_y: float,
    goal_yaw: float,
    max_linear_x: float,
    max_angular_z: float,
    slow_radius_m: float,
    heading_deadband_rad: float,
    goal_tolerance_m: float,
    yaw_tolerance_rad: float,
) -> DirectVelocityCommand:
    dx = float(goal_x) - float(current_x)
    dy = float(goal_y) - float(current_y)
    distance = math.hypot(dx, dy)
    yaw = float(current_yaw)
    yaw_error = normalize_angle(float(goal_yaw) - yaw)

    if distance <= float(goal_tolerance_m):
        if abs(yaw_error) <= float(yaw_tolerance_rad):
            return DirectVelocityCommand(
                linear_x=0.0,
                angular_z=0.0,
                distance_remaining=distance,
                heading_error_rad=0.0,
                yaw_error_rad=yaw_error,
                reached=True,
            )
        angular_z = _clamp(1.2 * yaw_error, max_angular_z)
        return DirectVelocityCommand(
            linear_x=0.0,
            angular_z=angular_z,
            distance_remaining=distance,
            heading_error_rad=0.0,
            yaw_error_rad=yaw_error,
            reached=False,
        )

    heading = math.atan2(dy, dx)
    heading_error = normalize_angle(heading - yaw)
    angular_z = _clamp(1.4 * heading_error, max_angular_z)
    if abs(heading_error) > float(heading_deadband_rad):
        linear_x = 0.0
    else:
        slow_radius = max(float(slow_radius_m), float(goal_tolerance_m), 0.01)
        linear_scale = max(0.25, min(1.0, distance / slow_radius))
        linear_x = max(0.05, abs(float(max_linear_x)) * linear_scale)

    return DirectVelocityCommand(
        linear_x=min(abs(float(max_linear_x)), linear_x),
        angular_z=angular_z,
        distance_remaining=distance,
        heading_error_rad=heading_error,
        yaw_error_rad=yaw_error,
        reached=False,
    )
