import math

from vehicle_model import DifferentialDriveVehicle


class PurePursuitLikeFollower:
    def __init__(
        self,
        vehicle,
        lookahead_distance=0.8,
        k_angle=2.0,
        slow_down_angle=1.0,
        waypoint_tolerance=0.35,
        goal_tolerance=0.35,
    ):
        self.vehicle = vehicle
        self.lookahead_distance = float(lookahead_distance)
        self.k_angle = float(k_angle)
        self.slow_down_angle = float(slow_down_angle)
        self.waypoint_tolerance = float(waypoint_tolerance)
        self.goal_tolerance = float(goal_tolerance)

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def path_simplify(self, path):
        if len(path) <= 2:
            return list(path)
        simplified = [path[0]]
        prev_dir = (path[1][0] - path[0][0], path[1][1] - path[0][1])
        for i in range(2, len(path)):
            cur_dir = (path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            if cur_dir != prev_dir:
                simplified.append(path[i - 1])
            prev_dir = cur_dir
        simplified.append(path[-1])
        # remove duplicates while preserving order
        dedup = []
        for p in simplified:
            if not dedup or dedup[-1] != p:
                dedup.append(p)
        return dedup

    def _closest_path_index(self, path):
        vx, vy = self.vehicle.position()
        best_i = 0
        best_d = float("inf")
        for i, p in enumerate(path):
            d = math.hypot(vx - p[0], vy - p[1])
            if d < best_d:
                best_d = d
                best_i = i
        return best_i, best_d

    def follow_path(self, path, max_steps=5000):
        if not path:
            return {
                "success": False,
                "trajectory": [],
                "trajectory_length": 0.0,
                "execution_time": 0.0,
                "heading_change_sum": 0.0,
                "step_count": 0,
                "final_position": self.vehicle.position(),
                "error_message": "empty_path",
                "raw_path_point_count": 0,
                "simplified_path_point_count": 0,
                "simplified_path_length": 0,
                "trajectory_to_plan_ratio": 0.0,
                "warning": "empty_path",
            }

        raw_path = list(path)
        simplified_path = self.path_simplify(raw_path)
        if len(simplified_path) < 2:
            simplified_path = raw_path

        target_index, closest_d = self._closest_path_index(simplified_path)
        target_index = min(max(target_index, 0), len(simplified_path) - 1)

        trajectory = [self.vehicle.position()]
        trajectory_length = 0.0
        heading_change_sum = 0.0
        prev_theta = self.vehicle.theta
        step_count = 0
        success = False
        error_message = ""
        warning = ""
        goal = simplified_path[-1]
        distance_history = []
        stuck_counter = 0
        prev_goal_dist = self.vehicle.distance_to(goal)

        while step_count < max_steps:
            step_count += 1
            dist_to_goal = self.vehicle.distance_to(goal)
            distance_history.append(dist_to_goal)
            if len(distance_history) > 50:
                distance_history.pop(0)

            if dist_to_goal <= self.goal_tolerance:
                success = True
                break

            while target_index < len(simplified_path) - 1 and self.vehicle.distance_to(simplified_path[target_index]) <= self.waypoint_tolerance:
                target_index += 1

            # allow skipping ahead if future waypoint is closer
            best_future_index = target_index
            best_future_dist = float("inf")
            for i in range(target_index, len(simplified_path)):
                d = self.vehicle.distance_to(simplified_path[i])
                if d < best_future_dist:
                    best_future_dist = d
                    best_future_index = i
            if best_future_dist + 1e-9 < self.vehicle.distance_to(simplified_path[target_index]):
                target_index = best_future_index

            current_target = simplified_path[min(target_index, len(simplified_path) - 1)]
            desired_heading = self.vehicle.heading_to(current_target)
            angle_error = self._normalize_angle(desired_heading - self.vehicle.theta)
            omega = self.k_angle * angle_error
            v = self.vehicle.v_max * max(0.1, math.cos(angle_error))
            if abs(angle_error) > self.slow_down_angle:
                v *= 0.35

            prev_x, prev_y = self.vehicle.position()
            self.vehicle.step(v, omega)
            new_x, new_y = self.vehicle.position()
            trajectory.append((new_x, new_y))
            trajectory_length += math.hypot(new_x - prev_x, new_y - prev_y)
            heading_change_sum += abs(self._normalize_angle(self.vehicle.theta - prev_theta))
            prev_theta = self.vehicle.theta

            new_goal_dist = self.vehicle.distance_to(goal)
            if new_goal_dist < prev_goal_dist - 1e-4:
                stuck_counter = 0
            else:
                stuck_counter += 1
            prev_goal_dist = new_goal_dist

            if stuck_counter > 200:
                warning = "goal_distance_not_decreasing"
                error_message = "stuck_near_goal"
                break

            if trajectory_length > 2.0 * max(1e-9, len(raw_path) - 1):
                warning = warning or "trajectory_to_plan_ratio_high"

            if self.vehicle.distance_to(goal) <= self.goal_tolerance:
                success = True
                break

        if not success and not error_message and step_count >= max_steps:
            error_message = "max_steps_exceeded"

        planned_path_length = max(0.0, float(len(raw_path) - 1))
        trajectory_to_plan_ratio = trajectory_length / planned_path_length if planned_path_length > 0 else 0.0

        return {
            "success": success,
            "trajectory": trajectory,
            "trajectory_length": trajectory_length,
            "execution_time": step_count * self.vehicle.dt,
            "heading_change_sum": heading_change_sum,
            "step_count": step_count,
            "final_position": self.vehicle.position(),
            "error_message": error_message,
            "raw_path_point_count": len(raw_path),
            "simplified_path_point_count": len(simplified_path),
            "simplified_path_length": max(0.0, float(len(simplified_path) - 1)),
            "trajectory_to_plan_ratio": trajectory_to_plan_ratio,
            "warning": warning,
        }
