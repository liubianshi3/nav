from dataclasses import dataclass

from .astar_planner import AStarPlanner


@dataclass
class AllocationResult:
    sequence: list
    total_path_length: float
    total_inspection_time: float
    high_priority_avg_response_time: float
    task_details: list


class PriorityCostTaskAllocator:
    def __init__(
        self,
        grid_map,
        start,
        tasks,
        robot_speed=0.6,
        inspection_time=5.0,
        alpha=0.25,
        beta=0.20,
        lambda_abnormal=0.20,
        gamma=0.20,
        delta=0.10,
        eta=0.05,
        mu=0.5,
    ):
        self.grid_map = grid_map
        self.start = start
        self.current_pos = start
        self.tasks = tasks
        self.robot_speed = robot_speed
        self.inspection_time = inspection_time
        self.alpha = alpha
        self.beta = beta
        self.lambda_abnormal = lambda_abnormal
        self.gamma = gamma
        self.delta = delta
        self.eta = eta
        self.mu = mu
        self.epsilon = 1e-6
        self.planner = AStarPlanner(grid_map)

    def evaluate_candidates(self, current_pos, tasks):
        candidates = []
        for task in tasks:
            path, path_length, turn_count, obstacle_nearby_count = self.planner.plan(
                current_pos, (task.x, task.y)
            )
            if not path:
                continue
            candidates.append(
                {
                    "task": task,
                    "path": path,
                    "path_length": path_length,
                    "turn_count": turn_count,
                    "obstacle_nearby_count": obstacle_nearby_count,
                }
            )

        if not candidates:
            return []

        lengths = [c["path_length"] for c in candidates]
        min_len = min(lengths)
        max_len = max(lengths)
        span = max(max_len - min_len, self.epsilon)

        for candidate in candidates:
            task = candidate["task"]
            distance_cost = (candidate["path_length"] - min_len) / span
            complexity_cost = (
                candidate["obstacle_nearby_count"] + self.mu * candidate["turn_count"]
            ) / (candidate["path_length"] + self.epsilon)
            energy_cost = 0.6 * distance_cost + 0.3 * complexity_cost + 0.1 * 0.0
            score = (
                self.alpha * task.priority
                + self.beta * task.risk
                + self.lambda_abnormal * task.abnormal_weight
                - self.gamma * distance_cost
                - self.delta * complexity_cost
                - self.eta * energy_cost
            )
            candidate.update(
                {
                    "score": score,
                    "distance_cost": distance_cost,
                    "d_norm": distance_cost,
                    "complexity_cost": complexity_cost,
                    "complexity": complexity_cost,
                    "energy_cost": energy_cost,
                    "energy": energy_cost,
                    "abnormal_weight": task.abnormal_weight,
                }
            )

        candidates.sort(key=lambda item: item["score"], reverse=True)
        return candidates

    def select_next_task(self, current_pos, tasks):
        candidates = self.evaluate_candidates(current_pos, tasks)
        return candidates[0] if candidates else None

    def allocate(self):
        remaining = [t for t in self.tasks if t.status == 0]
        sequence = []
        task_details = []
        total_path_length = 0.0
        total_inspection_time = 0.0
        high_priority_response_times = []

        order = 0
        while remaining:
            best = self.select_next_task(self.current_pos, remaining)
            if best is None:
                break

            task = best["task"]
            path_length = best["path_length"]

            sequence.append(task.task_id)
            total_path_length += path_length
            travel_time = path_length / self.robot_speed if self.robot_speed > 0 else 0.0
            total_inspection_time += travel_time + self.inspection_time
            if task.priority >= 0.75:
                high_priority_response_times.append(total_inspection_time)

            task.status = 1
            order += 1
            task_details.append(
                {
                    "order": order,
                    "task_id": task.task_id,
                    "score": best["score"],
                    "path_length": path_length,
                    "distance_cost": best["distance_cost"],
                    "d_norm": best["distance_cost"],
                    "complexity_cost": best["complexity_cost"],
                    "complexity": best["complexity_cost"],
                    "energy_cost": best["energy_cost"],
                    "energy": best["energy_cost"],
                    "abnormal_weight": task.abnormal_weight,
                }
            )
            self.current_pos = (task.x, task.y)
            remaining = [t for t in remaining if t.status == 0]

        high_priority_avg_response_time = (
            sum(high_priority_response_times) / len(high_priority_response_times)
            if high_priority_response_times
            else 0.0
        )
        return AllocationResult(
            sequence=sequence,
            total_path_length=total_path_length,
            total_inspection_time=total_inspection_time,
            high_priority_avg_response_time=high_priority_avg_response_time,
            task_details=task_details,
        )
