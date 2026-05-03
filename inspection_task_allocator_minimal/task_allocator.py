import math
from typing import Any, Dict, List, Tuple

from task_model import InspectionTask
from astar_planner import AStarPlanner


class PriorityCostTaskAllocator:
    def __init__(
        self,
        grid_map,
        tasks,
        start_pos,
        robot_speed=0.6,
        inspection_time=5.0,
        alpha=0.25,
        beta=0.20,
        lambda_abnormal=0.20,
        gamma=0.20,
        delta=0.10,
        eta=0.05,
        mu=0.5,
        epsilon=1e-6,
    ):
        self.grid_map = grid_map
        self.tasks = tasks
        self.start_pos = start_pos
        self.current_pos = start_pos
        self.robot_speed = robot_speed
        self.inspection_time = inspection_time
        self.alpha = alpha
        self.beta = beta
        self.lambda_abnormal = lambda_abnormal
        self.gamma = gamma
        self.delta = delta
        self.eta = eta
        self.mu = mu
        self.epsilon = epsilon
        self.planner = AStarPlanner(grid_map)
        self.task_sequence = []
        self.selection_records = []
        self.total_path_length = 0.0
        self.total_inspection_time = 0.0
        self.task_finish_times = {}

    def get_unfinished_tasks(self):
        return [task for task in self.tasks if task.status == 0]

    def compute_complexity_cost(self, path_info):
        if not path_info.get("reachable", False):
            return math.inf
        path_length = path_info.get("path_length", 0.0)
        turn_count = path_info.get("turn_count", 0)
        obstacle_nearby_count = path_info.get("obstacle_nearby_count", 0)
        return (obstacle_nearby_count + self.mu * turn_count) / (path_length + self.epsilon)

    def compute_energy_cost(self, distance_cost, complexity_cost):
        terrain_factor = 0.0
        return 0.6 * distance_cost + 0.3 * complexity_cost + 0.1 * terrain_factor

    def evaluate_candidates(self, current_pos):
        unfinished_tasks = self.get_unfinished_tasks()
        candidate_records = []
        reachable_records = []

        for task in unfinished_tasks:
            path_info = self.planner.plan(current_pos, task.position)
            if not path_info.get("reachable", False):
                continue
            reachable_records.append((task, path_info))

        if not reachable_records:
            return []

        path_lengths = [record[1]["path_length"] for record in reachable_records]
        l_min = min(path_lengths)
        l_max = max(path_lengths)
        denominator = l_max - l_min + self.epsilon

        for task, path_info in reachable_records:
            l_i = path_info["path_length"]
            distance_cost = (l_i - l_min) / denominator
            complexity_cost = self.compute_complexity_cost(path_info)
            energy_cost = self.compute_energy_cost(distance_cost, complexity_cost)
            score = (
                self.alpha * task.priority
                + self.beta * task.risk
                + self.lambda_abnormal * task.abnormal_weight
                - self.gamma * distance_cost
                - self.delta * complexity_cost
                - self.eta * energy_cost
            )
            candidate_records.append(
                {
                    "task": task,
                    "path_info": path_info,
                    "path_length": l_i,
                    "distance_cost": distance_cost,
                    "complexity_cost": complexity_cost,
                    "energy_cost": energy_cost,
                    "score": score,
                }
            )

        return candidate_records

    def select_next_task(self, current_pos):
        candidate_records = self.evaluate_candidates(current_pos)
        if not candidate_records:
            return None, None, None

        selected_record = max(candidate_records, key=lambda r: r["score"])
        selected_task = selected_record["task"]
        selected_path_info = selected_record["path_info"]
        selection_record = {
            "task_id": selected_task.task_id,
            "score": selected_record["score"],
            "path_length": selected_record["path_length"],
            "distance_cost": selected_record["distance_cost"],
            "complexity_cost": selected_record["complexity_cost"],
            "energy_cost": selected_record["energy_cost"],
            "priority": selected_task.priority,
            "risk": selected_task.risk,
            "abnormal_weight": selected_task.abnormal_weight,
            "turn_count": selected_path_info["turn_count"],
            "obstacle_nearby_count": selected_path_info["obstacle_nearby_count"],
        }
        return selected_task, selected_path_info, selection_record

    def compute_high_priority_avg_response_time(self):
        high_priority_tasks = [task for task in self.tasks if task.priority >= 0.75 and task.task_id in self.task_finish_times]
        if not high_priority_tasks:
            return 0.0
        finish_times = [self.task_finish_times[task.task_id] for task in high_priority_tasks]
        return sum(finish_times) / len(finish_times)

    def run(self):
        while True:
            unfinished_tasks = self.get_unfinished_tasks()
            if not unfinished_tasks:
                break

            selected_task, path_info, record = self.select_next_task(self.current_pos)

            if selected_task is None:
                print("No reachable unfinished tasks. Stop allocation.")
                break

            path_length = path_info["path_length"]
            travel_time = path_length / self.robot_speed
            finish_time = self.total_inspection_time + travel_time + self.inspection_time

            self.total_path_length += path_length
            self.total_inspection_time = finish_time

            selected_task.mark_completed()
            self.current_pos = selected_task.position

            self.task_sequence.append(selected_task.task_id)
            self.task_finish_times[selected_task.task_id] = finish_time

            record["finish_time"] = finish_time
            record["travel_time"] = travel_time
            self.selection_records.append(record)

        high_priority_avg_response_time = self.compute_high_priority_avg_response_time()
        return {
            "task_sequence": self.task_sequence,
            "total_path_length": self.total_path_length,
            "total_inspection_time": self.total_inspection_time,
            "high_priority_avg_response_time": high_priority_avg_response_time,
            "completed_task_num": len(self.task_sequence),
            "selection_records": self.selection_records,
        }
