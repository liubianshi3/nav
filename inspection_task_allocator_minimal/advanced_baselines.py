import math

from astar_planner import AStarPlanner
from deadline_model import (
    compute_deadline,
    compute_task_urgency,
    estimate_reference_time,
)


class _AdvancedBaseAllocator:
    def __init__(self, grid_map, tasks, start_pos, robot_speed=0.6, inspection_time=5.0):
        self.grid_map = grid_map
        self.tasks = tasks
        self.start_pos = start_pos
        self.current_pos = start_pos
        self.robot_speed = robot_speed
        self.inspection_time = inspection_time
        self.planner = AStarPlanner(grid_map)
        self.path_cache = {}
        self.task_sequence = []
        self.selection_records = []
        self.total_path_length = 0.0
        self.total_inspection_time = 0.0
        self.task_finish_times = {}

    def cached_plan(self, start, goal):
        key = (start, goal)
        if key not in self.path_cache:
            self.path_cache[key] = self.planner.plan(start, goal)
        return self.path_cache[key]

    def get_unfinished_tasks(self):
        return [task for task in self.tasks if task.status == 0]

    def compute_high_priority_avg_response_time(self):
        values = [
            self.task_finish_times[task.task_id]
            for task in self.tasks
            if task.priority >= 0.75 and task.task_id in self.task_finish_times
        ]
        return sum(values) / len(values) if values else 0.0

    def compute_priority_weighted_completion_time(self):
        weighted_sum = 0.0
        priority_sum = 0.0
        for task in self.tasks:
            if task.task_id in self.task_finish_times:
                weighted_sum += task.priority * self.task_finish_times[task.task_id]
                priority_sum += task.priority
        return weighted_sum / priority_sum if priority_sum else 0.0

    def compute_high_priority_top5_rate(self):
        if not self.task_sequence:
            return 0.0
        task_lookup = {task.task_id: task for task in self.tasks}
        top5 = self.task_sequence[:5]
        count = sum(1 for task_id in top5 if task_lookup[task_id].priority >= 0.75)
        return count / min(5, len(top5)) * 100.0

    def execute_task(self, task, path_info, record):
        path_length = path_info["path_length"]
        travel_time = path_length / self.robot_speed
        finish_time = self.total_inspection_time + travel_time + self.inspection_time
        self.total_path_length += path_length
        self.total_inspection_time = finish_time
        task.mark_completed()
        self.current_pos = task.position
        self.task_sequence.append(task.task_id)
        self.task_finish_times[task.task_id] = finish_time
        record["finish_time"] = finish_time
        record["travel_time"] = travel_time
        self.selection_records.append(record)

    def build_result(self):
        return {
            "task_sequence": self.task_sequence,
            "completed_task_num": len(self.task_sequence),
            "total_path_length": self.total_path_length,
            "total_inspection_time": self.total_inspection_time,
            "high_priority_avg_response_time": self.compute_high_priority_avg_response_time(),
            "priority_weighted_completion_time": self.compute_priority_weighted_completion_time(),
            "high_priority_top5_rate": self.compute_high_priority_top5_rate(),
            "selection_records": self.selection_records,
        }


class PriorityGreedyAllocator(_AdvancedBaseAllocator):
    def select_next_task(self):
        reachable = []
        for task in self.get_unfinished_tasks():
            path_info = self.cached_plan(self.current_pos, task.position)
            if path_info.get("reachable", False):
                reachable.append((task, path_info))
        if not reachable:
            return None, None, None

        lengths = [p["path_length"] for _, p in reachable]
        l_min = min(lengths)
        l_max = max(lengths)
        denom = l_max - l_min + 1e-6
        candidates = []
        for task, path_info in reachable:
            distance_cost = (path_info["path_length"] - l_min) / denom
            score = (
                0.45 * task.priority
                + 0.25 * task.risk
                + 0.15 * task.abnormal_weight
                - 0.15 * distance_cost
            )
            candidates.append((score, task.task_id, task, path_info, distance_cost))
        candidates.sort(key=lambda item: (-item[0], item[1]))
        score, _, task, path_info, distance_cost = candidates[0]
        return task, path_info, {
            "task_id": task.task_id,
            "score": score,
            "distance_cost": distance_cost,
            "path_length": path_info["path_length"],
            "priority": task.priority,
            "risk": task.risk,
            "abnormal_weight": task.abnormal_weight,
            "method_detail": "priority_greedy",
        }

    def run(self):
        while self.get_unfinished_tasks():
            task, path_info, record = self.select_next_task()
            if task is None:
                print("No reachable unfinished tasks. Stop Priority-Greedy.")
                break
            self.execute_task(task, path_info, record)
        return self.build_result()


class DeadlineGreedyAllocator(_AdvancedBaseAllocator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.reference_time = estimate_reference_time(
            self.tasks,
            self.planner,
            self.start_pos,
            self.robot_speed,
            self.inspection_time,
        )

    def select_next_task(self):
        reachable = []
        for task in self.get_unfinished_tasks():
            path_info = self.cached_plan(self.current_pos, task.position)
            if path_info.get("reachable", False):
                reachable.append((task, path_info))
        if not reachable:
            return None, None, None

        lengths = [p["path_length"] for _, p in reachable]
        l_min = min(lengths)
        l_max = max(lengths)
        denom = l_max - l_min + 1e-6
        candidates = []
        for task, path_info in reachable:
            arrival_time = path_info["path_length"] / self.robot_speed
            normalized_arrival = (path_info["path_length"] - l_min) / denom
            deadline = compute_deadline(task, self.reference_time)
            violation = max(0.0, arrival_time + self.inspection_time - deadline)
            normalized_violation = violation / (self.reference_time + 1e-6)
            urgency = compute_task_urgency(task)
            score = urgency - 0.25 * normalized_arrival - 0.25 * normalized_violation
            candidates.append((score, task.task_id, task, path_info, urgency, deadline, violation))
        candidates.sort(key=lambda item: (-item[0], item[1]))
        score, _, task, path_info, urgency, deadline, violation = candidates[0]
        return task, path_info, {
            "task_id": task.task_id,
            "score": score,
            "urgency": urgency,
            "deadline": deadline,
            "deadline_violation": violation,
            "path_length": path_info["path_length"],
            "priority": task.priority,
            "risk": task.risk,
            "abnormal_weight": task.abnormal_weight,
            "method_detail": "deadline_greedy",
        }

    def run(self):
        while self.get_unfinished_tasks():
            task, path_info, record = self.select_next_task()
            if task is None:
                print("No reachable unfinished tasks. Stop Deadline-Greedy.")
                break
            self.execute_task(task, path_info, record)
        return self.build_result()


class TSP2OptAllocator(_AdvancedBaseAllocator):
    def _distance(self, a, b):
        info = self.cached_plan(a, b)
        if not info.get("reachable", False):
            return 1e6
        return info["path_length"]

    def _sequence_cost(self, sequence):
        if not sequence:
            return 0.0
        cost = self._distance(self.start_pos, sequence[0].position)
        for prev, nxt in zip(sequence, sequence[1:]):
            cost += self._distance(prev.position, nxt.position)
        return cost

    def build_initial_sequence(self):
        remaining = list(self.tasks)
        sequence = []
        pos = self.start_pos
        while remaining:
            remaining.sort(key=lambda task: (self._distance(pos, task.position), task.task_id))
            task = remaining.pop(0)
            sequence.append(task)
            pos = task.position
        return sequence

    def two_opt(self, sequence, max_passes=40):
        best = list(sequence)
        best_cost = self._sequence_cost(best)
        improved = True
        passes = 0
        while improved and passes < max_passes:
            improved = False
            passes += 1
            for i in range(0, len(best) - 2):
                for j in range(i + 2, len(best)):
                    candidate = best[:i] + list(reversed(best[i:j])) + best[j:]
                    cost = self._sequence_cost(candidate)
                    if cost + 1e-6 < best_cost:
                        best = candidate
                        best_cost = cost
                        improved = True
                        break
                if improved:
                    break
        return best

    def run(self):
        sequence = self.two_opt(self.build_initial_sequence())
        for task in sequence:
            if task.status == 1:
                continue
            path_info = self.cached_plan(self.current_pos, task.position)
            if not path_info.get("reachable", False):
                self.selection_records.append(
                    {
                        "task_id": task.task_id,
                        "reachable": False,
                        "method_detail": "tsp_2opt_unreachable",
                    }
                )
                continue
            record = {
                "task_id": task.task_id,
                "path_length": path_info["path_length"],
                "priority": task.priority,
                "risk": task.risk,
                "abnormal_weight": task.abnormal_weight,
                "method_detail": "tsp_2opt_fixed_sequence",
            }
            self.execute_task(task, path_info, record)
        return self.build_result()
