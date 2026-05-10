import copy
from dataclasses import dataclass

from .astar_planner import AStarPlanner


@dataclass
class BaselineResult:
    task_sequence: list
    total_path_length: float
    total_inspection_time: float
    high_priority_avg_response_time: float
    completed_task_num: int
    selection_records: list


class _BaseBaselineAllocator:
    def __init__(self, grid_map, start, tasks, robot_speed=0.6, inspection_time=5.0):
        self.grid_map = grid_map
        self.start = start
        self.current_pos = start
        self.tasks = tasks
        self.robot_speed = robot_speed
        self.inspection_time = inspection_time
        self.planner = AStarPlanner(grid_map)
        self.epsilon = 1e-6

    def _make_result(self, sequence, total_path_length, total_inspection_time, high_priority_response_times, selection_records):
        avg_response = (
            sum(high_priority_response_times) / len(high_priority_response_times)
            if high_priority_response_times
            else 0.0
        )
        return BaselineResult(
            task_sequence=sequence,
            total_path_length=total_path_length,
            total_inspection_time=total_inspection_time,
            high_priority_avg_response_time=avg_response,
            completed_task_num=len(sequence),
            selection_records=selection_records,
        )


class FixedSequenceAllocator(_BaseBaselineAllocator):
    def allocate(self):
        remaining = [t for t in self.tasks if t.status == 0]
        sequence = []
        selection_records = []
        total_path_length = 0.0
        total_inspection_time = 0.0
        high_priority_response_times = []
        current_pos = self.start

        for order, task in enumerate(remaining, start=1):
            path, path_length, turn_count, obstacle_nearby_count = self.planner.plan(current_pos, (task.x, task.y))
            if not path:
                continue
            travel_time = path_length / self.robot_speed if self.robot_speed > 0 else 0.0
            total_path_length += path_length
            total_inspection_time += travel_time + self.inspection_time
            if task.priority >= 0.75:
                high_priority_response_times.append(total_inspection_time)
            task.status = 1
            current_pos = (task.x, task.y)
            sequence.append(task.task_id)
            selection_records.append(
                {
                    "order": order,
                    "task_id": task.task_id,
                    "path_length": path_length,
                    "turn_count": turn_count,
                    "obstacle_nearby_count": obstacle_nearby_count,
                    "selection_metric": order,
                }
            )

        return self._make_result(sequence, total_path_length, total_inspection_time, high_priority_response_times, selection_records)


class NearestNeighborAllocator(_BaseBaselineAllocator):
    def allocate(self):
        remaining = [t for t in self.tasks if t.status == 0]
        sequence = []
        selection_records = []
        total_path_length = 0.0
        total_inspection_time = 0.0
        high_priority_response_times = []
        current_pos = self.start
        order = 0

        while remaining:
            candidates = []
            for task in remaining:
                path, path_length, turn_count, obstacle_nearby_count = self.planner.plan(current_pos, (task.x, task.y))
                if not path:
                    continue
                candidates.append((path_length, task, turn_count, obstacle_nearby_count))

            if not candidates:
                break

            candidates.sort(key=lambda item: item[0])
            path_length, task, turn_count, obstacle_nearby_count = candidates[0]
            order += 1
            travel_time = path_length / self.robot_speed if self.robot_speed > 0 else 0.0
            total_path_length += path_length
            total_inspection_time += travel_time + self.inspection_time
            if task.priority >= 0.75:
                high_priority_response_times.append(total_inspection_time)
            task.status = 1
            current_pos = (task.x, task.y)
            sequence.append(task.task_id)
            selection_records.append(
                {
                    "order": order,
                    "task_id": task.task_id,
                    "path_length": path_length,
                    "turn_count": turn_count,
                    "obstacle_nearby_count": obstacle_nearby_count,
                    "selection_metric": path_length,
                }
            )
            remaining = [t for t in remaining if t.status == 0]

        return self._make_result(sequence, total_path_length, total_inspection_time, high_priority_response_times, selection_records)


class AStarOnlyAllocator(_BaseBaselineAllocator):
    def allocate(self):
        remaining = [t for t in self.tasks if t.status == 0]
        sequence = []
        selection_records = []
        total_path_length = 0.0
        total_inspection_time = 0.0
        high_priority_response_times = []
        current_pos = self.start
        order = 0

        while remaining:
            candidates = []
            for task in remaining:
                path, path_length, turn_count, obstacle_nearby_count = self.planner.plan(current_pos, (task.x, task.y))
                if not path:
                    continue
                cost = path_length + 0.5 * turn_count + 0.2 * obstacle_nearby_count
                candidates.append((cost, path_length, task, turn_count, obstacle_nearby_count))

            if not candidates:
                break

            candidates.sort(key=lambda item: item[0])
            cost, path_length, task, turn_count, obstacle_nearby_count = candidates[0]
            order += 1
            travel_time = path_length / self.robot_speed if self.robot_speed > 0 else 0.0
            total_path_length += path_length
            total_inspection_time += travel_time + self.inspection_time
            if task.priority >= 0.75:
                high_priority_response_times.append(total_inspection_time)
            task.status = 1
            current_pos = (task.x, task.y)
            sequence.append(task.task_id)
            selection_records.append(
                {
                    "order": order,
                    "task_id": task.task_id,
                    "path_length": path_length,
                    "turn_count": turn_count,
                    "obstacle_nearby_count": obstacle_nearby_count,
                    "selection_metric": cost,
                }
            )
            remaining = [t for t in remaining if t.status == 0]

        return self._make_result(sequence, total_path_length, total_inspection_time, high_priority_response_times, selection_records)
