import math

from astar_planner import AStarPlanner


class _BaseAllocator:
    def __init__(self, grid_map, tasks, start_pos, robot_speed=0.6, inspection_time=5.0):
        self.grid_map = grid_map
        self.tasks = tasks
        self.start_pos = start_pos
        self.current_pos = start_pos
        self.robot_speed = robot_speed
        self.inspection_time = inspection_time
        self.planner = AStarPlanner(grid_map)
        self.task_sequence = []
        self.selection_records = []
        self.total_path_length = 0.0
        self.total_inspection_time = 0.0
        self.task_finish_times = {}

    def get_unfinished_tasks(self):
        return [task for task in self.tasks if task.status == 0]

    def compute_high_priority_avg_response_time(self):
        finish_times = [
            self.task_finish_times[task.task_id]
            for task in self.tasks
            if task.priority >= 0.75 and task.task_id in self.task_finish_times
        ]
        if not finish_times:
            return 0.0
        return sum(finish_times) / len(finish_times)

    def build_result(self):
        return {
            "task_sequence": self.task_sequence,
            "total_path_length": self.total_path_length,
            "total_inspection_time": self.total_inspection_time,
            "high_priority_avg_response_time": self.compute_high_priority_avg_response_time(),
            "completed_task_num": len(self.task_sequence),
            "selection_records": self.selection_records,
        }


class FixedSequenceAllocator(_BaseAllocator):
    def run(self):
        for task in self.tasks:
            if task.status == 1:
                continue

            path_info = self.planner.plan(self.current_pos, task.position)
            if not path_info["reachable"]:
                self.selection_records.append(
                    {
                        "task_id": task.task_id,
                        "reachable": False,
                        "method_detail": "fixed_sequence_unreachable",
                    }
                )
                continue

            path_length = path_info["path_length"]
            travel_time = path_length / self.robot_speed
            finish_time = self.total_inspection_time + travel_time + self.inspection_time

            self.total_path_length += path_length
            self.total_inspection_time = finish_time
            task.mark_completed()
            self.current_pos = task.position
            self.task_sequence.append(task.task_id)
            self.task_finish_times[task.task_id] = finish_time
            self.selection_records.append(
                {
                    "task_id": task.task_id,
                    "path_length": path_length,
                    "priority": task.priority,
                    "risk": task.risk,
                    "abnormal_weight": task.abnormal_weight,
                    "finish_time": finish_time,
                    "travel_time": travel_time,
                    "reachable": True,
                    "method_detail": "fixed_sequence",
                    "turn_count": path_info["turn_count"],
                    "obstacle_nearby_count": path_info["obstacle_nearby_count"],
                }
            )
        return self.build_result()


class NearestNeighborAllocator(_BaseAllocator):
    def run(self):
        while True:
            unfinished_tasks = self.get_unfinished_tasks()
            if not unfinished_tasks:
                break

            candidates = []
            for task in unfinished_tasks:
                path_info = self.planner.plan(self.current_pos, task.position)
                if path_info["reachable"]:
                    candidates.append((path_info["path_length"], task.task_id, task, path_info))

            if not candidates:
                print("No reachable unfinished tasks. Stop NNF.")
                break

            candidates.sort(key=lambda item: (item[0], item[1]))
            _, _, selected_task, path_info = candidates[0]

            path_length = path_info["path_length"]
            travel_time = path_length / self.robot_speed
            finish_time = self.total_inspection_time + travel_time + self.inspection_time

            self.total_path_length += path_length
            self.total_inspection_time = finish_time
            selected_task.mark_completed()
            self.current_pos = selected_task.position
            self.task_sequence.append(selected_task.task_id)
            self.task_finish_times[selected_task.task_id] = finish_time
            self.selection_records.append(
                {
                    "task_id": selected_task.task_id,
                    "path_length": path_length,
                    "priority": selected_task.priority,
                    "risk": selected_task.risk,
                    "abnormal_weight": selected_task.abnormal_weight,
                    "finish_time": finish_time,
                    "travel_time": travel_time,
                    "turn_count": path_info["turn_count"],
                    "obstacle_nearby_count": path_info["obstacle_nearby_count"],
                    "reachable": True,
                    "method_detail": "nearest_neighbor_astar_length",
                }
            )
        return self.build_result()


class AStarOnlyAllocator(_BaseAllocator):
    def run(self):
        while True:
            unfinished_tasks = self.get_unfinished_tasks()
            if not unfinished_tasks:
                break

            candidates = []
            for task in unfinished_tasks:
                path_info = self.planner.plan(self.current_pos, task.position)
                if not path_info["reachable"]:
                    continue
                cost = path_info["path_length"] + 0.5 * path_info["turn_count"] + 0.2 * path_info["obstacle_nearby_count"]
                candidates.append((cost, task.task_id, task, path_info))

            if not candidates:
                print("No reachable unfinished tasks. Stop AStarOnly.")
                break

            candidates.sort(key=lambda item: (item[0], item[1]))
            cost, _, selected_task, path_info = candidates[0]

            path_length = path_info["path_length"]
            travel_time = path_length / self.robot_speed
            finish_time = self.total_inspection_time + travel_time + self.inspection_time

            self.total_path_length += path_length
            self.total_inspection_time = finish_time
            selected_task.mark_completed()
            self.current_pos = selected_task.position
            self.task_sequence.append(selected_task.task_id)
            self.task_finish_times[selected_task.task_id] = finish_time
            self.selection_records.append(
                {
                    "task_id": selected_task.task_id,
                    "path_length": path_length,
                    "astar_only_cost": cost,
                    "priority": selected_task.priority,
                    "risk": selected_task.risk,
                    "abnormal_weight": selected_task.abnormal_weight,
                    "finish_time": finish_time,
                    "travel_time": travel_time,
                    "turn_count": path_info["turn_count"],
                    "obstacle_nearby_count": path_info["obstacle_nearby_count"],
                    "reachable": True,
                    "method_detail": "astar_only_cost",
                }
            )
        return self.build_result()
