import copy
import csv
import random
import statistics
import time
from pathlib import Path

from advanced_baselines import (
    DeadlineGreedyAllocator,
    PriorityGreedyAllocator,
    TSP2OptAllocator,
)
from baseline_methods import AStarOnlyAllocator, NearestNeighborAllocator
from drf_rh_allocator import DRFRHAllocator
from receding_horizon_allocator_v2 import RHProposedAllocatorV2
from task_allocator import PriorityCostTaskAllocator
from task_model import InspectionTask


BALANCED_WEIGHTS = {
    "alpha": 0.22,
    "beta": 0.18,
    "lambda_abnormal": 0.15,
    "gamma": 0.27,
    "delta": 0.12,
    "eta": 0.06,
}


def create_grid_map(width, height, obstacle_ratio, start_pos, seed):
    random.seed(seed)
    grid_map = [[0 for _ in range(width)] for _ in range(height)]
    obstacle_num = int(width * height * obstacle_ratio)
    candidates = [(x, y) for y in range(height) for x in range(width) if (x, y) != start_pos]
    random.shuffle(candidates)
    for x, y in candidates[:obstacle_num]:
        grid_map[y][x] = 1
    sx, sy = start_pos
    grid_map[sy][sx] = 0
    return grid_map


def get_free_cells(grid_map):
    return [
        (x, y)
        for y, row in enumerate(grid_map)
        for x, value in enumerate(row)
        if value == 0
    ]


def create_tasks(grid_map, start_pos, task_num, seed):
    rng = random.Random(seed + 1000)
    free_cells = [cell for cell in get_free_cells(grid_map) if cell != start_pos]
    selected = rng.sample(free_cells, min(task_num, len(free_cells)))
    tasks = []
    for i, (x, y) in enumerate(selected):
        tasks.append(
            InspectionTask(
                task_id=f"P{i + 1}",
                x=x,
                y=y,
                priority=rng.random(),
                risk=rng.random(),
                abnormal_weight=0.0,
                status=0,
            )
        )
    return tasks


def compute_priority_weighted_completion_time(tasks, task_finish_times):
    weighted_sum = 0.0
    priority_sum = 0.0
    for task in tasks:
        if task.task_id in task_finish_times:
            weighted_sum += task.priority * task_finish_times[task.task_id]
            priority_sum += task.priority
    return weighted_sum / priority_sum if priority_sum else 0.0


def compute_high_priority_top5_rate(tasks, task_sequence):
    if not task_sequence:
        return 0.0
    lookup = {task.task_id: task for task in tasks}
    top5 = task_sequence[:5]
    count = sum(1 for task_id in top5 if lookup[task_id].priority >= 0.75)
    return count / min(5, len(top5)) * 100.0


def add_missing_metrics(result, allocator):
    result.setdefault(
        "priority_weighted_completion_time",
        compute_priority_weighted_completion_time(allocator.tasks, allocator.task_finish_times),
    )
    result.setdefault(
        "high_priority_top5_rate",
        compute_high_priority_top5_rate(allocator.tasks, allocator.task_sequence),
    )
    result.setdefault("completed_task_num", len(allocator.task_sequence))
    result.setdefault("selection_records", allocator.selection_records)
    return result


def build_allocator(method, grid_map, tasks, start_pos, robot_speed=0.6, inspection_time=5.0, **kwargs):
    if method == "NNF":
        return NearestNeighborAllocator(grid_map, tasks, start_pos, robot_speed, inspection_time)
    if method == "AStarOnly":
        return AStarOnlyAllocator(grid_map, tasks, start_pos, robot_speed, inspection_time)
    if method == "Proposed-Balanced":
        return PriorityCostTaskAllocator(
            grid_map,
            tasks,
            start_pos,
            robot_speed=robot_speed,
            inspection_time=inspection_time,
            **BALANCED_WEIGHTS,
        )
    if method == "RH-Proposed-v2":
        return RHProposedAllocatorV2(
            grid_map,
            tasks,
            start_pos,
            robot_speed=robot_speed,
            inspection_time=inspection_time,
            **BALANCED_WEIGHTS,
            horizon=4,
            beam_width=8,
            candidate_pool_size=10,
        )
    if method == "Priority-Greedy":
        return PriorityGreedyAllocator(grid_map, tasks, start_pos, robot_speed, inspection_time)
    if method == "Deadline-Greedy":
        return DeadlineGreedyAllocator(grid_map, tasks, start_pos, robot_speed, inspection_time)
    if method == "TSP-2opt":
        return TSP2OptAllocator(grid_map, tasks, start_pos, robot_speed, inspection_time)
    if method == "DRF-RH-Full":
        return DRFRHAllocator(
            grid_map,
            tasks,
            start_pos,
            robot_speed=robot_speed,
            inspection_time=inspection_time,
            **BALANCED_WEIGHTS,
            horizon=4,
            beam_width=8,
            candidate_pool_size=10,
            **kwargs,
        )
    if method == "DRF-RH-Light":
        return DRFRHAllocator(
            grid_map,
            tasks,
            start_pos,
            robot_speed=robot_speed,
            inspection_time=inspection_time,
            **BALANCED_WEIGHTS,
            horizon=3,
            beam_width=5,
            candidate_pool_size=6,
            **kwargs,
        )
    raise ValueError(f"Unknown method: {method}")


def run_allocator(method, grid_map, tasks, start_pos, robot_speed=0.6, inspection_time=5.0, **kwargs):
    tasks_copy = copy.deepcopy(tasks)
    allocator = build_allocator(
        method,
        grid_map,
        tasks_copy,
        start_pos,
        robot_speed,
        inspection_time,
        **kwargs,
    )
    t0 = time.perf_counter()
    result = allocator.run()
    runtime_ms = (time.perf_counter() - t0) * 1000.0
    result = add_missing_metrics(result, allocator)
    result["algorithm_runtime_ms"] = runtime_ms
    return result


def summarize(rows, group_keys, metrics):
    groups = {}
    for row in rows:
        key = tuple(row[k] for k in group_keys)
        groups.setdefault(key, []).append(row)

    summary = []
    for key, items in sorted(groups.items()):
        out = {k: v for k, v in zip(group_keys, key)}
        for metric in metrics:
            values = [float(item[metric]) for item in items if item.get(metric, "") != ""]
            out[f"{metric}_mean"] = statistics.mean(values) if values else 0.0
            out[f"{metric}_std"] = statistics.stdev(values) if len(values) > 1 else 0.0
        summary.append(out)
    return summary


def write_csv(path, rows, fieldnames):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def percent_change(a, b):
    if abs(b) < 1e-9:
        return 0.0
    return (a - b) / b * 100.0


def print_summary_table(summary, method_key="method"):
    print(
        "Method | Path Mean | Time Mean | High Priority Response Mean | "
        "Priority Weighted Completion Mean | Top5 High Priority Rate Mean | Runtime Mean(ms)"
    )
    for row in summary:
        print(
            f"{row[method_key]} | "
            f"{float(row.get('total_path_length_mean', 0)):.2f} | "
            f"{float(row.get('total_inspection_time_mean', 0)):.2f} | "
            f"{float(row.get('high_priority_avg_response_time_mean', 0)):.2f} | "
            f"{float(row.get('priority_weighted_completion_time_mean', 0)):.2f} | "
            f"{float(row.get('high_priority_top5_rate_mean', 0)):.2f} | "
            f"{float(row.get('algorithm_runtime_ms_mean', 0)):.2f}"
        )
