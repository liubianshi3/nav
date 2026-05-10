import copy
import math
import random
import time
from pathlib import Path

from advanced_baselines import DeadlineGreedyAllocator, PriorityGreedyAllocator
from baseline_methods import AStarOnlyAllocator
from drf_rh_allocator import DRFRHAllocator
from drf_rh_experiment_utils import (
    BALANCED_WEIGHTS,
    create_grid_map,
    create_tasks,
    percent_change,
    print_summary_table,
    summarize,
    write_csv,
)
from receding_horizon_allocator_v2 import RHProposedAllocatorV2
from task_allocator import PriorityCostTaskAllocator


METHODS = [
    "AStarOnly",
    "Proposed-Balanced",
    "RH-v2-Full",
    "RH-v2-Light",
    "DRF-RH-Full",
    "DRF-RH-Light",
    "Priority-Greedy",
    "Deadline-Greedy",
]

METRICS = [
    "completed_task_num",
    "total_path_length",
    "total_inspection_time",
    "high_priority_avg_response_time",
    "priority_weighted_completion_time",
    "high_priority_top5_rate",
    "abnormal_priority_rate",
    "abnormal_avg_response_time",
    "replanning_count",
    "algorithm_runtime_ms",
]


def build_allocator(method, grid_map, tasks, start_pos, robot_speed, inspection_time):
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
    if method == "RH-v2-Full":
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
    if method == "RH-v2-Light":
        return RHProposedAllocatorV2(
            grid_map,
            tasks,
            start_pos,
            robot_speed=robot_speed,
            inspection_time=inspection_time,
            **BALANCED_WEIGHTS,
            horizon=3,
            beam_width=5,
            candidate_pool_size=6,
        )
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
        )
    if method == "Priority-Greedy":
        return PriorityGreedyAllocator(grid_map, tasks, start_pos, robot_speed, inspection_time)
    if method == "Deadline-Greedy":
        return DeadlineGreedyAllocator(grid_map, tasks, start_pos, robot_speed, inspection_time)
    raise ValueError(method)


def select_astar_only(allocator):
    candidates = []
    for task in allocator.get_unfinished_tasks():
        path_info = allocator.planner.plan(allocator.current_pos, task.position)
        if path_info.get("reachable", False):
            cost = (
                path_info["path_length"]
                + 0.5 * path_info["turn_count"]
                + 0.2 * path_info["obstacle_nearby_count"]
            )
            candidates.append((cost, task.task_id, task, path_info))
    if not candidates:
        return None, None, None
    candidates.sort(key=lambda item: (item[0], item[1]))
    cost, _, task, path_info = candidates[0]
    return task, path_info, {
        "task_id": task.task_id,
        "astar_only_cost": cost,
        "path_length": path_info["path_length"],
        "priority": task.priority,
        "risk": task.risk,
        "abnormal_weight": task.abnormal_weight,
    }


def select_next(allocator, method):
    if method == "AStarOnly":
        return select_astar_only(allocator)
    if method.startswith("DRF-RH"):
        return allocator.select_next_task(allocator.current_pos, allocator.total_inspection_time)
    if method in {"Priority-Greedy", "Deadline-Greedy"}:
        return allocator.select_next_task()
    return allocator.select_next_task(allocator.current_pos)


def execute(allocator, task, path_info, record):
    path_length = path_info["path_length"]
    travel_time = path_length / allocator.robot_speed
    finish_time = allocator.total_inspection_time + travel_time + allocator.inspection_time
    allocator.total_path_length += path_length
    allocator.total_inspection_time = finish_time
    task.mark_completed()
    allocator.current_pos = task.position
    allocator.task_sequence.append(task.task_id)
    allocator.task_finish_times[task.task_id] = finish_time
    record["finish_time"] = finish_time
    record["travel_time"] = travel_time
    allocator.selection_records.append(record)


def sample_abnormal_tasks(allocator, seed):
    remaining = [task for task in allocator.tasks if task.status == 0]
    rng = random.Random(seed + 5000)
    return rng.sample(remaining, min(4, len(remaining))) if remaining else []


def static_abnormal_update(allocator, abnormal_tasks, rho=0.5, sigma=5.0):
    abnormal_ids = {task.task_id for task in abnormal_tasks}
    for task in allocator.tasks:
        if task.status == 1:
            continue
        if task.task_id in abnormal_ids:
            task.abnormal_weight = 1.0
        elif abnormal_tasks:
            distance = min(
                abs(task.x - abnormal.x) + abs(task.y - abnormal.y)
                for abnormal in abnormal_tasks
            )
            task.abnormal_weight = min(
                1.0,
                max(task.abnormal_weight, rho * math.exp(-distance / sigma)),
            )


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
    count = sum(1 for task_id in task_sequence[:5] if lookup[task_id].priority >= 0.75)
    return count / min(5, len(task_sequence)) * 100.0


def compute_abnormal_metrics(task_sequence, abnormal_ids, trigger_time, finish_times, total_time):
    if not abnormal_ids:
        return 0.0, 0.0
    first_k = task_sequence[3 : 3 + len(abnormal_ids)]
    rate = len(set(first_k) & set(abnormal_ids)) / len(abnormal_ids) * 100.0
    responses = [
        (finish_times[task_id] if task_id in finish_times else total_time) - trigger_time
        for task_id in abnormal_ids
    ]
    return rate, sum(responses) / len(responses)


def run_method(method, grid_map, tasks, start_pos, seed, robot_speed, inspection_time):
    tasks_copy = copy.deepcopy(tasks)
    allocator = build_allocator(method, grid_map, tasks_copy, start_pos, robot_speed, inspection_time)
    abnormal_ids = []
    trigger_time = 0.0
    replanning_count = 0
    started = time.perf_counter()
    while allocator.get_unfinished_tasks():
        selected, path_info, record = select_next(allocator, method)
        if selected is None:
            break
        execute(allocator, selected, path_info, record)
        if len(allocator.task_sequence) == 3:
            trigger_time = allocator.total_inspection_time
            abnormal_tasks = sample_abnormal_tasks(allocator, seed)
            abnormal_ids = [task.task_id for task in abnormal_tasks]
            if method.startswith("DRF-RH"):
                for event_task in abnormal_tasks:
                    allocator.add_abnormal_event(
                        f"{seed}_{event_task.task_id}",
                        event_task.position,
                        trigger_time,
                        intensity=1.0,
                    )
                allocator.update_dynamic_risk(trigger_time)
                replanning_count = 1
            elif method != "AStarOnly":
                static_abnormal_update(allocator, abnormal_tasks)
                replanning_count = 1
    runtime_ms = (time.perf_counter() - started) * 1000.0
    abnormal_priority_rate, abnormal_avg_response_time = compute_abnormal_metrics(
        allocator.task_sequence,
        abnormal_ids,
        trigger_time,
        allocator.task_finish_times,
        allocator.total_inspection_time,
    )
    return {
        "completed_task_num": len(allocator.task_sequence),
        "total_path_length": allocator.total_path_length,
        "total_inspection_time": allocator.total_inspection_time,
        "high_priority_avg_response_time": allocator.compute_high_priority_avg_response_time(),
        "priority_weighted_completion_time": compute_priority_weighted_completion_time(
            allocator.tasks,
            allocator.task_finish_times,
        ),
        "high_priority_top5_rate": compute_high_priority_top5_rate(
            allocator.tasks,
            allocator.task_sequence,
        ),
        "abnormal_task_ids": ",".join(abnormal_ids),
        "abnormal_priority_rate": abnormal_priority_rate,
        "abnormal_avg_response_time": abnormal_avg_response_time,
        "replanning_count": replanning_count,
        "algorithm_runtime_ms": runtime_ms,
        "task_sequence": "->".join(allocator.task_sequence),
    }


def main():
    width = 30
    height = 30
    obstacle_ratio = 0.2
    task_num = 20
    start_pos = (2, 2)
    robot_speed = 0.6
    inspection_time = 5.0
    seeds = list(range(30))

    rows = []
    for seed in seeds:
        grid_map = create_grid_map(width, height, obstacle_ratio, start_pos, seed)
        tasks = create_tasks(grid_map, start_pos, task_num, seed)
        for method in METHODS:
            result = run_method(method, grid_map, tasks, start_pos, seed, robot_speed, inspection_time)
            row = {"seed": seed, "method": method}
            row.update(result)
            rows.append(row)

    base_dir = Path(__file__).resolve().parent
    results_path = base_dir / "results" / "drf_rh_abnormal_results.csv"
    summary_path = base_dir / "results" / "drf_rh_abnormal_summary.csv"
    fields = [
        "seed",
        "method",
        "completed_task_num",
        "total_path_length",
        "total_inspection_time",
        "high_priority_avg_response_time",
        "priority_weighted_completion_time",
        "high_priority_top5_rate",
        "abnormal_task_ids",
        "abnormal_priority_rate",
        "abnormal_avg_response_time",
        "replanning_count",
        "algorithm_runtime_ms",
        "task_sequence",
    ]
    write_csv(results_path, rows, fields)
    summary = summarize(rows, ["method"], METRICS)
    write_csv(summary_path, summary, list(summary[0].keys()))

    print(f"Total rows: {len(rows)}")
    print({m: sum(1 for row in rows if row['method'] == m) for m in METHODS})
    print_summary_table(summary)
    row_map = {row["method"]: row for row in summary}
    for target in ["RH-v2-Full", "RH-v2-Light", "Proposed-Balanced", "AStarOnly"]:
        source = "DRF-RH-Light" if target == "RH-v2-Light" else "DRF-RH-Full"
        print(f"{source} vs {target}:")
        for metric in [
            "abnormal_avg_response_time",
            "abnormal_priority_rate",
            "high_priority_avg_response_time",
            "total_inspection_time",
            "algorithm_runtime_ms",
        ]:
            change = percent_change(
                float(row_map[source][f"{metric}_mean"]),
                float(row_map[target][f"{metric}_mean"]),
            )
            print(f"- {metric}: {change:.2f}%")
    print(f"Saved: {results_path}")
    print(f"Saved: {summary_path}")


if __name__ == "__main__":
    main()
