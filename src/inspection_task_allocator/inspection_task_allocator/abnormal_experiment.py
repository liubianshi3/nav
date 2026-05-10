import copy
import json
import os
import random
import sys
import time

import numpy as np
import pandas as pd

if __package__ is None or __package__ == "":
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from inspection_task_allocator.astar_planner import AStarPlanner
    from inspection_task_allocator.baseline_methods import (
        AStarOnlyAllocator,
        FixedSequenceAllocator,
        NearestNeighborAllocator,
    )
    from inspection_task_allocator.task_allocator import PriorityCostTaskAllocator
    from inspection_task_allocator.task_model import InspectionTask
else:
    from .astar_planner import AStarPlanner
    from .baseline_methods import AStarOnlyAllocator, FixedSequenceAllocator, NearestNeighborAllocator
    from .task_allocator import PriorityCostTaskAllocator
    from .task_model import InspectionTask


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(ROOT_DIR, "results")
CSV_PATH = os.path.join(RESULTS_DIR, "abnormal_results.csv")
JSON_PATH = os.path.join(RESULTS_DIR, "abnormal_results.json")
SUMMARY_PATH = os.path.join(RESULTS_DIR, "abnormal_summary.csv")

MAP_SIZE = (30, 30)
OBSTACLE_RATIO = 0.2
TASK_NUM = 20
REPEATS = 20
START = (2, 2)
METHODS = [
    ("FS", FixedSequenceAllocator),
    ("NNF", NearestNeighborAllocator),
    ("AStarOnly", AStarOnlyAllocator),
    ("Proposed", PriorityCostTaskAllocator),
]
ROBOT_SPEED = 0.6
INSPECTION_TIME = 5.0
TRIGGER_STEP = 3
ABNORMAL_TASK_NUM = 4
HIGH_PRIORITY_THRESHOLD = 0.75
HIGH_RISK_THRESHOLD = 0.75


def generate_map(width, height, obstacle_ratio, start, seed):
    rng = random.Random(seed)
    grid = np.zeros((height, width), dtype=int)
    protected = {start}
    candidates = [(x, y) for y in range(height) for x in range(width) if (x, y) not in protected]
    rng.shuffle(candidates)
    obstacle_count = int(width * height * obstacle_ratio)
    for x, y in candidates[:obstacle_count]:
        grid[y][x] = 1
    return grid


def reachable_cells(grid, start):
    height, width = grid.shape
    stack = [start]
    visited = {start}
    while stack:
        x, y = stack.pop()
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= nx < width and 0 <= ny < height and grid[ny][nx] == 0 and (nx, ny) not in visited:
                visited.add((nx, ny))
                stack.append((nx, ny))
    visited.discard(start)
    return list(visited)


def create_tasks(grid, start, count, seed):
    rng = random.Random(seed)
    reachable = reachable_cells(grid, start)
    rng.shuffle(reachable)
    tasks = []
    for idx, (x, y) in enumerate(reachable[:count]):
        tasks.append(
            InspectionTask(
                task_id=f"P{idx + 1}",
                x=x,
                y=y,
                priority=round(rng.random(), 3),
                risk=round(rng.random(), 3),
                abnormal_weight=0.0,
            )
        )
    return tasks


def clone_tasks(tasks):
    return copy.deepcopy(tasks)


def _average(values):
    return sum(values) / len(values) if values else 0.0


def _replay_sequence_metrics(sequence, task_lookup, planner):
    completion_times = {}
    high_priority_response_times = []
    high_risk_response_times = []
    cumulative_time = 0.0
    current_pos = START

    for task_id in sequence:
        task = task_lookup[task_id]
        path, path_length, _, _ = planner.plan(current_pos, (task.x, task.y))
        if not path:
            continue
        cumulative_time += path_length / ROBOT_SPEED + INSPECTION_TIME
        completion_times[task_id] = cumulative_time
        if task.priority >= HIGH_PRIORITY_THRESHOLD:
            high_priority_response_times.append(cumulative_time)
        if task.risk >= HIGH_RISK_THRESHOLD:
            high_risk_response_times.append(cumulative_time)
        current_pos = (task.x, task.y)

    return {
        "completion_times": completion_times,
        "total_time": cumulative_time,
        "high_priority_avg_response_time": _average(high_priority_response_times),
        "high_risk_avg_response_time": _average(high_risk_response_times),
    }


def _compute_abnormal_metrics(sequence, abnormal_task_ids, completion_times, trigger_time, total_time):
    abnormal_set = set(abnormal_task_ids)
    completed_before_trigger = set(sequence[: min(TRIGGER_STEP, len(sequence))])
    active_abnormal_set = abnormal_set - completed_before_trigger
    if not active_abnormal_set:
        return 0.0, 0.0, active_abnormal_set

    post_trigger_sequence = sequence[TRIGGER_STEP : TRIGGER_STEP + len(active_abnormal_set)]
    abnormal_hit_count = sum(1 for tid in post_trigger_sequence if tid in active_abnormal_set)
    abnormal_priority_rate = abnormal_hit_count / len(active_abnormal_set) * 100.0

    response_times = []
    for task_id in active_abnormal_set:
        if task_id in completion_times:
            response_times.append(max(completion_times[task_id] - trigger_time, 0.0))
        else:
            response_times.append(max(total_time - trigger_time, 0.0))

    return abnormal_priority_rate, _average(response_times), active_abnormal_set


def _build_baseline_result(grid, tasks, allocator_cls, abnormal_task_ids):
    task_copy = clone_tasks(tasks)
    allocator = allocator_cls(
        grid_map=grid.tolist(),
        start=START,
        tasks=task_copy,
        robot_speed=ROBOT_SPEED,
        inspection_time=INSPECTION_TIME,
    )
    result = allocator.allocate()

    sequence = result.task_sequence
    planner = AStarPlanner(grid.tolist())
    task_lookup = {task.task_id: task for task in task_copy}
    metrics = _replay_sequence_metrics(sequence, task_lookup, planner)
    trigger_task_id = sequence[TRIGGER_STEP - 1] if len(sequence) >= TRIGGER_STEP else None
    trigger_time = metrics["completion_times"].get(trigger_task_id, metrics["total_time"])
    abnormal_priority_rate, abnormal_avg_response_time, active_abnormal_set = _compute_abnormal_metrics(
        sequence,
        abnormal_task_ids,
        metrics["completion_times"],
        trigger_time,
        metrics["total_time"],
    )

    return {
        "task_sequence": sequence,
        "total_path_length": result.total_path_length,
        "total_inspection_time": result.total_inspection_time,
        "high_priority_avg_response_time": metrics["high_priority_avg_response_time"],
        "high_risk_avg_response_time": metrics["high_risk_avg_response_time"],
        "completed_task_num": result.completed_task_num,
        "selection_records": result.selection_records,
        "replanning_count": 0,
        "abnormal_priority_rate": abnormal_priority_rate,
        "abnormal_avg_response_time": abnormal_avg_response_time,
        "active_abnormal_task_ids": sorted(active_abnormal_set),
    }


def run_fixed_sequence(grid, tasks, abnormal_task_ids):
    return _build_baseline_result(grid, tasks, FixedSequenceAllocator, abnormal_task_ids)


def run_nearest_neighbor(grid, tasks, abnormal_task_ids):
    return _build_baseline_result(grid, tasks, NearestNeighborAllocator, abnormal_task_ids)


def run_astar_only(grid, tasks, abnormal_task_ids):
    return _build_baseline_result(grid, tasks, AStarOnlyAllocator, abnormal_task_ids)


def run_proposed_abnormal(grid, tasks, abnormal_task_ids, rho=0.5, sigma=5.0):
    task_copy = clone_tasks(tasks)
    planner = AStarPlanner(grid.tolist())
    task_map = {task.task_id: task for task in task_copy}
    allocator = PriorityCostTaskAllocator(
        grid_map=grid.tolist(),
        start=START,
        tasks=task_copy,
        robot_speed=ROBOT_SPEED,
        inspection_time=INSPECTION_TIME,
    )
    current_pos = START
    sequence = []
    selection_records = []
    total_path_length = 0.0
    total_inspection_time = 0.0
    completed = set()
    trigger_time = None
    replanning_count = 0
    order = 0

    def execute_candidate(candidate, phase):
        nonlocal current_pos, order, total_path_length, total_inspection_time
        task = candidate["task"]
        order += 1
        path_length = candidate["path_length"]
        travel_time = path_length / ROBOT_SPEED if ROBOT_SPEED > 0 else 0.0
        total_path_length += path_length
        total_inspection_time += travel_time + INSPECTION_TIME
        completed.add(task.task_id)
        task.status = 1
        sequence.append(task.task_id)
        selection_records.append(
            {
                "order": order,
                "task_id": task.task_id,
                "score": candidate["score"],
                "path_length": path_length,
                "turn_count": candidate["turn_count"],
                "obstacle_nearby_count": candidate["obstacle_nearby_count"],
                "distance_cost": candidate["distance_cost"],
                "d_norm": candidate["distance_cost"],
                "complexity_cost": candidate["complexity_cost"],
                "complexity": candidate["complexity_cost"],
                "energy_cost": candidate["energy_cost"],
                "energy": candidate["energy_cost"],
                "abnormal_weight": task.abnormal_weight,
                "phase": phase,
            }
        )
        current_pos = (task.x, task.y)

    while order < TRIGGER_STEP:
        remaining = [t for t in task_copy if t.task_id not in completed]
        if not remaining:
            break

        chosen = allocator.select_next_task(current_pos, remaining)
        if chosen is None:
            break
        execute_candidate(chosen, "pre_abnormal")

    if order >= TRIGGER_STEP:
        trigger_time = total_inspection_time

    if trigger_time is not None:
        replanning_count = 1
        abnormal_set = set(abnormal_task_ids) - completed
        abnormal_tasks = [task_map[task_id] for task_id in abnormal_set if task_id in task_map]
        for task in abnormal_tasks:
            task.abnormal_weight = 1.0

        for task in task_copy:
            if task.task_id in completed:
                continue
            if task.task_id in abnormal_set:
                task.abnormal_weight = 1.0
                continue
            if not abnormal_tasks:
                continue
            distance_to_abnormal = min(
                abs(task.x - abnormal_task.x) + abs(task.y - abnormal_task.y)
                for abnormal_task in abnormal_tasks
            )
            task.abnormal_weight = min(
                1.0,
                max(task.abnormal_weight, rho * np.exp(-distance_to_abnormal / sigma)),
            )

        while True:
            remaining = [t for t in task_copy if t.task_id not in completed]
            if not remaining:
                break
            chosen = allocator.select_next_task(current_pos, remaining)
            if chosen is None:
                break
            execute_candidate(chosen, "post_abnormal")

    metrics = _replay_sequence_metrics(sequence, task_map, planner)
    if trigger_time is None:
        trigger_task_id = sequence[TRIGGER_STEP - 1] if len(sequence) >= TRIGGER_STEP else None
        trigger_time = metrics["completion_times"].get(trigger_task_id, metrics["total_time"])
    abnormal_priority_rate, abnormal_avg_response_time, active_abnormal_set = _compute_abnormal_metrics(
        sequence,
        abnormal_task_ids,
        metrics["completion_times"],
        trigger_time,
        metrics["total_time"],
    )

    return {
        "task_sequence": sequence,
        "total_path_length": total_path_length,
        "total_inspection_time": total_inspection_time,
        "high_priority_avg_response_time": metrics["high_priority_avg_response_time"],
        "high_risk_avg_response_time": metrics["high_risk_avg_response_time"],
        "completed_task_num": len(sequence),
        "selection_records": selection_records,
        "replanning_count": replanning_count,
        "abnormal_priority_rate": abnormal_priority_rate,
        "abnormal_avg_response_time": abnormal_avg_response_time,
        "active_abnormal_task_ids": sorted(active_abnormal_set),
    }


def sample_abnormal_task_ids(grid, tasks, seed):
    task_copy = clone_tasks(tasks)
    allocator = PriorityCostTaskAllocator(
        grid_map=grid.tolist(),
        start=START,
        tasks=task_copy,
        robot_speed=ROBOT_SPEED,
        inspection_time=INSPECTION_TIME,
    )
    completed = set()
    current_pos = START

    for _ in range(TRIGGER_STEP):
        remaining = [task for task in task_copy if task.task_id not in completed]
        chosen = allocator.select_next_task(current_pos, remaining)
        if chosen is None:
            break
        task = chosen["task"]
        completed.add(task.task_id)
        task.status = 1
        current_pos = (task.x, task.y)

    candidates = [task for task in task_copy if task.task_id not in completed]
    rng = random.Random(seed)
    sample_count = min(ABNORMAL_TASK_NUM, len(candidates))
    return [task.task_id for task in rng.sample(candidates, sample_count)]


def summarize(df):
    summary = df.groupby("method", as_index=False).agg(
        replanning_count_mean=("replanning_count", "mean"),
        replanning_count_std=("replanning_count", "std"),
        abnormal_priority_rate_mean=("abnormal_priority_rate", "mean"),
        abnormal_priority_rate_std=("abnormal_priority_rate", "std"),
        abnormal_avg_response_time_mean=("abnormal_avg_response_time", "mean"),
        abnormal_avg_response_time_std=("abnormal_avg_response_time", "std"),
        high_risk_avg_response_time_mean=("high_risk_avg_response_time", "mean"),
        high_risk_avg_response_time_std=("high_risk_avg_response_time", "std"),
        total_path_length_mean=("total_path_length", "mean"),
        total_path_length_std=("total_path_length", "std"),
        total_inspection_time_mean=("total_inspection_time", "mean"),
        total_inspection_time_std=("total_inspection_time", "std"),
    )
    return summary


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    rows = []
    json_rows = []
    run_id = 0

    for repeat_idx in range(REPEATS):
        seed = 5000 + repeat_idx
        grid = generate_map(MAP_SIZE[0], MAP_SIZE[1], OBSTACLE_RATIO, START, seed)
        tasks = create_tasks(grid, START, TASK_NUM, seed + 1)
        abnormal_task_ids = sample_abnormal_task_ids(grid, tasks, seed + 2)

        for method_name, allocator_cls in METHODS:
            run_id += 1
            if method_name == "Proposed":
                result = run_proposed_abnormal(grid, tasks, abnormal_task_ids)
            elif method_name == "FS":
                result = run_fixed_sequence(grid, tasks, abnormal_task_ids)
            elif method_name == "NNF":
                result = run_nearest_neighbor(grid, tasks, abnormal_task_ids)
            else:
                result = run_astar_only(grid, tasks, abnormal_task_ids)

            csv_row = {
                "run_id": run_id,
                "method": method_name,
                "seed": seed,
                "abnormal_task_ids": "|".join(abnormal_task_ids),
                "task_sequence": " -> ".join(result["task_sequence"]),
                "replanning_count": result["replanning_count"],
                "abnormal_priority_rate": result.get("abnormal_priority_rate", 0.0),
                "abnormal_avg_response_time": result.get("abnormal_avg_response_time", 0.0),
                "high_risk_avg_response_time": result["high_risk_avg_response_time"],
                "total_path_length": result["total_path_length"],
                "total_inspection_time": result["total_inspection_time"],
                "completed_task_num": result["completed_task_num"],
            }
            rows.append(csv_row)
            json_row = dict(csv_row)
            json_row["selection_records"] = result["selection_records"]
            json_row["active_abnormal_task_ids"] = result.get("active_abnormal_task_ids", [])
            json_rows.append(json_row)

    df = pd.DataFrame(rows)
    df.to_csv(CSV_PATH, index=False)
    with open(JSON_PATH, "w", encoding="utf-8") as f:
        json.dump(json_rows, f, ensure_ascii=False, indent=2)

    summary = summarize(df)
    summary.to_csv(SUMMARY_PATH, index=False)

    print(f"实验总次数: {len(rows)}")
    print(f"abnormal_results.csv 保存路径: {CSV_PATH}")
    print(f"abnormal_summary.csv 保存路径: {SUMMARY_PATH}")

    for method in ["FS", "NNF", "AStarOnly", "Proposed"]:
        subset = df[df["method"] == method]
        print(f"{method} abnormal_priority_rate 均值: {subset['abnormal_priority_rate'].mean():.3f}")
        print(f"{method} abnormal_avg_response_time 均值: {subset['abnormal_avg_response_time'].mean():.3f}")

    proposed = df[df["method"] == "Proposed"]["abnormal_avg_response_time"].mean()
    astar = df[df["method"] == "AStarOnly"]["abnormal_avg_response_time"].mean()
    if astar > 0:
        reduction = (astar - proposed) / astar * 100.0
        print(f"Proposed 相比 AStarOnly 在 abnormal_avg_response_time 上的降低比例: {reduction:.2f}%")
    else:
        print("Proposed 相比 AStarOnly 在 abnormal_avg_response_time 上的降低比例: 0.00%")


if __name__ == "__main__":
    main()
