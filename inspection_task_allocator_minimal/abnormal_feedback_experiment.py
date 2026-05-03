import copy
import csv
import math
import random
import statistics
from pathlib import Path

from task_model import InspectionTask
from task_allocator import PriorityCostTaskAllocator
from baseline_methods import (
    FixedSequenceAllocator,
    NearestNeighborAllocator,
    AStarOnlyAllocator,
)


def create_grid_map(width, height, obstacle_ratio, start_pos, seed):
    random.seed(seed)
    grid_map = [[0 for _ in range(width)] for _ in range(height)]
    total_cells = width * height
    obstacle_num = int(total_cells * obstacle_ratio)
    candidates = [(x, y) for y in range(height) for x in range(width) if (x, y) != start_pos]
    random.shuffle(candidates)
    for x, y in candidates[:obstacle_num]:
        grid_map[y][x] = 1
    sx, sy = start_pos
    grid_map[sy][sx] = 0
    return grid_map


def get_free_cells(grid_map):
    free_cells = []
    for y, row in enumerate(grid_map):
        for x, value in enumerate(row):
            if value == 0:
                free_cells.append((x, y))
    return free_cells


def create_tasks(grid_map, start_pos, task_num, seed):
    random.seed(seed + 1000)
    free_cells = [cell for cell in get_free_cells(grid_map) if cell != start_pos]
    if len(free_cells) < task_num:
        raise ValueError("可用空闲点不足 task_num，无法生成任务。")
    selected_cells = random.sample(free_cells, task_num)
    tasks = []
    for i, (x, y) in enumerate(selected_cells):
        tasks.append(
            InspectionTask(
                task_id=f"P{i + 1}",
                x=x,
                y=y,
                priority=random.random(),
                risk=random.random(),
                abnormal_weight=0.0,
                status=0,
            )
        )
    return tasks


def build_allocator(method, grid_map, tasks, start_pos, weights=None):
    tasks_copy = copy.deepcopy(tasks)
    if method == "FS":
        return FixedSequenceAllocator(grid_map=grid_map, tasks=tasks_copy, start_pos=start_pos, robot_speed=0.6, inspection_time=5.0)
    if method == "NNF":
        return NearestNeighborAllocator(grid_map=grid_map, tasks=tasks_copy, start_pos=start_pos, robot_speed=0.6, inspection_time=5.0)
    if method == "AStarOnly":
        return AStarOnlyAllocator(grid_map=grid_map, tasks=tasks_copy, start_pos=start_pos, robot_speed=0.6, inspection_time=5.0)
    return PriorityCostTaskAllocator(
        grid_map=grid_map,
        tasks=tasks_copy,
        start_pos=start_pos,
        robot_speed=0.6,
        inspection_time=5.0,
        alpha=weights["alpha"],
        beta=weights["beta"],
        lambda_abnormal=weights["lambda_abnormal"],
        gamma=weights["gamma"],
        delta=weights["delta"],
        eta=weights["eta"],
    )


def execute_selected_task(allocator, selected_task, path_info, record):
    path_length = path_info["path_length"]
    travel_time = path_length / allocator.robot_speed
    finish_time = allocator.total_inspection_time + travel_time + allocator.inspection_time
    allocator.total_path_length += path_length
    allocator.total_inspection_time = finish_time
    selected_task.mark_completed()
    allocator.current_pos = selected_task.position
    allocator.task_sequence.append(selected_task.task_id)
    allocator.task_finish_times[selected_task.task_id] = finish_time
    record["finish_time"] = finish_time
    record["travel_time"] = travel_time
    allocator.selection_records.append(record)


def compute_abnormal_metrics(task_sequence, abnormal_task_ids, abnormal_trigger_time, task_finish_times, total_inspection_time):
    if not abnormal_task_ids:
        return 0.0, 0.0
    post_sequence = task_sequence[3:]
    first_k_after_trigger = post_sequence[: len(abnormal_task_ids)]
    abnormal_priority_rate = len(set(first_k_after_trigger) & set(abnormal_task_ids)) / len(abnormal_task_ids) * 100.0

    response_times = []
    for task_id in abnormal_task_ids:
        if task_id in task_finish_times:
            response_times.append(task_finish_times[task_id] - abnormal_trigger_time)
        else:
            response_times.append(total_inspection_time - abnormal_trigger_time)
    abnormal_avg_response_time = sum(response_times) / len(response_times)
    return abnormal_priority_rate, abnormal_avg_response_time


def step_run_fixed_sequence(allocator, abnormal_tasks, trigger_done):
    for task in allocator.tasks:
        if task.status == 1:
            continue
        path_info = allocator.planner.plan(allocator.current_pos, task.position)
        if not path_info["reachable"]:
            allocator.selection_records.append({"task_id": task.task_id, "reachable": False, "method_detail": "fixed_sequence_unreachable"})
            continue
        record = {
            "task_id": task.task_id,
            "path_length": path_info["path_length"],
            "priority": task.priority,
            "risk": task.risk,
            "abnormal_weight": task.abnormal_weight,
            "finish_time": None,
            "travel_time": None,
            "reachable": True,
            "method_detail": "fixed_sequence",
        }
        execute_selected_task(allocator, task, path_info, record)
        if len(allocator.task_sequence) == 3 and not trigger_done[0]:
            trigger_done[0] = True
        break


def run_fs_with_abnormal_tracking(grid_map, tasks, start_pos, seed):
    allocator = build_allocator("FS", grid_map, tasks, start_pos)
    trigger_done = [False]
    abnormal_task_ids = []
    abnormal_trigger_time = 0.0
    while True:
        unfinished = [t for t in allocator.tasks if t.status == 0]
        if not unfinished:
            break
        selected = None
        for task in allocator.tasks:
            if task.status == 1:
                continue
            path_info = allocator.planner.plan(allocator.current_pos, task.position)
            if not path_info["reachable"]:
                allocator.selection_records.append({"task_id": task.task_id, "reachable": False, "method_detail": "fixed_sequence_unreachable"})
                continue
            selected = (task, path_info)
            break
        if selected is None:
            print("No reachable unfinished tasks. Stop FS.")
            break
        task, path_info = selected
        record = {
            "task_id": task.task_id,
            "path_length": path_info["path_length"],
            "priority": task.priority,
            "risk": task.risk,
            "abnormal_weight": task.abnormal_weight,
            "finish_time": None,
            "travel_time": None,
            "reachable": True,
            "method_detail": "fixed_sequence",
        }
        execute_selected_task(allocator, task, path_info, record)
        if len(allocator.task_sequence) == 3 and not trigger_done[0]:
            trigger_done[0] = True
            abnormal_trigger_time = allocator.total_inspection_time
            remaining = [t for t in allocator.tasks if t.status == 0]
            sample_n = min(4, len(remaining))
            rng = random.Random(seed + 5000)
            abnormal_tasks = rng.sample(remaining, sample_n) if sample_n > 0 else []
            abnormal_task_ids = [t.task_id for t in abnormal_tasks]
        continue
    if not trigger_done[0]:
        abnormal_trigger_time = allocator.total_inspection_time
        remaining = [t for t in allocator.tasks if t.status == 0]
        sample_n = min(4, len(remaining))
        rng = random.Random(seed + 5000)
        abnormal_tasks = rng.sample(remaining, sample_n) if sample_n > 0 else []
        abnormal_task_ids = [t.task_id for t in abnormal_tasks]
    abnormal_priority_rate, abnormal_avg_response_time = compute_abnormal_metrics(
        allocator.task_sequence, abnormal_task_ids, abnormal_trigger_time, allocator.task_finish_times, allocator.total_inspection_time
    )
    return {
        "task_sequence": allocator.task_sequence,
        "total_path_length": allocator.total_path_length,
        "total_inspection_time": allocator.total_inspection_time,
        "high_priority_avg_response_time": allocator.compute_high_priority_avg_response_time(),
        "completed_task_num": len(allocator.task_sequence),
        "abnormal_task_ids": ",".join(abnormal_task_ids),
        "abnormal_priority_rate": abnormal_priority_rate,
        "abnormal_avg_response_time": abnormal_avg_response_time,
        "replanning_count": 0,
        "selection_records": allocator.selection_records,
    }


def run_nnf_with_abnormal_tracking(grid_map, tasks, start_pos, seed):
    allocator = build_allocator("NNF", grid_map, tasks, start_pos)
    trigger_done = False
    abnormal_task_ids = []
    abnormal_trigger_time = 0.0
    while True:
        unfinished = [t for t in allocator.tasks if t.status == 0]
        if not unfinished:
            break
        candidates = []
        for task in unfinished:
            path_info = allocator.planner.plan(allocator.current_pos, task.position)
            if path_info["reachable"]:
                candidates.append((path_info["path_length"], task.task_id, task, path_info))
        if not candidates:
            print("No reachable unfinished tasks. Stop NNF.")
            break
        candidates.sort(key=lambda x: (x[0], x[1]))
        _, _, task, path_info = candidates[0]
        record = {
            "task_id": task.task_id,
            "path_length": path_info["path_length"],
            "priority": task.priority,
            "risk": task.risk,
            "abnormal_weight": task.abnormal_weight,
            "finish_time": None,
            "travel_time": None,
            "turn_count": path_info["turn_count"],
            "obstacle_nearby_count": path_info["obstacle_nearby_count"],
            "method_detail": "nearest_neighbor_astar_length",
        }
        execute_selected_task(allocator, task, path_info, record)
        if len(allocator.task_sequence) == 3 and not trigger_done:
            trigger_done = True
            abnormal_trigger_time = allocator.total_inspection_time
            remaining = [t for t in allocator.tasks if t.status == 0]
            sample_n = min(4, len(remaining))
            rng = random.Random(seed + 5000)
            abnormal_tasks = rng.sample(remaining, sample_n) if sample_n > 0 else []
            abnormal_task_ids = [t.task_id for t in abnormal_tasks]
    if not trigger_done:
        abnormal_trigger_time = allocator.total_inspection_time
        remaining = [t for t in allocator.tasks if t.status == 0]
        sample_n = min(4, len(remaining))
        rng = random.Random(seed + 5000)
        abnormal_tasks = rng.sample(remaining, sample_n) if sample_n > 0 else []
        abnormal_task_ids = [t.task_id for t in abnormal_tasks]
    abnormal_priority_rate, abnormal_avg_response_time = compute_abnormal_metrics(
        allocator.task_sequence, abnormal_task_ids, abnormal_trigger_time, allocator.task_finish_times, allocator.total_inspection_time
    )
    return {
        "task_sequence": allocator.task_sequence,
        "total_path_length": allocator.total_path_length,
        "total_inspection_time": allocator.total_inspection_time,
        "high_priority_avg_response_time": allocator.compute_high_priority_avg_response_time(),
        "completed_task_num": len(allocator.task_sequence),
        "abnormal_task_ids": ",".join(abnormal_task_ids),
        "abnormal_priority_rate": abnormal_priority_rate,
        "abnormal_avg_response_time": abnormal_avg_response_time,
        "replanning_count": 0,
        "selection_records": allocator.selection_records,
    }


def run_astar_only_with_abnormal_tracking(grid_map, tasks, start_pos, seed):
    allocator = build_allocator("AStarOnly", grid_map, tasks, start_pos)
    trigger_done = False
    abnormal_task_ids = []
    abnormal_trigger_time = 0.0
    while True:
        unfinished = [t for t in allocator.tasks if t.status == 0]
        if not unfinished:
            break
        candidates = []
        for task in unfinished:
            path_info = allocator.planner.plan(allocator.current_pos, task.position)
            if path_info["reachable"]:
                cost = path_info["path_length"] + 0.5 * path_info["turn_count"] + 0.2 * path_info["obstacle_nearby_count"]
                candidates.append((cost, task.task_id, task, path_info))
        if not candidates:
            print("No reachable unfinished tasks. Stop AStarOnly.")
            break
        candidates.sort(key=lambda x: (x[0], x[1]))
        cost, _, task, path_info = candidates[0]
        record = {
            "task_id": task.task_id,
            "path_length": path_info["path_length"],
            "astar_only_cost": cost,
            "priority": task.priority,
            "risk": task.risk,
            "abnormal_weight": task.abnormal_weight,
            "finish_time": None,
            "travel_time": None,
            "turn_count": path_info["turn_count"],
            "obstacle_nearby_count": path_info["obstacle_nearby_count"],
            "method_detail": "astar_only_cost",
        }
        execute_selected_task(allocator, task, path_info, record)
        if len(allocator.task_sequence) == 3 and not trigger_done:
            trigger_done = True
            abnormal_trigger_time = allocator.total_inspection_time
            remaining = [t for t in allocator.tasks if t.status == 0]
            sample_n = min(4, len(remaining))
            rng = random.Random(seed + 5000)
            abnormal_tasks = rng.sample(remaining, sample_n) if sample_n > 0 else []
            abnormal_task_ids = [t.task_id for t in abnormal_tasks]
    if not trigger_done:
        abnormal_trigger_time = allocator.total_inspection_time
        remaining = [t for t in allocator.tasks if t.status == 0]
        sample_n = min(4, len(remaining))
        rng = random.Random(seed + 5000)
        abnormal_tasks = rng.sample(remaining, sample_n) if sample_n > 0 else []
        abnormal_task_ids = [t.task_id for t in abnormal_tasks]
    abnormal_priority_rate, abnormal_avg_response_time = compute_abnormal_metrics(
        allocator.task_sequence, abnormal_task_ids, abnormal_trigger_time, allocator.task_finish_times, allocator.total_inspection_time
    )
    return {
        "task_sequence": allocator.task_sequence,
        "total_path_length": allocator.total_path_length,
        "total_inspection_time": allocator.total_inspection_time,
        "high_priority_avg_response_time": allocator.compute_high_priority_avg_response_time(),
        "completed_task_num": len(allocator.task_sequence),
        "abnormal_task_ids": ",".join(abnormal_task_ids),
        "abnormal_priority_rate": abnormal_priority_rate,
        "abnormal_avg_response_time": abnormal_avg_response_time,
        "replanning_count": 0,
        "selection_records": allocator.selection_records,
    }


def run_proposed_balanced_with_abnormal(grid_map, tasks, start_pos, seed):
    tasks_copy = copy.deepcopy(tasks)
    allocator = PriorityCostTaskAllocator(
        grid_map=grid_map,
        tasks=tasks_copy,
        start_pos=start_pos,
        robot_speed=0.6,
        inspection_time=5.0,
        alpha=0.22,
        beta=0.18,
        lambda_abnormal=0.15,
        gamma=0.27,
        delta=0.12,
        eta=0.06,
    )
    abnormal_task_ids = []
    abnormal_trigger_time = 0.0
    replanning_count = 0
    abnormal_updated = False
    while True:
        unfinished = allocator.get_unfinished_tasks()
        if not unfinished:
            break
        selected_task, path_info, record = allocator.select_next_task(allocator.current_pos)
        if selected_task is None:
            print("No reachable unfinished tasks. Stop allocation.")
            break
        execute_selected_task(allocator, selected_task, path_info, record)
        if len(allocator.task_sequence) == 3 and replanning_count == 0:
            abnormal_trigger_time = allocator.total_inspection_time
            remaining = [t for t in allocator.tasks if t.status == 0]
            sample_n = min(4, len(remaining))
            rng = random.Random(seed + 5000)
            abnormal_tasks = rng.sample(remaining, sample_n) if sample_n > 0 else []
            abnormal_task_ids = [t.task_id for t in abnormal_tasks]
            for task in allocator.tasks:
                if task.status == 1:
                    continue
                if task.task_id in abnormal_task_ids:
                    task.abnormal_weight = 1.0
                    abnormal_updated = True
                else:
                    distance_to_abnormal = min(
                        abs(task.x - ab_task.x) + abs(task.y - ab_task.y)
                        for ab_task in abnormal_tasks
                    ) if abnormal_tasks else 0
                    new_weight = max(task.abnormal_weight, 0.5 * math.exp(-distance_to_abnormal / 5.0))
                    task.abnormal_weight = min(1.0, new_weight)
                    if task.abnormal_weight > 0:
                        abnormal_updated = True
            replanning_count = 1
    if not abnormal_task_ids:
        remaining = [t for t in allocator.tasks if t.status == 0]
        sample_n = min(4, len(remaining))
        rng = random.Random(seed + 5000)
        abnormal_tasks = rng.sample(remaining, sample_n) if sample_n > 0 else []
        abnormal_task_ids = [t.task_id for t in abnormal_tasks]
    abnormal_priority_rate, abnormal_avg_response_time = compute_abnormal_metrics(
        allocator.task_sequence, abnormal_task_ids, abnormal_trigger_time, allocator.task_finish_times, allocator.total_inspection_time
    )
    print(f"Proposed abnormal_weight updated: {abnormal_updated}")
    return {
        "task_sequence": allocator.task_sequence,
        "total_path_length": allocator.total_path_length,
        "total_inspection_time": allocator.total_inspection_time,
        "high_priority_avg_response_time": allocator.compute_high_priority_avg_response_time(),
        "completed_task_num": len(allocator.task_sequence),
        "abnormal_task_ids": ",".join(abnormal_task_ids),
        "abnormal_priority_rate": abnormal_priority_rate,
        "abnormal_avg_response_time": abnormal_avg_response_time,
        "replanning_count": replanning_count,
        "selection_records": allocator.selection_records,
    }


def save_results(rows, path):
    fieldnames = [
        "seed",
        "method",
        "completed_task_num",
        "total_path_length",
        "total_inspection_time",
        "high_priority_avg_response_time",
        "abnormal_task_ids",
        "abnormal_priority_rate",
        "abnormal_avg_response_time",
        "replanning_count",
        "task_sequence",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def summarize(rows):
    groups = {}
    for row in rows:
        groups.setdefault(row["method"], []).append(row)
    summary = []
    for method in ["FS", "NNF", "AStarOnly", "Proposed-Balanced"]:
        items = groups.get(method, [])
        def vals(key):
            return [item[key] for item in items]
        summary.append(
            {
                "method": method,
                "completed_task_num_mean": statistics.mean(vals("completed_task_num")) if items else 0.0,
                "completed_task_num_std": statistics.stdev(vals("completed_task_num")) if len(items) > 1 else 0.0,
                "total_path_length_mean": statistics.mean(vals("total_path_length")) if items else 0.0,
                "total_path_length_std": statistics.stdev(vals("total_path_length")) if len(items) > 1 else 0.0,
                "total_inspection_time_mean": statistics.mean(vals("total_inspection_time")) if items else 0.0,
                "total_inspection_time_std": statistics.stdev(vals("total_inspection_time")) if len(items) > 1 else 0.0,
                "high_priority_avg_response_time_mean": statistics.mean(vals("high_priority_avg_response_time")) if items else 0.0,
                "high_priority_avg_response_time_std": statistics.stdev(vals("high_priority_avg_response_time")) if len(items) > 1 else 0.0,
                "abnormal_priority_rate_mean": statistics.mean(vals("abnormal_priority_rate")) if items else 0.0,
                "abnormal_priority_rate_std": statistics.stdev(vals("abnormal_priority_rate")) if len(items) > 1 else 0.0,
                "abnormal_avg_response_time_mean": statistics.mean(vals("abnormal_avg_response_time")) if items else 0.0,
                "abnormal_avg_response_time_std": statistics.stdev(vals("abnormal_avg_response_time")) if len(items) > 1 else 0.0,
                "replanning_count_mean": statistics.mean(vals("replanning_count")) if items else 0.0,
                "replanning_count_std": statistics.stdev(vals("replanning_count")) if len(items) > 1 else 0.0,
            }
        )
    return summary


def save_summary(summary, path):
    fieldnames = [
        "method",
        "completed_task_num_mean",
        "completed_task_num_std",
        "total_path_length_mean",
        "total_path_length_std",
        "total_inspection_time_mean",
        "total_inspection_time_std",
        "high_priority_avg_response_time_mean",
        "high_priority_avg_response_time_std",
        "abnormal_priority_rate_mean",
        "abnormal_priority_rate_std",
        "abnormal_avg_response_time_mean",
        "abnormal_avg_response_time_std",
        "replanning_count_mean",
        "replanning_count_std",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary)


def main():
    width = 30
    height = 30
    obstacle_ratio = 0.2
    task_num = 20
    start_pos = (2, 2)
    seeds = list(range(30))
    methods = ["FS", "NNF", "AStarOnly", "Proposed-Balanced"]
    results_dir = Path(__file__).resolve().parent / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    results_path = results_dir / "abnormal_feedback_results.csv"
    summary_path = results_dir / "abnormal_feedback_summary.csv"
    all_rows = []

    for seed in seeds:
        grid_map = create_grid_map(width, height, obstacle_ratio, start_pos, seed)
        tasks = create_tasks(grid_map, start_pos, task_num, seed)
        for method in methods:
            if method == "FS":
                result = run_fs_with_abnormal_tracking(grid_map, tasks, start_pos, seed)
            elif method == "NNF":
                result = run_nnf_with_abnormal_tracking(grid_map, tasks, start_pos, seed)
            elif method == "AStarOnly":
                result = run_astar_only_with_abnormal_tracking(grid_map, tasks, start_pos, seed)
            else:
                result = run_proposed_balanced_with_abnormal(grid_map, tasks, start_pos, seed)
            all_rows.append(
                {
                    "seed": seed,
                    "method": method,
                    "completed_task_num": result["completed_task_num"],
                    "total_path_length": result["total_path_length"],
                    "total_inspection_time": result["total_inspection_time"],
                    "high_priority_avg_response_time": result["high_priority_avg_response_time"],
                    "abnormal_task_ids": result["abnormal_task_ids"],
                    "abnormal_priority_rate": result["abnormal_priority_rate"],
                    "abnormal_avg_response_time": result["abnormal_avg_response_time"],
                    "replanning_count": result["replanning_count"],
                    "task_sequence": "->".join(result["task_sequence"]),
                }
            )

    save_results(all_rows, results_path)
    summary = summarize(all_rows)
    save_summary(summary, summary_path)

    print("Experiment settings:")
    print(f"- map size: {width}x{height}")
    print(f"- obstacle ratio: {obstacle_ratio}")
    print(f"- task num: {task_num}")
    print(f"- seed count: {len(seeds)}")
    print(f"- methods: {', '.join(methods)}")
    print(f"\nTotal experiment rows: {len(all_rows)}")

    counts = {m: 0 for m in methods}
    for row in all_rows:
        counts[row["method"]] += 1
    for method in methods:
        print(f"{method}: {counts[method]} rows")

    print("\nMethod | Completed Mean | Path Mean | Time Mean | High Priority Response Mean | Abnormal Priority Rate Mean | Abnormal Response Mean | Replanning Mean")
    summary_map = {row["method"]: row for row in summary}
    for method in methods:
        row = summary_map[method]
        print(
            f"{method} | {row['completed_task_num_mean']:.2f} | {row['total_path_length_mean']:.2f} | {row['total_inspection_time_mean']:.2f} | "
            f"{row['high_priority_avg_response_time_mean']:.2f} | {row['abnormal_priority_rate_mean']:.2f} | {row['abnormal_avg_response_time_mean']:.2f} | {row['replanning_count_mean']:.2f}"
        )

    a = summary_map["AStarOnly"]
    p = summary_map["Proposed-Balanced"]
    path_change = (p["total_path_length_mean"] - a["total_path_length_mean"]) / a["total_path_length_mean"] * 100 if a["total_path_length_mean"] else 0.0
    time_change = (p["total_inspection_time_mean"] - a["total_inspection_time_mean"]) / a["total_inspection_time_mean"] * 100 if a["total_inspection_time_mean"] else 0.0
    high_change = (p["high_priority_avg_response_time_mean"] - a["high_priority_avg_response_time_mean"]) / a["high_priority_avg_response_time_mean"] * 100 if a["high_priority_avg_response_time_mean"] else 0.0
    abnormal_rate_change = (p["abnormal_priority_rate_mean"] - a["abnormal_priority_rate_mean"]) / a["abnormal_priority_rate_mean"] * 100 if a["abnormal_priority_rate_mean"] else 0.0
    abnormal_response_change = (p["abnormal_avg_response_time_mean"] - a["abnormal_avg_response_time_mean"]) / a["abnormal_avg_response_time_mean"] * 100 if a["abnormal_avg_response_time_mean"] else 0.0

    print("\nProposed-Balanced vs AStarOnly mean change:")
    print(f"- total_path_length: {path_change:.2f}%")
    print(f"- total_inspection_time: {time_change:.2f}%")
    print(f"- high_priority_avg_response_time: {high_change:.2f}%")
    print(f"- abnormal_priority_rate: {abnormal_rate_change:.2f}%")
    print(f"- abnormal_avg_response_time: {abnormal_response_change:.2f}%")

    print(f"\nResults saved to: {results_path}")
    print(f"Summary saved to: {summary_path}")


if __name__ == "__main__":
    main()
