import copy
import csv
import json
import os
import random
import sys
import time
from collections import defaultdict

import numpy as np

if __package__ is None or __package__ == "":
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from inspection_task_allocator.baseline_methods import (
        AStarOnlyAllocator,
        FixedSequenceAllocator,
        NearestNeighborAllocator,
    )
    from inspection_task_allocator.task_allocator import PriorityCostTaskAllocator
    from inspection_task_allocator.task_model import InspectionTask
else:
    from .baseline_methods import AStarOnlyAllocator, FixedSequenceAllocator, NearestNeighborAllocator
    from .task_allocator import PriorityCostTaskAllocator
    from .task_model import InspectionTask


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(ROOT_DIR, "results")
CSV_PATH = os.path.join(RESULTS_DIR, "all_methods_results.csv")
JSON_PATH = os.path.join(RESULTS_DIR, "all_methods_results.json")

MAP_SIZE = (30, 30)
OBSTACLE_RATIOS = [0.1, 0.2, 0.3]
TASK_NUMS = [10, 20, 30]
REPEATS = 20
START = (2, 2)
METHODS = [
    ("FS", FixedSequenceAllocator),
    ("NNF", NearestNeighborAllocator),
    ("AStarOnly", AStarOnlyAllocator),
    ("Proposed", PriorityCostTaskAllocator),
]


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


def _normalize_result(method, result):
    if hasattr(result, "task_sequence"):
        task_sequence = result.task_sequence
        completed_task_num = result.completed_task_num
        selection_records = result.selection_records
    else:
        task_sequence = result.sequence
        completed_task_num = len(result.sequence)
        selection_records = result.task_details

    return task_sequence, completed_task_num, selection_records


def run_single_method(method_name, allocator_cls, grid, tasks, run_id, seed, obstacle_ratio, task_num):
    task_copy = copy.deepcopy(tasks)
    allocator = allocator_cls(
        grid_map=grid.tolist(),
        start=START,
        tasks=task_copy,
        robot_speed=0.6,
        inspection_time=5.0,
    )

    begin = time.perf_counter()
    result = allocator.allocate()
    algorithm_runtime_ms = (time.perf_counter() - begin) * 1000.0

    task_sequence, completed_task_num, selection_records = _normalize_result(method_name, result)

    csv_row = {
        "run_id": run_id,
        "seed": seed,
        "map_size": f"{MAP_SIZE[0]}x{MAP_SIZE[1]}",
        "obstacle_ratio": obstacle_ratio,
        "task_num": task_num,
        "method": method_name,
        "total_path_length": result.total_path_length,
        "total_inspection_time": result.total_inspection_time,
        "high_priority_avg_response_time": result.high_priority_avg_response_time,
        "completed_task_num": completed_task_num,
        "task_sequence": " -> ".join(task_sequence),
        "algorithm_runtime_ms": round(algorithm_runtime_ms, 3),
    }

    json_row = dict(csv_row)
    json_row["selection_records"] = selection_records
    return csv_row, json_row


def save_csv(rows, path):
    fieldnames = [
        "run_id",
        "seed",
        "map_size",
        "obstacle_ratio",
        "task_num",
        "method",
        "total_path_length",
        "total_inspection_time",
        "high_priority_avg_response_time",
        "completed_task_num",
        "task_sequence",
        "algorithm_runtime_ms",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def save_json(rows, path):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(rows, f, ensure_ascii=False, indent=2)


def print_summary(rows):
    print(f"实验总次数: {len(rows)}")
    print(f"CSV 保存路径: {CSV_PATH}")
    print(f"JSON 保存路径: {JSON_PATH}")

    by_method_path = defaultdict(list)
    by_method_time = defaultdict(list)
    by_method_response = defaultdict(list)
    for row in rows:
        by_method_path[row["method"]].append(row["total_path_length"])
        by_method_time[row["method"]].append(row["total_inspection_time"])
        by_method_response[row["method"]].append(row["high_priority_avg_response_time"])

    for method, _ in METHODS:
        path_vals = by_method_path.get(method, [])
        time_vals = by_method_time.get(method, [])
        resp_vals = by_method_response.get(method, [])
        print(f"{method} 平均 total_path_length: {sum(path_vals) / len(path_vals):.3f}")
        print(f"{method} 平均 total_inspection_time: {sum(time_vals) / len(time_vals):.3f}")
        print(f"{method} 平均 high_priority_avg_response_time: {sum(resp_vals) / len(resp_vals):.3f}")


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    csv_rows = []
    json_rows = []
    run_id = 0

    for obstacle_ratio in OBSTACLE_RATIOS:
        for task_num in TASK_NUMS:
            for repeat_idx in range(REPEATS):
                obstacle_index = OBSTACLE_RATIOS.index(obstacle_ratio)
                seed = 1000 + obstacle_index * 100 + task_num * 10 + repeat_idx
                grid = generate_map(MAP_SIZE[0], MAP_SIZE[1], obstacle_ratio=obstacle_ratio, start=START, seed=seed)
                tasks = create_tasks(grid, start=START, count=task_num, seed=seed + 1)

                for method_name, allocator_cls in METHODS:
                    run_id += 1
                    csv_row, json_row = run_single_method(
                        method_name=method_name,
                        allocator_cls=allocator_cls,
                        grid=grid,
                        tasks=tasks,
                        run_id=run_id,
                        seed=seed,
                        obstacle_ratio=obstacle_ratio,
                        task_num=task_num,
                    )
                    csv_rows.append(csv_row)
                    json_rows.append(json_row)

    save_csv(csv_rows, CSV_PATH)
    save_json(json_rows, JSON_PATH)
    print_summary(csv_rows)


if __name__ == "__main__":
    main()
