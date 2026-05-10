import copy
import csv
import statistics
from pathlib import Path
import random

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


def run_method(allocator_class, grid_map, tasks, start_pos):
    tasks_copy = copy.deepcopy(tasks)
    allocator = allocator_class(
        grid_map=grid_map,
        tasks=tasks_copy,
        start_pos=start_pos,
        robot_speed=0.6,
        inspection_time=5.0,
    )
    return allocator.run()


def save_csv(rows, path):
    fieldnames = [
        "seed",
        "method",
        "completed_task_num",
        "total_path_length",
        "total_inspection_time",
        "high_priority_avg_response_time",
        "task_sequence",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def summarize_by_method(rows):
    by_method = {}
    for row in rows:
        by_method.setdefault(row["method"], []).append(row)

    summary_rows = []
    for method in ["FS", "NNF", "AStarOnly", "Proposed"]:
        items = by_method.get(method, [])
        completed = [item["completed_task_num"] for item in items]
        path = [item["total_path_length"] for item in items]
        time_vals = [item["total_inspection_time"] for item in items]
        resp = [item["high_priority_avg_response_time"] for item in items]
        summary_rows.append(
            {
                "method": method,
                "completed_task_num_mean": statistics.mean(completed) if completed else 0.0,
                "completed_task_num_std": statistics.stdev(completed) if len(completed) > 1 else 0.0,
                "total_path_length_mean": statistics.mean(path) if path else 0.0,
                "total_path_length_std": statistics.stdev(path) if len(path) > 1 else 0.0,
                "total_inspection_time_mean": statistics.mean(time_vals) if time_vals else 0.0,
                "total_inspection_time_std": statistics.stdev(time_vals) if len(time_vals) > 1 else 0.0,
                "high_priority_avg_response_time_mean": statistics.mean(resp) if resp else 0.0,
                "high_priority_avg_response_time_std": statistics.stdev(resp) if len(resp) > 1 else 0.0,
            }
        )
    return summary_rows


def save_summary_csv(summary_rows, path):
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
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)


def print_summary_table(summary_rows):
    print("\nMethod | Completed Mean | Path Mean | Path Std | Time Mean | Time Std | High Priority Response Mean | High Priority Response Std")
    for row in summary_rows:
        print(
            f"{row['method']} | {row['completed_task_num_mean']:.2f} | {row['total_path_length_mean']:.2f} | "
            f"{row['total_path_length_std']:.2f} | {row['total_inspection_time_mean']:.2f} | {row['total_inspection_time_std']:.2f} | "
            f"{row['high_priority_avg_response_time_mean']:.2f} | {row['high_priority_avg_response_time_std']:.2f}"
        )


def main():
    width = 30
    height = 30
    obstacle_ratio = 0.2
    task_num = 20
    start_pos = (2, 2)
    seeds = list(range(30))

    results_dir = Path(__file__).resolve().parent / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    results_path = results_dir / "batch_compare_results.csv"
    summary_path = results_dir / "batch_compare_summary.csv"

    methods = [
        ("FS", FixedSequenceAllocator),
        ("NNF", NearestNeighborAllocator),
        ("AStarOnly", AStarOnlyAllocator),
        ("Proposed", PriorityCostTaskAllocator),
    ]

    rows = []
    for seed in seeds:
        grid_map = create_grid_map(width, height, obstacle_ratio, start_pos, seed)
        tasks = create_tasks(grid_map, start_pos, task_num, seed)
        for method_name, allocator_class in methods:
            result = run_method(allocator_class, grid_map, tasks, start_pos)
            rows.append(
                {
                    "seed": seed,
                    "method": method_name,
                    "completed_task_num": result["completed_task_num"],
                    "total_path_length": result["total_path_length"],
                    "total_inspection_time": result["total_inspection_time"],
                    "high_priority_avg_response_time": result["high_priority_avg_response_time"],
                    "task_sequence": "->".join(result["task_sequence"]),
                }
            )

    save_csv(rows, results_path)
    summary_rows = summarize_by_method(rows)
    save_summary_csv(summary_rows, summary_path)

    print("Experiment settings:")
    print(f"- map size: {width}x{height}")
    print(f"- obstacle ratio: {obstacle_ratio}")
    print(f"- task num: {task_num}")
    print(f"- seed count: {len(seeds)}")
    print(f"\nTotal experiment rows: {len(rows)}")

    by_method_count = {method: 0 for method, _ in methods}
    for row in rows:
        by_method_count[row["method"]] += 1
    for method in ["FS", "NNF", "AStarOnly", "Proposed"]:
        print(f"{method}: {by_method_count[method]} rows")

    print_summary_table(summary_rows)

    summary_map = {row["method"]: row for row in summary_rows}
    a = summary_map["AStarOnly"]
    p = summary_map["Proposed"]
    path_change = (p["total_path_length_mean"] - a["total_path_length_mean"]) / a["total_path_length_mean"] * 100 if a["total_path_length_mean"] else 0.0
    time_change = (p["total_inspection_time_mean"] - a["total_inspection_time_mean"]) / a["total_inspection_time_mean"] * 100 if a["total_inspection_time_mean"] else 0.0
    response_change = (p["high_priority_avg_response_time_mean"] - a["high_priority_avg_response_time_mean"]) / a["high_priority_avg_response_time_mean"] * 100 if a["high_priority_avg_response_time_mean"] else 0.0

    print("\nProposed vs AStarOnly mean change:")
    print(f"- total_path_length: {path_change:.2f}%")
    print(f"- total_inspection_time: {time_change:.2f}%")
    print(f"- high_priority_avg_response_time: {response_change:.2f}%")

    print(f"\nResults saved to: {results_path}")
    print(f"Summary saved to: {summary_path}")


if __name__ == "__main__":
    main()
