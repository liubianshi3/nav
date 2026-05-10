import copy
import csv
import statistics
from pathlib import Path
import random

from task_model import InspectionTask
from task_allocator import PriorityCostTaskAllocator


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


def run_group(weight_group, weights, grid_map, tasks, start_pos):
    tasks_copy = copy.deepcopy(tasks)
    allocator = PriorityCostTaskAllocator(
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
    result = allocator.run()
    return {
        "weight_group": weight_group,
        "alpha": weights["alpha"],
        "beta": weights["beta"],
        "lambda_abnormal": weights["lambda_abnormal"],
        "gamma": weights["gamma"],
        "delta": weights["delta"],
        "eta": weights["eta"],
        "completed_task_num": result["completed_task_num"],
        "total_path_length": result["total_path_length"],
        "total_inspection_time": result["total_inspection_time"],
        "high_priority_avg_response_time": result["high_priority_avg_response_time"],
        "task_sequence": "->".join(result["task_sequence"]),
    }


def summarize(rows):
    groups = {}
    for row in rows:
        groups.setdefault(row["weight_group"], []).append(row)

    summary_rows = []
    for name in ["Default", "PathPriority", "TaskPriority", "Balanced", "WeakPathPenalty"]:
        items = groups.get(name, [])
        completed = [r["completed_task_num"] for r in items]
        path = [r["total_path_length"] for r in items]
        time_vals = [r["total_inspection_time"] for r in items]
        resp = [r["high_priority_avg_response_time"] for r in items]
        summary_rows.append(
            {
                "weight_group": name,
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


def save_csv(rows, path):
    fieldnames = [
        "seed",
        "weight_group",
        "alpha",
        "beta",
        "lambda_abnormal",
        "gamma",
        "delta",
        "eta",
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


def save_summary_csv(rows, path):
    fieldnames = [
        "weight_group",
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
        writer.writerows(rows)


def print_summary(summary_rows):
    print("\nWeight Group | Completed Mean | Path Mean | Path Std | Time Mean | Time Std | High Priority Response Mean | High Priority Response Std")
    for row in summary_rows:
        print(
            f"{row['weight_group']} | {row['completed_task_num_mean']:.2f} | {row['total_path_length_mean']:.2f} | {row['total_path_length_std']:.2f} | "
            f"{row['total_inspection_time_mean']:.2f} | {row['total_inspection_time_std']:.2f} | "
            f"{row['high_priority_avg_response_time_mean']:.2f} | {row['high_priority_avg_response_time_std']:.2f}"
        )


def print_changes(summary_rows):
    summary_map = {row["weight_group"]: row for row in summary_rows}
    default = summary_map["Default"]
    print("\nWeight group vs Default mean change:")
    for name in ["PathPriority", "TaskPriority", "Balanced", "WeakPathPenalty"]:
        row = summary_map[name]
        path_change = (row["total_path_length_mean"] - default["total_path_length_mean"]) / default["total_path_length_mean"] * 100 if default["total_path_length_mean"] else 0.0
        time_change = (row["total_inspection_time_mean"] - default["total_inspection_time_mean"]) / default["total_inspection_time_mean"] * 100 if default["total_inspection_time_mean"] else 0.0
        response_change = (row["high_priority_avg_response_time_mean"] - default["high_priority_avg_response_time_mean"]) / default["high_priority_avg_response_time_mean"] * 100 if default["high_priority_avg_response_time_mean"] else 0.0
        print(f"- {name}:")
        print(f"  total_path_length: {path_change:.2f}%")
        print(f"  total_inspection_time: {time_change:.2f}%")
        print(f"  high_priority_avg_response_time: {response_change:.2f}%")


def main():
    width = 30
    height = 30
    obstacle_ratio = 0.2
    task_num = 20
    start_pos = (2, 2)
    seeds = list(range(30))

    weight_groups = {
        "Default": {"alpha": 0.25, "beta": 0.20, "lambda_abnormal": 0.20, "gamma": 0.20, "delta": 0.10, "eta": 0.05},
        "PathPriority": {"alpha": 0.15, "beta": 0.15, "lambda_abnormal": 0.10, "gamma": 0.35, "delta": 0.15, "eta": 0.10},
        "TaskPriority": {"alpha": 0.35, "beta": 0.25, "lambda_abnormal": 0.15, "gamma": 0.15, "delta": 0.07, "eta": 0.03},
        "Balanced": {"alpha": 0.22, "beta": 0.18, "lambda_abnormal": 0.15, "gamma": 0.27, "delta": 0.12, "eta": 0.06},
        "WeakPathPenalty": {"alpha": 0.30, "beta": 0.25, "lambda_abnormal": 0.20, "gamma": 0.12, "delta": 0.08, "eta": 0.05},
    }

    results_dir = Path(__file__).resolve().parent / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    results_path = results_dir / "weight_sensitivity_results.csv"
    summary_path = results_dir / "weight_sensitivity_summary.csv"

    rows = []
    for seed in seeds:
        grid_map = create_grid_map(width, height, obstacle_ratio, start_pos, seed)
        tasks = create_tasks(grid_map, start_pos, task_num, seed)
        for group_name, weights in weight_groups.items():
            row = run_group(group_name, weights, grid_map, tasks, start_pos)
            row["seed"] = seed
            rows.append(row)

    save_csv(rows, results_path)
    summary_rows = summarize(rows)
    save_summary_csv(summary_rows, summary_path)

    print("Experiment settings:")
    print(f"- map size: {width}x{height}")
    print(f"- obstacle ratio: {obstacle_ratio}")
    print(f"- task num: {task_num}")
    print(f"- seed count: {len(seeds)}")
    print(f"- weight group count: {len(weight_groups)}")
    print(f"\nTotal experiment rows: {len(rows)}")

    counts = {name: 0 for name in weight_groups}
    for row in rows:
        counts[row["weight_group"]] += 1
    for name in ["Default", "PathPriority", "TaskPriority", "Balanced", "WeakPathPenalty"]:
        print(f"{name}: {counts[name]} rows")

    print_summary(summary_rows)
    print_changes(summary_rows)

    print(f"\nResults saved to: {results_path}")
    print(f"Summary saved to: {summary_path}")


if __name__ == "__main__":
    main()
