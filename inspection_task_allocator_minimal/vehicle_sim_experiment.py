import csv
import json
import random
import statistics
from pathlib import Path

from task_model import InspectionTask
from vehicle_simulator import VehicleTaskExecutionSimulator


def create_grid_map(width, height, obstacle_ratio, start_pos, seed):
    random.seed(seed)
    grid_map = [[0 for _ in range(width)] for _ in range(height)]
    total_cells = width * height
    obstacle_num = int(total_cells * obstacle_ratio)
    candidates = [(x, y) for y in range(height) for x in range(width) if (x, y) != start_pos]
    random.shuffle(candidates)
    for x, y in candidates[:obstacle_num]:
        grid_map[y][x] = 1
    grid_map[start_pos[1]][start_pos[0]] = 0
    return grid_map


def get_free_cells(grid_map):
    cells = []
    for y, row in enumerate(grid_map):
        for x, value in enumerate(row):
            if value == 0:
                cells.append((x, y))
    return cells


def create_tasks(grid_map, start_pos, task_num, seed):
    random.seed(seed + 1000)
    free_cells = [cell for cell in get_free_cells(grid_map) if cell != start_pos]
    if len(free_cells) < task_num:
        raise ValueError("free cells not enough")
    selected = random.sample(free_cells, task_num)
    tasks = []
    for i, (x, y) in enumerate(selected):
        tasks.append(InspectionTask(f"P{i+1}", x, y, random.random(), random.random(), 0.0, 0))
    return tasks


def save_csv(rows, path):
    fieldnames = [
        "seed", "method", "completed_task_num", "total_planned_path_length", "vehicle_trajectory_length",
        "vehicle_execution_time", "total_inspection_time", "high_priority_avg_response_time",
        "abnormal_avg_response_time", "abnormal_priority_rate", "heading_change_sum", "goal_success_rate",
        "trajectory_to_plan_ratio", "failed_task_num", "unreachable_task_num", "follower_failed_num",
        "adapter_failed_num", "task_sequence"
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def save_json(rows, path):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(rows, f, ensure_ascii=False, indent=2)


def summarize(rows):
    by_method = {}
    for row in rows:
        by_method.setdefault(row["method"], []).append(row)
    summary = []
    methods = ["AStarOnly", "Proposed-Balanced", "RH-v2-Light", "TSP-2opt", "Priority-Greedy"]
    keys = [
        "completed_task_num", "total_planned_path_length", "vehicle_trajectory_length",
        "vehicle_execution_time", "total_inspection_time", "high_priority_avg_response_time",
        "heading_change_sum", "goal_success_rate", "trajectory_to_plan_ratio",
        "failed_task_num", "unreachable_task_num", "follower_failed_num", "adapter_failed_num",
    ]
    for method in methods:
        items = by_method.get(method, [])
        row = {"method": method}
        for key in keys:
            vals = [float(x[key]) for x in items] if items else []
            row[f"{key}_mean"] = statistics.mean(vals) if vals else 0.0
            row[f"{key}_std"] = statistics.stdev(vals) if len(vals) > 1 else 0.0
        summary.append(row)
    return summary


def main():
    width = 30
    height = 30
    obstacle_ratio = 0.2
    task_num = 20
    start_pos = (2, 2)
    seeds = list(range(30))
    methods = ["AStarOnly", "Proposed-Balanced", "RH-v2-Light", "TSP-2opt", "Priority-Greedy"]

    results_dir = Path(__file__).resolve().parent / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    csv_path = results_dir / "vehicle_sim_results.csv"
    summary_path = results_dir / "vehicle_sim_summary.csv"
    json_path = results_dir / "vehicle_sim_records.json"

    rows = []
    records = []
    for seed in seeds:
        grid_map = create_grid_map(width, height, obstacle_ratio, start_pos, seed)
        tasks = create_tasks(grid_map, start_pos, task_num, seed)
        for method in methods:
            sim = VehicleTaskExecutionSimulator(
                grid_map=grid_map,
                tasks=tasks,
                allocator_name=method,
                start_pos=start_pos,
                start_theta=0.0,
                robot_speed=0.6,
                inspection_time=5.0,
                seed=seed,
            )
            result = sim.run()
            rows.append({
                "seed": seed,
                "method": method,
                "completed_task_num": result["completed_task_num"],
                "total_planned_path_length": result["total_planned_path_length"],
                "vehicle_trajectory_length": result["vehicle_trajectory_length"],
                "vehicle_execution_time": result["vehicle_execution_time"],
                "total_inspection_time": result["total_inspection_time"],
                "high_priority_avg_response_time": result["high_priority_avg_response_time"],
                "abnormal_avg_response_time": result["abnormal_avg_response_time"],
                "abnormal_priority_rate": result["abnormal_priority_rate"],
                "heading_change_sum": result["heading_change_sum"],
                "goal_success_rate": result["goal_success_rate"],
                "trajectory_to_plan_ratio": result["trajectory_to_plan_ratio"],
                "failed_task_num": result["failed_task_num"],
                "unreachable_task_num": result["unreachable_task_num"],
                "follower_failed_num": result["follower_failed_num"],
                "adapter_failed_num": result["adapter_failed_num"],
                "task_sequence": "->".join(result["task_sequence"]),
            })
            records.append({"seed": seed, **result})

    save_csv(rows, csv_path)
    save_json(records, json_path)
    summary = summarize(rows)
    summary_fieldnames = list(summary[0].keys()) if summary else []
    with open(summary_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=summary_fieldnames)
        writer.writeheader()
        writer.writerows(summary)

    print(f"Total rows: {len(rows)}")
    for method in methods:
        print(f"{method}: {sum(1 for r in rows if r['method'] == method)}")
    print("Summary:")
    for row in summary:
        print(row)


if __name__ == "__main__":
    main()
