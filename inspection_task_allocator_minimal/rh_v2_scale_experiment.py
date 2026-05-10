import copy
import csv
import random
import statistics
import time
from pathlib import Path

from baseline_methods import AStarOnlyAllocator, NearestNeighborAllocator
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

METHODS = [
    "NNF",
    "AStarOnly",
    "Proposed-Balanced",
    "RH-Proposed-v2",
]

METRICS = [
    "completed_task_num",
    "total_path_length",
    "total_inspection_time",
    "high_priority_avg_response_time",
    "priority_weighted_completion_time",
    "high_priority_top5_rate",
    "algorithm_runtime_ms",
]


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


def compute_priority_weighted_completion_time(tasks, task_finish_times):
    weighted_sum = 0.0
    priority_sum = 0.0
    for task in tasks:
        if task.task_id not in task_finish_times:
            continue
        weighted_sum += task.priority * task_finish_times[task.task_id]
        priority_sum += task.priority
    if priority_sum == 0:
        return 0.0
    return weighted_sum / priority_sum


def compute_high_priority_top5_rate(tasks, task_sequence):
    if not task_sequence:
        return 0.0
    task_lookup = {task.task_id: task for task in tasks}
    top5 = task_sequence[:5]
    high_priority_count = sum(
        1 for task_id in top5 if task_lookup[task_id].priority >= 0.75
    )
    return high_priority_count / min(5, len(top5)) * 100.0


def build_allocator(method, grid_map, tasks, start_pos, robot_speed, inspection_time):
    if method == "NNF":
        return NearestNeighborAllocator(
            grid_map=grid_map,
            tasks=tasks,
            start_pos=start_pos,
            robot_speed=robot_speed,
            inspection_time=inspection_time,
        )
    if method == "AStarOnly":
        return AStarOnlyAllocator(
            grid_map=grid_map,
            tasks=tasks,
            start_pos=start_pos,
            robot_speed=robot_speed,
            inspection_time=inspection_time,
        )
    if method == "Proposed-Balanced":
        return PriorityCostTaskAllocator(
            grid_map=grid_map,
            tasks=tasks,
            start_pos=start_pos,
            robot_speed=robot_speed,
            inspection_time=inspection_time,
            **BALANCED_WEIGHTS,
        )
    return RHProposedAllocatorV2(
        grid_map=grid_map,
        tasks=tasks,
        start_pos=start_pos,
        robot_speed=robot_speed,
        inspection_time=inspection_time,
        **BALANCED_WEIGHTS,
        horizon=4,
        beam_width=8,
        candidate_pool_size=10,
    )


def run_method(method, grid_map, tasks, start_pos, robot_speed, inspection_time):
    tasks_copy = copy.deepcopy(tasks)
    allocator = build_allocator(
        method,
        grid_map,
        tasks_copy,
        start_pos,
        robot_speed,
        inspection_time,
    )

    start_time = time.perf_counter()
    result = allocator.run()
    end_time = time.perf_counter()
    algorithm_runtime_ms = (end_time - start_time) * 1000.0

    result.setdefault(
        "priority_weighted_completion_time",
        compute_priority_weighted_completion_time(tasks_copy, allocator.task_finish_times),
    )
    result.setdefault(
        "high_priority_top5_rate",
        compute_high_priority_top5_rate(tasks_copy, allocator.task_sequence),
    )
    result["algorithm_runtime_ms"] = algorithm_runtime_ms
    return result


def save_results(rows, path):
    fieldnames = [
        "map_width",
        "map_height",
        "obstacle_ratio",
        "task_num",
        "seed",
        "method",
        "completed_task_num",
        "total_path_length",
        "total_inspection_time",
        "high_priority_avg_response_time",
        "priority_weighted_completion_time",
        "high_priority_top5_rate",
        "algorithm_runtime_ms",
        "task_sequence",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def grouped_summary(rows):
    groups = {}
    for row in rows:
        key = (
            row["map_width"],
            row["map_height"],
            row["obstacle_ratio"],
            row["task_num"],
            row["method"],
        )
        groups.setdefault(key, []).append(row)

    summary_rows = []
    for key in sorted(groups, key=lambda item: (item[0], item[1], item[3], item[2], item[4])):
        map_width, map_height, obstacle_ratio, task_num, method = key
        items = groups[key]
        row = {
            "map_width": map_width,
            "map_height": map_height,
            "obstacle_ratio": obstacle_ratio,
            "task_num": task_num,
            "method": method,
        }
        for metric in METRICS:
            values = [float(item[metric]) for item in items]
            row[f"{metric}_mean"] = statistics.mean(values) if values else 0.0
            row[f"{metric}_std"] = statistics.stdev(values) if len(values) > 1 else 0.0
        summary_rows.append(row)
    return summary_rows


def save_summary(summary_rows, path):
    fieldnames = [
        "map_width",
        "map_height",
        "obstacle_ratio",
        "task_num",
        "method",
    ]
    for metric in METRICS:
        fieldnames.extend([f"{metric}_mean", f"{metric}_std"])

    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)


def summarize_by_method(rows):
    groups = {}
    for row in rows:
        groups.setdefault(row["method"], []).append(row)

    summary = []
    for method in METHODS:
        items = groups.get(method, [])
        row = {"method": method}
        for metric in METRICS:
            values = [float(item[metric]) for item in items]
            row[f"{metric}_mean"] = statistics.mean(values) if values else 0.0
        summary.append(row)
    return summary


def percent_change(new_value, base_value):
    if base_value == 0:
        return 0.0
    return (new_value - base_value) / base_value * 100.0


def print_overall_summary(overall_summary):
    print(
        "\nMethod | Path Mean | Time Mean | High Priority Response Mean | "
        "Priority Weighted Completion Mean | Top5 High Priority Rate Mean | Runtime Mean(ms)"
    )
    for row in overall_summary:
        print(
            f"{row['method']} | {row['total_path_length_mean']:.2f} | "
            f"{row['total_inspection_time_mean']:.2f} | "
            f"{row['high_priority_avg_response_time_mean']:.2f} | "
            f"{row['priority_weighted_completion_time_mean']:.2f} | "
            f"{row['high_priority_top5_rate_mean']:.2f} | "
            f"{row['algorithm_runtime_ms_mean']:.2f}"
        )


def print_change_block(summary_map, base_method):
    rh = summary_map["RH-Proposed-v2"]
    base = summary_map[base_method]
    print(f"\nRH-Proposed-v2 vs {base_method} overall mean change:")
    for metric in [
        "total_path_length",
        "total_inspection_time",
        "high_priority_avg_response_time",
        "priority_weighted_completion_time",
        "high_priority_top5_rate",
        "algorithm_runtime_ms",
    ]:
        change = percent_change(rh[f"{metric}_mean"], base[f"{metric}_mean"])
        print(f"- {metric}: {change:.2f}%")


def main():
    map_sizes = [(30, 30), (50, 50)]
    task_nums = [10, 20, 40]
    obstacle_ratios = [0.1, 0.2, 0.3]
    seeds = list(range(20))
    start_pos = (2, 2)
    robot_speed = 0.6
    inspection_time = 5.0

    results_dir = Path(__file__).resolve().parent / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    results_path = results_dir / "rh_v2_scale_results.csv"
    summary_path = results_dir / "rh_v2_scale_summary.csv"

    rows = []
    scenario_index = 0
    total_scenarios = len(map_sizes) * len(task_nums) * len(obstacle_ratios)
    for width, height in map_sizes:
        for task_num in task_nums:
            for obstacle_ratio in obstacle_ratios:
                scenario_index += 1
                print(
                    f"Running scenario {scenario_index}/{total_scenarios}: "
                    f"{width}x{height}, tasks={task_num}, obstacle_ratio={obstacle_ratio}"
                )
                for seed in seeds:
                    grid_map = create_grid_map(width, height, obstacle_ratio, start_pos, seed)
                    tasks = create_tasks(grid_map, start_pos, task_num, seed)
                    for method in METHODS:
                        result = run_method(
                            method,
                            grid_map,
                            tasks,
                            start_pos,
                            robot_speed,
                            inspection_time,
                        )
                        rows.append(
                            {
                                "map_width": width,
                                "map_height": height,
                                "obstacle_ratio": obstacle_ratio,
                                "task_num": task_num,
                                "seed": seed,
                                "method": method,
                                "completed_task_num": result["completed_task_num"],
                                "total_path_length": result["total_path_length"],
                                "total_inspection_time": result["total_inspection_time"],
                                "high_priority_avg_response_time": result["high_priority_avg_response_time"],
                                "priority_weighted_completion_time": result["priority_weighted_completion_time"],
                                "high_priority_top5_rate": result["high_priority_top5_rate"],
                                "algorithm_runtime_ms": result["algorithm_runtime_ms"],
                                "task_sequence": "->".join(result["task_sequence"]),
                            }
                        )

    save_results(rows, results_path)
    summary_rows = grouped_summary(rows)
    save_summary(summary_rows, summary_path)

    print(f"\nTotal experiment rows: {len(rows)}")
    counts = {method: 0 for method in METHODS}
    scenario_counts = {}
    for row in rows:
        counts[row["method"]] += 1
        scenario_key = (
            row["map_width"],
            row["map_height"],
            row["obstacle_ratio"],
            row["task_num"],
        )
        scenario_counts[scenario_key] = scenario_counts.get(scenario_key, 0) + 1

    for method in METHODS:
        print(f"{method}: {counts[method]} rows")

    overall_summary = summarize_by_method(rows)
    print_overall_summary(overall_summary)
    summary_map = {row["method"]: row for row in overall_summary}
    print_change_block(summary_map, "AStarOnly")
    print_change_block(summary_map, "Proposed-Balanced")

    bad_scenarios = [key for key, count in scenario_counts.items() if count != 80]
    print(f"\nScenario count check: {len(scenario_counts)} scenarios, bad scenarios={len(bad_scenarios)}")
    print("说明：路径、时间、响应时间、加权完成时间、运行时间越小越好；top5 rate 越大越好。")
    print("变化率为 (RH-Proposed-v2 - 对比方法) / 对比方法 * 100%。")
    print(f"\nResults saved to: {results_path}")
    print(f"Summary saved to: {summary_path}")


if __name__ == "__main__":
    main()
