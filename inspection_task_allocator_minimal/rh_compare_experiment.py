import copy
import csv
import random
import statistics
from pathlib import Path

from baseline_methods import AStarOnlyAllocator, NearestNeighborAllocator
from receding_horizon_allocator import RHProposedAllocator
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
    return RHProposedAllocator(
        grid_map=grid_map,
        tasks=tasks,
        start_pos=start_pos,
        robot_speed=robot_speed,
        inspection_time=inspection_time,
        **BALANCED_WEIGHTS,
        horizon=3,
        beam_width=5,
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
    result = allocator.run()
    result.setdefault(
        "priority_weighted_completion_time",
        compute_priority_weighted_completion_time(tasks_copy, allocator.task_finish_times),
    )
    result.setdefault(
        "high_priority_top5_rate",
        compute_high_priority_top5_rate(tasks_copy, allocator.task_sequence),
    )
    return result


def save_results(rows, path):
    fieldnames = [
        "seed",
        "method",
        "completed_task_num",
        "total_path_length",
        "total_inspection_time",
        "high_priority_avg_response_time",
        "priority_weighted_completion_time",
        "high_priority_top5_rate",
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
    for method in ["NNF", "AStarOnly", "Proposed-Balanced", "RH-Proposed"]:
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
                "priority_weighted_completion_time_mean": statistics.mean(vals("priority_weighted_completion_time")) if items else 0.0,
                "priority_weighted_completion_time_std": statistics.stdev(vals("priority_weighted_completion_time")) if len(items) > 1 else 0.0,
                "high_priority_top5_rate_mean": statistics.mean(vals("high_priority_top5_rate")) if items else 0.0,
                "high_priority_top5_rate_std": statistics.stdev(vals("high_priority_top5_rate")) if len(items) > 1 else 0.0,
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
        "priority_weighted_completion_time_mean",
        "priority_weighted_completion_time_std",
        "high_priority_top5_rate_mean",
        "high_priority_top5_rate_std",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary)


def percent_change(new_value, base_value):
    if base_value == 0:
        return 0.0
    return (new_value - base_value) / base_value * 100.0


def print_change_block(summary_map, base_method):
    rh = summary_map["RH-Proposed"]
    base = summary_map[base_method]
    print(f"\nRH-Proposed vs {base_method} mean change:")
    print(
        f"- total_path_length: {percent_change(rh['total_path_length_mean'], base['total_path_length_mean']):.2f}%"
    )
    print(
        f"- total_inspection_time: {percent_change(rh['total_inspection_time_mean'], base['total_inspection_time_mean']):.2f}%"
    )
    print(
        f"- high_priority_avg_response_time: {percent_change(rh['high_priority_avg_response_time_mean'], base['high_priority_avg_response_time_mean']):.2f}%"
    )
    print(
        f"- priority_weighted_completion_time: {percent_change(rh['priority_weighted_completion_time_mean'], base['priority_weighted_completion_time_mean']):.2f}%"
    )
    print(
        f"- high_priority_top5_rate: {percent_change(rh['high_priority_top5_rate_mean'], base['high_priority_top5_rate_mean']):.2f}%"
    )


def print_summary_table(summary):
    print(
        "\nMethod | Path Mean | Time Mean | High Priority Response Mean | "
        "Priority Weighted Completion Mean | Top5 High Priority Rate Mean"
    )
    for row in summary:
        print(
            f"{row['method']} | {row['total_path_length_mean']:.2f} | "
            f"{row['total_inspection_time_mean']:.2f} | "
            f"{row['high_priority_avg_response_time_mean']:.2f} | "
            f"{row['priority_weighted_completion_time_mean']:.2f} | "
            f"{row['high_priority_top5_rate_mean']:.2f}"
        )


def main():
    width = 30
    height = 30
    obstacle_ratio = 0.2
    task_num = 20
    start_pos = (2, 2)
    robot_speed = 0.6
    inspection_time = 5.0
    seeds = list(range(30))
    methods = ["NNF", "AStarOnly", "Proposed-Balanced", "RH-Proposed"]

    results_dir = Path(__file__).resolve().parent / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    results_path = results_dir / "rh_compare_results.csv"
    summary_path = results_dir / "rh_compare_summary.csv"

    rows = []
    for seed in seeds:
        grid_map = create_grid_map(width, height, obstacle_ratio, start_pos, seed)
        tasks = create_tasks(grid_map, start_pos, task_num, seed)
        for method in methods:
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
                    "seed": seed,
                    "method": method,
                    "completed_task_num": result["completed_task_num"],
                    "total_path_length": result["total_path_length"],
                    "total_inspection_time": result["total_inspection_time"],
                    "high_priority_avg_response_time": result["high_priority_avg_response_time"],
                    "priority_weighted_completion_time": result["priority_weighted_completion_time"],
                    "high_priority_top5_rate": result["high_priority_top5_rate"],
                    "task_sequence": "->".join(result["task_sequence"]),
                }
            )

    save_results(rows, results_path)
    summary = summarize(rows)
    save_summary(summary, summary_path)

    print("Experiment settings:")
    print(f"- map size: {width}x{height}")
    print(f"- obstacle ratio: {obstacle_ratio}")
    print(f"- task num: {task_num}")
    print(f"- seed count: {len(seeds)}")
    print(f"- methods: {', '.join(methods)}")
    print(f"\nTotal experiment rows: {len(rows)}")

    counts = {method: 0 for method in methods}
    for row in rows:
        counts[row["method"]] += 1
    for method in methods:
        print(f"{method}: {counts[method]} rows")

    print_summary_table(summary)
    summary_map = {row["method"]: row for row in summary}
    print_change_block(summary_map, "Proposed-Balanced")
    print_change_block(summary_map, "AStarOnly")

    print(
        "\n说明：正数表示 RH-Proposed 数值更大；负数表示 RH-Proposed 数值更小。"
    )
    print("路径、时间、响应时间、加权完成时间越小越好；top5 rate 越大越好。")
    print(f"\nResults saved to: {results_path}")
    print(f"Summary saved to: {summary_path}")


if __name__ == "__main__":
    main()
