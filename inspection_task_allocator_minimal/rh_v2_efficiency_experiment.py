import copy
import csv
import random
import statistics
import time
from pathlib import Path

from astar_planner import AStarPlanner
from receding_horizon_allocator_v2 import RHProposedAllocatorV2
from task_model import InspectionTask


BALANCED_WEIGHTS = {
    "alpha": 0.22,
    "beta": 0.18,
    "lambda_abnormal": 0.15,
    "gamma": 0.27,
    "delta": 0.12,
    "eta": 0.06,
}

PARAMETER_GROUPS = [
    {
        "method": "RH-v2-Full",
        "horizon": 4,
        "beam_width": 8,
        "candidate_pool_size": 10,
    },
    {
        "method": "RH-v2-Medium",
        "horizon": 3,
        "beam_width": 6,
        "candidate_pool_size": 8,
    },
    {
        "method": "RH-v2-Light",
        "horizon": 3,
        "beam_width": 5,
        "candidate_pool_size": 6,
    },
    {
        "method": "RH-v2-Fast",
        "horizon": 2,
        "beam_width": 5,
        "candidate_pool_size": 6,
    },
]

METRICS = [
    "reachable_task_count",
    "completed_task_num",
    "total_path_length",
    "total_inspection_time",
    "high_priority_avg_response_time",
    "priority_weighted_completion_time",
    "high_priority_top5_rate",
    "algorithm_runtime_ms",
]

RESULT_FIELDNAMES = [
    "map_width",
    "map_height",
    "obstacle_ratio",
    "task_num",
    "seed",
    "method",
    "horizon",
    "beam_width",
    "candidate_pool_size",
    "reachable_task_count",
    "completed_task_num",
    "total_path_length",
    "total_inspection_time",
    "high_priority_avg_response_time",
    "priority_weighted_completion_time",
    "high_priority_top5_rate",
    "algorithm_runtime_ms",
    "task_sequence",
]


class CachedAStarPlanner(AStarPlanner):
    def __init__(self, grid_map):
        super().__init__(grid_map)
        self._cache = {}

    def plan(self, start, goal):
        key = (start, goal)
        if key not in self._cache:
            self._cache[key] = super().plan(start, goal)
        return self._cache[key]


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


def compute_reachable_task_count(grid_map, start_pos, tasks):
    planner = AStarPlanner(grid_map)
    reachable_count = 0
    for task in tasks:
        path_info = planner.plan(start_pos, task.position)
        if path_info.get("reachable", False):
            reachable_count += 1
    return reachable_count


def run_parameter_group(group, grid_map, tasks, start_pos, robot_speed, inspection_time):
    tasks_copy = copy.deepcopy(tasks)
    allocator = RHProposedAllocatorV2(
        grid_map=grid_map,
        tasks=tasks_copy,
        start_pos=start_pos,
        robot_speed=robot_speed,
        inspection_time=inspection_time,
        **BALANCED_WEIGHTS,
        horizon=group["horizon"],
        beam_width=group["beam_width"],
        candidate_pool_size=group["candidate_pool_size"],
    )
    allocator.planner = CachedAStarPlanner(grid_map)

    start_time = time.perf_counter()
    result = allocator.run()
    end_time = time.perf_counter()
    result["algorithm_runtime_ms"] = (end_time - start_time) * 1000.0
    return result


def save_results(rows, path):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=RESULT_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def summarize(rows):
    groups = {}
    for row in rows:
        key = (
            row["method"],
            row["horizon"],
            row["beam_width"],
            row["candidate_pool_size"],
        )
        groups.setdefault(key, []).append(row)

    summary_rows = []
    method_order = {group["method"]: index for index, group in enumerate(PARAMETER_GROUPS)}
    for key in sorted(groups, key=lambda item: method_order.get(item[0], 999)):
        method, horizon, beam_width, candidate_pool_size = key
        items = groups[key]
        row = {
            "method": method,
            "horizon": horizon,
            "beam_width": beam_width,
            "candidate_pool_size": candidate_pool_size,
        }
        for metric in METRICS:
            values = [float(item[metric]) for item in items]
            row[f"{metric}_mean"] = statistics.mean(values) if values else 0.0
            row[f"{metric}_std"] = statistics.stdev(values) if len(values) > 1 else 0.0
        summary_rows.append(row)
    return summary_rows


def save_summary(summary_rows, path):
    fieldnames = [
        "method",
        "horizon",
        "beam_width",
        "candidate_pool_size",
    ]
    for metric in METRICS:
        fieldnames.extend([f"{metric}_mean", f"{metric}_std"])

    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)


def percent_change(new_value, base_value):
    if base_value == 0:
        return 0.0
    return (new_value - base_value) / base_value * 100.0


def print_overall_summary(summary_rows):
    print(
        "\nMethod | Path Mean | Time Mean | High Priority Response Mean | "
        "Priority Weighted Completion Mean | Top5 High Priority Rate Mean | Runtime Mean(ms)"
    )
    for row in summary_rows:
        print(
            f"{row['method']} | {row['total_path_length_mean']:.2f} | "
            f"{row['total_inspection_time_mean']:.2f} | "
            f"{row['high_priority_avg_response_time_mean']:.2f} | "
            f"{row['priority_weighted_completion_time_mean']:.2f} | "
            f"{row['high_priority_top5_rate_mean']:.2f} | "
            f"{row['algorithm_runtime_ms_mean']:.2f}"
        )


def print_change_blocks(summary_rows):
    summary_map = {row["method"]: row for row in summary_rows}
    full = summary_map["RH-v2-Full"]
    for method in ["RH-v2-Medium", "RH-v2-Light", "RH-v2-Fast"]:
        row = summary_map[method]
        print(f"\n{method} vs RH-v2-Full mean change:")
        for metric in [
            "total_path_length",
            "total_inspection_time",
            "high_priority_avg_response_time",
            "priority_weighted_completion_time",
            "high_priority_top5_rate",
            "algorithm_runtime_ms",
        ]:
            change = percent_change(row[f"{metric}_mean"], full[f"{metric}_mean"])
            print(f"- {metric}: {change:.2f}%")


def main():
    map_sizes = [(30, 30), (50, 50)]
    task_nums = [20, 40]
    obstacle_ratios = [0.1, 0.2, 0.3]
    seeds = list(range(20))
    start_pos = (2, 2)
    robot_speed = 0.6
    inspection_time = 5.0

    results_dir = Path(__file__).resolve().parent / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    results_path = results_dir / "rh_v2_efficiency_results.csv"
    summary_path = results_dir / "rh_v2_efficiency_summary.csv"

    scenario_index = 0
    total_scenarios = len(map_sizes) * len(task_nums) * len(obstacle_ratios)
    rows = []
    with open(results_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=RESULT_FIELDNAMES)
        writer.writeheader()
        for width, height in map_sizes:
            for task_num in task_nums:
                for obstacle_ratio in obstacle_ratios:
                    scenario_index += 1
                    print(
                        f"Running scenario {scenario_index}/{total_scenarios}: "
                        f"{width}x{height}, tasks={task_num}, obstacle_ratio={obstacle_ratio}",
                        flush=True,
                    )
                    for seed in seeds:
                        grid_map = create_grid_map(width, height, obstacle_ratio, start_pos, seed)
                        tasks = create_tasks(grid_map, start_pos, task_num, seed)
                        reachable_task_count = compute_reachable_task_count(
                            grid_map,
                            start_pos,
                            tasks,
                        )
                        for group in PARAMETER_GROUPS:
                            result = run_parameter_group(
                                group,
                                grid_map,
                                tasks,
                                start_pos,
                                robot_speed,
                                inspection_time,
                            )
                            row = {
                                "map_width": width,
                                "map_height": height,
                                "obstacle_ratio": obstacle_ratio,
                                "task_num": task_num,
                                "seed": seed,
                                "method": group["method"],
                                "horizon": group["horizon"],
                                "beam_width": group["beam_width"],
                                "candidate_pool_size": group["candidate_pool_size"],
                                "reachable_task_count": reachable_task_count,
                                "completed_task_num": result["completed_task_num"],
                                "total_path_length": result["total_path_length"],
                                "total_inspection_time": result["total_inspection_time"],
                                "high_priority_avg_response_time": result["high_priority_avg_response_time"],
                                "priority_weighted_completion_time": result["priority_weighted_completion_time"],
                                "high_priority_top5_rate": result["high_priority_top5_rate"],
                                "algorithm_runtime_ms": result["algorithm_runtime_ms"],
                                "task_sequence": "->".join(result["task_sequence"]),
                            }
                            rows.append(row)
                            writer.writerow(row)
                        f.flush()

    summary_rows = summarize(rows)
    save_summary(summary_rows, summary_path)

    print(f"\nTotal experiment rows: {len(rows)}")
    counts = {group["method"]: 0 for group in PARAMETER_GROUPS}
    for row in rows:
        counts[row["method"]] += 1
    for group in PARAMETER_GROUPS:
        print(f"{group['method']}: {counts[group['method']]} rows")

    zero_reachable_rows = sum(1 for row in rows if int(row["reachable_task_count"]) == 0)
    print(f"reachable_task_count == 0 rows: {zero_reachable_rows}")

    print_overall_summary(summary_rows)
    print_change_blocks(summary_rows)
    print("说明：路径、时间、响应时间、加权完成时间、运行时间越小越好；top5 rate 越大越好。")
    print("变化率为 (参数组 - RH-v2-Full) / RH-v2-Full * 100%。")
    print(f"\nResults saved to: {results_path}")
    print(f"Summary saved to: {summary_path}")


if __name__ == "__main__":
    main()
