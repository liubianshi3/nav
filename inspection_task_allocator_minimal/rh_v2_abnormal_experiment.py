import copy
import csv
import math
import random
import statistics
import time
from pathlib import Path

from baseline_methods import AStarOnlyAllocator
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
    "AStarOnly",
    "Proposed-Balanced",
    "RH-v2-Full",
    "RH-v2-Light",
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


def compute_abnormal_metrics(
    task_sequence,
    abnormal_task_ids,
    abnormal_trigger_time,
    task_finish_times,
    total_inspection_time,
):
    if not abnormal_task_ids:
        return 0.0, 0.0

    post_sequence = task_sequence[3:]
    first_k_after_trigger = post_sequence[: len(abnormal_task_ids)]
    abnormal_priority_rate = (
        len(set(first_k_after_trigger) & set(abnormal_task_ids))
        / len(abnormal_task_ids)
        * 100.0
    )

    response_times = []
    for task_id in abnormal_task_ids:
        if task_id in task_finish_times:
            response_times.append(task_finish_times[task_id] - abnormal_trigger_time)
        else:
            response_times.append(total_inspection_time - abnormal_trigger_time)
    abnormal_avg_response_time = sum(response_times) / len(response_times)
    return abnormal_priority_rate, abnormal_avg_response_time


def sample_abnormal_tasks(allocator, seed):
    remaining = [task for task in allocator.tasks if task.status == 0]
    sample_n = min(4, len(remaining))
    rng = random.Random(seed + 5000)
    return rng.sample(remaining, sample_n) if sample_n > 0 else []


def update_abnormal_weights(allocator, abnormal_tasks, rho=0.5, sigma=5.0):
    abnormal_task_ids = {task.task_id for task in abnormal_tasks}
    for task in allocator.tasks:
        if task.status == 1:
            continue
        if task.task_id in abnormal_task_ids:
            task.abnormal_weight = 1.0
            continue
        if not abnormal_tasks:
            continue
        distance_to_abnormal = min(
            abs(task.x - abnormal_task.x) + abs(task.y - abnormal_task.y)
            for abnormal_task in abnormal_tasks
        )
        new_weight = max(
            task.abnormal_weight,
            rho * math.exp(-distance_to_abnormal / sigma),
        )
        task.abnormal_weight = min(1.0, new_weight)


def build_result(allocator, abnormal_task_ids, abnormal_trigger_time, replanning_count, runtime_ms):
    abnormal_priority_rate, abnormal_avg_response_time = compute_abnormal_metrics(
        allocator.task_sequence,
        abnormal_task_ids,
        abnormal_trigger_time,
        allocator.task_finish_times,
        allocator.total_inspection_time,
    )
    return {
        "task_sequence": allocator.task_sequence,
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
        "completed_task_num": len(allocator.task_sequence),
        "abnormal_task_ids": ",".join(abnormal_task_ids),
        "abnormal_priority_rate": abnormal_priority_rate,
        "abnormal_avg_response_time": abnormal_avg_response_time,
        "replanning_count": replanning_count,
        "algorithm_runtime_ms": runtime_ms,
        "selection_records": allocator.selection_records,
    }


def run_astar_only_with_abnormal_tracking(grid_map, tasks, start_pos, seed, robot_speed, inspection_time):
    tasks_copy = copy.deepcopy(tasks)
    allocator = AStarOnlyAllocator(
        grid_map=grid_map,
        tasks=tasks_copy,
        start_pos=start_pos,
        robot_speed=robot_speed,
        inspection_time=inspection_time,
    )

    abnormal_task_ids = []
    abnormal_trigger_time = 0.0
    trigger_done = False

    start_time = time.perf_counter()
    while True:
        unfinished = allocator.get_unfinished_tasks()
        if not unfinished:
            break

        candidates = []
        for task in unfinished:
            path_info = allocator.planner.plan(allocator.current_pos, task.position)
            if not path_info["reachable"]:
                continue
            cost = (
                path_info["path_length"]
                + 0.5 * path_info["turn_count"]
                + 0.2 * path_info["obstacle_nearby_count"]
            )
            candidates.append((cost, task.task_id, task, path_info))

        if not candidates:
            print("No reachable unfinished tasks. Stop AStarOnly.")
            break

        candidates.sort(key=lambda item: (item[0], item[1]))
        cost, _, selected_task, path_info = candidates[0]
        record = {
            "task_id": selected_task.task_id,
            "path_length": path_info["path_length"],
            "astar_only_cost": cost,
            "priority": selected_task.priority,
            "risk": selected_task.risk,
            "abnormal_weight": selected_task.abnormal_weight,
            "turn_count": path_info["turn_count"],
            "obstacle_nearby_count": path_info["obstacle_nearby_count"],
            "method_detail": "astar_only_cost",
        }
        execute_selected_task(allocator, selected_task, path_info, record)

        if len(allocator.task_sequence) == 3 and not trigger_done:
            trigger_done = True
            abnormal_trigger_time = allocator.total_inspection_time
            abnormal_task_ids = [
                task.task_id for task in sample_abnormal_tasks(allocator, seed)
            ]

    end_time = time.perf_counter()
    return build_result(
        allocator,
        abnormal_task_ids,
        abnormal_trigger_time,
        replanning_count=0,
        runtime_ms=(end_time - start_time) * 1000.0,
    )


def run_proposed_balanced_with_abnormal(grid_map, tasks, start_pos, seed, robot_speed, inspection_time):
    tasks_copy = copy.deepcopy(tasks)
    allocator = PriorityCostTaskAllocator(
        grid_map=grid_map,
        tasks=tasks_copy,
        start_pos=start_pos,
        robot_speed=robot_speed,
        inspection_time=inspection_time,
        **BALANCED_WEIGHTS,
    )

    abnormal_task_ids = []
    abnormal_trigger_time = 0.0
    replanning_count = 0

    start_time = time.perf_counter()
    while True:
        if not allocator.get_unfinished_tasks():
            break

        selected_task, path_info, record = allocator.select_next_task(allocator.current_pos)
        if selected_task is None:
            print("No reachable unfinished tasks. Stop Proposed-Balanced.")
            break

        execute_selected_task(allocator, selected_task, path_info, record)

        if len(allocator.task_sequence) == 3 and replanning_count == 0:
            abnormal_trigger_time = allocator.total_inspection_time
            abnormal_tasks = sample_abnormal_tasks(allocator, seed)
            abnormal_task_ids = [task.task_id for task in abnormal_tasks]
            update_abnormal_weights(allocator, abnormal_tasks)
            replanning_count = 1

    end_time = time.perf_counter()
    return build_result(
        allocator,
        abnormal_task_ids,
        abnormal_trigger_time,
        replanning_count=replanning_count,
        runtime_ms=(end_time - start_time) * 1000.0,
    )


def run_rh_v2_with_abnormal(
    grid_map,
    tasks,
    start_pos,
    seed,
    robot_speed,
    inspection_time,
    horizon,
    beam_width,
    candidate_pool_size,
):
    tasks_copy = copy.deepcopy(tasks)
    allocator = RHProposedAllocatorV2(
        grid_map=grid_map,
        tasks=tasks_copy,
        start_pos=start_pos,
        robot_speed=robot_speed,
        inspection_time=inspection_time,
        **BALANCED_WEIGHTS,
        horizon=horizon,
        beam_width=beam_width,
        candidate_pool_size=candidate_pool_size,
    )

    abnormal_task_ids = []
    abnormal_trigger_time = 0.0
    replanning_count = 0

    start_time = time.perf_counter()
    while True:
        if not allocator.get_unfinished_tasks():
            break

        selected_task, path_info, record = allocator.select_next_task(allocator.current_pos)
        if selected_task is None:
            print("No reachable unfinished tasks. Stop RH-v2.")
            break

        execute_selected_task(allocator, selected_task, path_info, record)

        if len(allocator.task_sequence) == 3 and replanning_count == 0:
            abnormal_trigger_time = allocator.total_inspection_time
            abnormal_tasks = sample_abnormal_tasks(allocator, seed)
            abnormal_task_ids = [task.task_id for task in abnormal_tasks]
            update_abnormal_weights(allocator, abnormal_tasks)
            replanning_count = 1

    end_time = time.perf_counter()
    return build_result(
        allocator,
        abnormal_task_ids,
        abnormal_trigger_time,
        replanning_count=replanning_count,
        runtime_ms=(end_time - start_time) * 1000.0,
    )


def run_method(method, grid_map, tasks, start_pos, seed, robot_speed, inspection_time):
    if method == "AStarOnly":
        return run_astar_only_with_abnormal_tracking(
            grid_map,
            tasks,
            start_pos,
            seed,
            robot_speed,
            inspection_time,
        )
    if method == "Proposed-Balanced":
        return run_proposed_balanced_with_abnormal(
            grid_map,
            tasks,
            start_pos,
            seed,
            robot_speed,
            inspection_time,
        )
    if method == "RH-v2-Full":
        return run_rh_v2_with_abnormal(
            grid_map,
            tasks,
            start_pos,
            seed,
            robot_speed,
            inspection_time,
            horizon=4,
            beam_width=8,
            candidate_pool_size=10,
        )
    return run_rh_v2_with_abnormal(
        grid_map,
        tasks,
        start_pos,
        seed,
        robot_speed,
        inspection_time,
        horizon=3,
        beam_width=5,
        candidate_pool_size=6,
    )


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
        "abnormal_task_ids",
        "abnormal_priority_rate",
        "abnormal_avg_response_time",
        "replanning_count",
        "algorithm_runtime_ms",
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

    summary_rows = []
    for method in METHODS:
        items = groups.get(method, [])
        summary = {"method": method}
        for metric in METRICS:
            values = [float(item[metric]) for item in items]
            summary[f"{metric}_mean"] = statistics.mean(values) if values else 0.0
            summary[f"{metric}_std"] = statistics.stdev(values) if len(values) > 1 else 0.0
        summary_rows.append(summary)
    return summary_rows


def save_summary(summary_rows, path):
    fieldnames = ["method"]
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


def print_summary_table(summary_rows):
    print(
        "\nMethod | Path Mean | Time Mean | High Priority Response Mean | "
        "Priority Weighted Completion Mean | Top5 High Priority Rate Mean | "
        "Abnormal Priority Rate Mean | Abnormal Response Mean | Runtime Mean(ms)"
    )
    for row in summary_rows:
        print(
            f"{row['method']} | {row['total_path_length_mean']:.2f} | "
            f"{row['total_inspection_time_mean']:.2f} | "
            f"{row['high_priority_avg_response_time_mean']:.2f} | "
            f"{row['priority_weighted_completion_time_mean']:.2f} | "
            f"{row['high_priority_top5_rate_mean']:.2f} | "
            f"{row['abnormal_priority_rate_mean']:.2f} | "
            f"{row['abnormal_avg_response_time_mean']:.2f} | "
            f"{row['algorithm_runtime_ms_mean']:.2f}"
        )


def print_change_block(summary_map, target_method, base_method):
    target = summary_map[target_method]
    base = summary_map[base_method]
    print(f"\n{target_method} vs {base_method} mean change:")
    for metric in [
        "total_path_length",
        "total_inspection_time",
        "high_priority_avg_response_time",
        "priority_weighted_completion_time",
        "high_priority_top5_rate",
        "abnormal_priority_rate",
        "abnormal_avg_response_time",
        "algorithm_runtime_ms",
    ]:
        change = percent_change(target[f"{metric}_mean"], base[f"{metric}_mean"])
        print(f"- {metric}: {change:.2f}%")


def main():
    width = 30
    height = 30
    obstacle_ratio = 0.2
    task_num = 20
    start_pos = (2, 2)
    robot_speed = 0.6
    inspection_time = 5.0
    seeds = list(range(30))

    results_dir = Path(__file__).resolve().parent / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    results_path = results_dir / "rh_v2_abnormal_results.csv"
    summary_path = results_dir / "rh_v2_abnormal_summary.csv"

    rows = []
    for seed in seeds:
        grid_map = create_grid_map(width, height, obstacle_ratio, start_pos, seed)
        tasks = create_tasks(grid_map, start_pos, task_num, seed)
        for method in METHODS:
            result = run_method(
                method,
                grid_map,
                tasks,
                start_pos,
                seed,
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
                    "abnormal_task_ids": result["abnormal_task_ids"],
                    "abnormal_priority_rate": result["abnormal_priority_rate"],
                    "abnormal_avg_response_time": result["abnormal_avg_response_time"],
                    "replanning_count": result["replanning_count"],
                    "algorithm_runtime_ms": result["algorithm_runtime_ms"],
                    "task_sequence": "->".join(result["task_sequence"]),
                }
            )

    save_results(rows, results_path)
    summary_rows = summarize(rows)
    save_summary(summary_rows, summary_path)

    print(f"Total experiment rows: {len(rows)}")
    counts = {method: 0 for method in METHODS}
    for row in rows:
        counts[row["method"]] += 1
    for method in METHODS:
        print(f"{method}: {counts[method]} rows")

    print_summary_table(summary_rows)
    summary_map = {row["method"]: row for row in summary_rows}
    print_change_block(summary_map, "RH-v2-Full", "Proposed-Balanced")
    print_change_block(summary_map, "RH-v2-Light", "Proposed-Balanced")
    print_change_block(summary_map, "RH-v2-Full", "AStarOnly")
    print_change_block(summary_map, "RH-v2-Light", "AStarOnly")

    print(
        "\n说明：路径、时间、响应时间、加权完成时间、异常响应时间、运行时间越小越好。"
    )
    print("Top5 高优先级比例和异常任务优先处理率越大越好。")
    print("变化率为 (目标方法 - 对比方法) / 对比方法 * 100%。")
    print(f"\nResults saved to: {results_path}")
    print(f"Summary saved to: {summary_path}")


if __name__ == "__main__":
    main()
