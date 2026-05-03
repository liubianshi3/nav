import copy
import csv
import math
import random
import statistics
from pathlib import Path

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


def build_allocator(grid_map, tasks, start_pos, weights):
    return PriorityCostTaskAllocator(
        grid_map=grid_map,
        tasks=tasks,
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
    return abnormal_priority_rate, sum(response_times) / len(response_times)


def compute_high_priority_avg_response_time(tasks, task_finish_times):
    selected = [t for t in tasks if t.priority >= 0.75 and t.task_id in task_finish_times]
    if not selected:
        return 0.0
    return sum(task_finish_times[t.task_id] for t in selected) / len(selected)


def compute_high_risk_avg_response_time(tasks, task_finish_times):
    selected = [t for t in tasks if t.risk >= 0.75 and t.task_id in task_finish_times]
    if not selected:
        return 0.0
    return sum(task_finish_times[t.task_id] for t in selected) / len(selected)


def compute_top5_high_priority_rate(task_sequence, tasks):
    if not task_sequence:
        return 0.0
    task_map = {t.task_id: t for t in tasks}
    top5 = task_sequence[:5]
    count = sum(1 for task_id in top5 if task_map[task_id].priority >= 0.75)
    return count / min(5, len(top5)) * 100.0


def compute_weighted_completion_time(tasks, task_finish_times, attr):
    weighted_sum = 0.0
    weight_sum = 0.0
    for task in tasks:
        if task.task_id not in task_finish_times:
            continue
        w = getattr(task, attr)
        weighted_sum += w * task_finish_times[task.task_id]
        weight_sum += w
    return weighted_sum / weight_sum if weight_sum > 0 else 0.0


def sample_abnormal_tasks(allocator, seed):
    remaining = [t for t in allocator.tasks if t.status == 0]
    sample_n = min(4, len(remaining))
    rng = random.Random(seed + 5000)
    abnormal_tasks = rng.sample(remaining, sample_n) if sample_n > 0 else []
    return abnormal_tasks


def update_abnormal_weights(allocator, abnormal_tasks, rho=0.5, sigma=5.0):
    for task in allocator.tasks:
        if task.status == 1:
            continue
        if task.task_id in {t.task_id for t in abnormal_tasks}:
            task.abnormal_weight = 1.0
        else:
            if not abnormal_tasks:
                continue
            distance_to_abnormal = min(
                abs(task.x - abnormal_task.x) + abs(task.y - abnormal_task.y)
                for abnormal_task in abnormal_tasks
            )
            new_weight = max(task.abnormal_weight, rho * math.exp(-distance_to_abnormal / sigma))
            task.abnormal_weight = min(1.0, new_weight)


def run_ablation_version(grid_map, tasks, start_pos, seed, method_name, weights):
    tasks_copy = copy.deepcopy(tasks)
    allocator = build_allocator(grid_map, tasks_copy, start_pos, weights)
    abnormal_trigger_time = 0.0
    abnormal_task_ids = []
    replanning_count = 0
    abnormal_triggered = False

    while True:
        unfinished = allocator.get_unfinished_tasks()
        if not unfinished:
            break
        selected_task, path_info, record = allocator.select_next_task(allocator.current_pos)
        if selected_task is None:
            print(f"No reachable unfinished tasks. Stop {method_name}.")
            break
        execute_selected_task(allocator, selected_task, path_info, record)

        if len(allocator.task_sequence) == 3 and not abnormal_triggered:
            abnormal_triggered = True
            abnormal_trigger_time = allocator.total_inspection_time
            abnormal_tasks = sample_abnormal_tasks(allocator, seed)
            abnormal_task_ids = [t.task_id for t in abnormal_tasks]
            update_abnormal_weights(allocator, abnormal_tasks)
            replanning_count = 1

    if not abnormal_triggered:
        abnormal_trigger_time = allocator.total_inspection_time
        abnormal_tasks = sample_abnormal_tasks(allocator, seed)
        abnormal_task_ids = [t.task_id for t in abnormal_tasks]

    task_map = {t.task_id: t for t in allocator.tasks}
    high_priority_top5_rate = compute_top5_high_priority_rate(allocator.task_sequence, allocator.tasks)
    priority_weighted_completion_time = compute_weighted_completion_time(allocator.tasks, allocator.task_finish_times, "priority")
    risk_weighted_completion_time = compute_weighted_completion_time(allocator.tasks, allocator.task_finish_times, "risk")
    abnormal_priority_rate, abnormal_avg_response_time = compute_abnormal_metrics(
        allocator.task_sequence,
        abnormal_task_ids,
        abnormal_trigger_time,
        allocator.task_finish_times,
        allocator.total_inspection_time,
    )

    return {
        "task_sequence": allocator.task_sequence,
        "completed_task_num": len(allocator.task_sequence),
        "total_path_length": allocator.total_path_length,
        "total_inspection_time": allocator.total_inspection_time,
        "high_priority_avg_response_time": compute_high_priority_avg_response_time(allocator.tasks, allocator.task_finish_times),
        "high_risk_avg_response_time": compute_high_risk_avg_response_time(allocator.tasks, allocator.task_finish_times),
        "abnormal_task_ids": ",".join(abnormal_task_ids),
        "abnormal_priority_rate": abnormal_priority_rate,
        "abnormal_avg_response_time": abnormal_avg_response_time,
        "high_priority_top5_rate": high_priority_top5_rate,
        "priority_weighted_completion_time": priority_weighted_completion_time,
        "risk_weighted_completion_time": risk_weighted_completion_time,
        "replanning_count": replanning_count,
        "selection_records": allocator.selection_records,
        "task_finish_times": allocator.task_finish_times,
    }


def summarize(rows):
    groups = {}
    for row in rows:
        groups.setdefault(row["method"], []).append(row)

    summary_rows = []
    methods = [
        "Full-Proposed-Balanced",
        "w/o Priority",
        "w/o Risk",
        "w/o Abnormal",
        "w/o Complexity",
        "w/o Energy",
    ]
    for method in methods:
        items = groups.get(method, [])
        def vals(key):
            return [item[key] for item in items]
        summary_rows.append(
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
                "high_risk_avg_response_time_mean": statistics.mean(vals("high_risk_avg_response_time")) if items else 0.0,
                "high_risk_avg_response_time_std": statistics.stdev(vals("high_risk_avg_response_time")) if len(items) > 1 else 0.0,
                "abnormal_priority_rate_mean": statistics.mean(vals("abnormal_priority_rate")) if items else 0.0,
                "abnormal_priority_rate_std": statistics.stdev(vals("abnormal_priority_rate")) if len(items) > 1 else 0.0,
                "abnormal_avg_response_time_mean": statistics.mean(vals("abnormal_avg_response_time")) if items else 0.0,
                "abnormal_avg_response_time_std": statistics.stdev(vals("abnormal_avg_response_time")) if len(items) > 1 else 0.0,
                "high_priority_top5_rate_mean": statistics.mean(vals("high_priority_top5_rate")) if items else 0.0,
                "high_priority_top5_rate_std": statistics.stdev(vals("high_priority_top5_rate")) if len(items) > 1 else 0.0,
                "priority_weighted_completion_time_mean": statistics.mean(vals("priority_weighted_completion_time")) if items else 0.0,
                "priority_weighted_completion_time_std": statistics.stdev(vals("priority_weighted_completion_time")) if len(items) > 1 else 0.0,
                "risk_weighted_completion_time_mean": statistics.mean(vals("risk_weighted_completion_time")) if items else 0.0,
                "risk_weighted_completion_time_std": statistics.stdev(vals("risk_weighted_completion_time")) if len(items) > 1 else 0.0,
                "replanning_count_mean": statistics.mean(vals("replanning_count")) if items else 0.0,
                "replanning_count_std": statistics.stdev(vals("replanning_count")) if len(items) > 1 else 0.0,
            }
        )
    return summary_rows


def save_csv(rows, path):
    fieldnames = [
        "seed",
        "method",
        "completed_task_num",
        "total_path_length",
        "total_inspection_time",
        "high_priority_avg_response_time",
        "high_risk_avg_response_time",
        "abnormal_task_ids",
        "abnormal_priority_rate",
        "abnormal_avg_response_time",
        "high_priority_top5_rate",
        "priority_weighted_completion_time",
        "risk_weighted_completion_time",
        "replanning_count",
        "task_sequence",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def save_summary(rows, path):
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
        "high_risk_avg_response_time_mean",
        "high_risk_avg_response_time_std",
        "abnormal_priority_rate_mean",
        "abnormal_priority_rate_std",
        "abnormal_avg_response_time_mean",
        "abnormal_avg_response_time_std",
        "high_priority_top5_rate_mean",
        "high_priority_top5_rate_std",
        "priority_weighted_completion_time_mean",
        "priority_weighted_completion_time_std",
        "risk_weighted_completion_time_mean",
        "risk_weighted_completion_time_std",
        "replanning_count_mean",
        "replanning_count_std",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def print_summary(summary_rows):
    print("\nMethod | Path Mean | Time Mean | High Priority Response Mean | High Risk Response Mean | Abnormal Priority Rate Mean | Abnormal Response Mean | Top5 High Priority Rate Mean | Priority Weighted Completion Mean")
    for row in summary_rows:
        print(
            f"{row['method']} | {row['total_path_length_mean']:.2f} | {row['total_inspection_time_mean']:.2f} | {row['high_priority_avg_response_time_mean']:.2f} | {row['high_risk_avg_response_time_mean']:.2f} | {row['abnormal_priority_rate_mean']:.2f} | {row['abnormal_avg_response_time_mean']:.2f} | {row['high_priority_top5_rate_mean']:.2f} | {row['priority_weighted_completion_time_mean']:.2f}"
        )


def change(full, ablation):
    if ablation == 0:
        return 0.0
    return (full - ablation) / ablation * 100.0


def print_differences(summary_map):
    full = summary_map["Full-Proposed-Balanced"]
    print("\nFull-Proposed-Balanced vs w/o Priority:")
    print(f"- high_priority_avg_response_time_change: {change(full['high_priority_avg_response_time_mean'], summary_map['w/o Priority']['high_priority_avg_response_time_mean']):.2f}%")
    print(f"- high_priority_top5_rate_change: {change(full['high_priority_top5_rate_mean'], summary_map['w/o Priority']['high_priority_top5_rate_mean']):.2f}%")
    print(f"- priority_weighted_completion_time_change: {change(full['priority_weighted_completion_time_mean'], summary_map['w/o Priority']['priority_weighted_completion_time_mean']):.2f}%")

    print("\nFull-Proposed-Balanced vs w/o Risk:")
    print(f"- high_risk_avg_response_time_change: {change(full['high_risk_avg_response_time_mean'], summary_map['w/o Risk']['high_risk_avg_response_time_mean']):.2f}%")
    print(f"- risk_weighted_completion_time_change: {change(full['risk_weighted_completion_time_mean'], summary_map['w/o Risk']['risk_weighted_completion_time_mean']):.2f}%")

    print("\nFull-Proposed-Balanced vs w/o Abnormal:")
    print(f"- abnormal_priority_rate_change: {change(full['abnormal_priority_rate_mean'], summary_map['w/o Abnormal']['abnormal_priority_rate_mean']):.2f}%")
    print(f"- abnormal_avg_response_time_change: {change(full['abnormal_avg_response_time_mean'], summary_map['w/o Abnormal']['abnormal_avg_response_time_mean']):.2f}%")

    print("\nFull-Proposed-Balanced vs w/o Complexity:")
    print(f"- total_path_length_change: {change(full['total_path_length_mean'], summary_map['w/o Complexity']['total_path_length_mean']):.2f}%")
    print(f"- total_inspection_time_change: {change(full['total_inspection_time_mean'], summary_map['w/o Complexity']['total_inspection_time_mean']):.2f}%")

    print("\nFull-Proposed-Balanced vs w/o Energy:")
    print(f"- total_path_length_change: {change(full['total_path_length_mean'], summary_map['w/o Energy']['total_path_length_mean']):.2f}%")
    print(f"- total_inspection_time_change: {change(full['total_inspection_time_mean'], summary_map['w/o Energy']['total_inspection_time_mean']):.2f}%")


def main():
    width = 30
    height = 30
    obstacle_ratio = 0.2
    task_num = 20
    start_pos = (2, 2)
    seeds = list(range(30))
    methods = [
        "Full-Proposed-Balanced",
        "w/o Priority",
        "w/o Risk",
        "w/o Abnormal",
        "w/o Complexity",
        "w/o Energy",
    ]
    weights_map = {
        "Full-Proposed-Balanced": {"alpha": 0.22, "beta": 0.18, "lambda_abnormal": 0.15, "gamma": 0.27, "delta": 0.12, "eta": 0.06},
        "w/o Priority": {"alpha": 0.0, "beta": 0.18, "lambda_abnormal": 0.15, "gamma": 0.27, "delta": 0.12, "eta": 0.06},
        "w/o Risk": {"alpha": 0.22, "beta": 0.0, "lambda_abnormal": 0.15, "gamma": 0.27, "delta": 0.12, "eta": 0.06},
        "w/o Abnormal": {"alpha": 0.22, "beta": 0.18, "lambda_abnormal": 0.0, "gamma": 0.27, "delta": 0.12, "eta": 0.06},
        "w/o Complexity": {"alpha": 0.22, "beta": 0.18, "lambda_abnormal": 0.15, "gamma": 0.27, "delta": 0.0, "eta": 0.06},
        "w/o Energy": {"alpha": 0.22, "beta": 0.18, "lambda_abnormal": 0.15, "gamma": 0.27, "delta": 0.12, "eta": 0.0},
    }

    results_dir = Path(__file__).resolve().parent / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    results_path = results_dir / "ablation_results.csv"
    summary_path = results_dir / "ablation_summary.csv"

    rows = []
    for seed in seeds:
        grid_map = create_grid_map(width, height, obstacle_ratio, start_pos, seed)
        tasks = create_tasks(grid_map, start_pos, task_num, seed)
        for method in methods:
            result = run_ablation_version(grid_map, tasks, start_pos, seed, method, weights_map[method])
            rows.append(
                {
                    "seed": seed,
                    "method": method,
                    "completed_task_num": result["completed_task_num"],
                    "total_path_length": result["total_path_length"],
                    "total_inspection_time": result["total_inspection_time"],
                    "high_priority_avg_response_time": result["high_priority_avg_response_time"],
                    "high_risk_avg_response_time": result["high_risk_avg_response_time"],
                    "abnormal_task_ids": result["abnormal_task_ids"],
                    "abnormal_priority_rate": result["abnormal_priority_rate"],
                    "abnormal_avg_response_time": result["abnormal_avg_response_time"],
                    "high_priority_top5_rate": result["high_priority_top5_rate"],
                    "priority_weighted_completion_time": result["priority_weighted_completion_time"],
                    "risk_weighted_completion_time": result["risk_weighted_completion_time"],
                    "replanning_count": result["replanning_count"],
                    "task_sequence": "->".join(result["task_sequence"]),
                }
            )

    save_csv(rows, results_path)
    summary_rows = summarize(rows)
    save_summary(summary_rows, summary_path)

    print("Experiment settings:")
    print(f"- map size: {width}x{height}")
    print(f"- obstacle ratio: {obstacle_ratio}")
    print(f"- task num: {task_num}")
    print(f"- seed count: {len(seeds)}")
    print(f"- ablation methods: {', '.join(methods)}")
    print(f"\nTotal experiment rows: {len(rows)}")

    counts = {m: 0 for m in methods}
    for row in rows:
        counts[row["method"]] += 1
    for method in methods:
        print(f"{method}: {counts[method]} rows")

    print_summary(summary_rows)
    summary_map = {row["method"]: row for row in summary_rows}
    print_differences(summary_map)

    print(f"\nResults saved to: {results_path}")
    print(f"Summary saved to: {summary_path}")


if __name__ == "__main__":
    main()
