import copy
import csv
import math
import random
import statistics
import time

from adaptive_rh_pads_allocator import AdaptiveRHPADSAllocator
from advanced_baselines import PriorityGreedyAllocator, TSP2OptAllocator
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


def create_grid_map(width, height, obstacle_ratio, start_pos, seed):
    random.seed(seed)
    grid_map = [[0 for _ in range(width)] for _ in range(height)]
    obstacle_num = int(width * height * obstacle_ratio)
    candidates = [
        (x, y)
        for y in range(height)
        for x in range(width)
        if (x, y) != start_pos
    ]
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
        raise ValueError("Not enough free cells to create tasks.")
    selected = random.sample(free_cells, task_num)
    tasks = []
    for i, (x, y) in enumerate(selected):
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
        if task.task_id in task_finish_times:
            weighted_sum += task.priority * task_finish_times[task.task_id]
            priority_sum += task.priority
    return weighted_sum / priority_sum if priority_sum else 0.0


def compute_high_priority_avg_response_time(tasks, task_finish_times, threshold=0.75):
    values = [
        task_finish_times[task.task_id]
        for task in tasks
        if task.priority >= threshold and task.task_id in task_finish_times
    ]
    return sum(values) / len(values) if values else 0.0


def compute_high_priority_top5_rate(tasks, task_sequence, threshold=0.75):
    if not task_sequence:
        return 0.0
    task_lookup = {task.task_id: task for task in tasks}
    top5 = task_sequence[:5]
    count = sum(
        1
        for task_id in top5
        if task_id in task_lookup and task_lookup[task_id].priority >= threshold
    )
    return count / min(5, len(top5)) * 100.0


def build_allocator(method, grid_map, tasks, start_pos, robot_speed, inspection_time):
    if method == "AStarOnly":
        return AStarOnlyAllocator(
            grid_map=grid_map,
            tasks=tasks,
            start_pos=start_pos,
            robot_speed=robot_speed,
            inspection_time=inspection_time,
        )
    if method == "TSP-2opt":
        return TSP2OptAllocator(
            grid_map=grid_map,
            tasks=tasks,
            start_pos=start_pos,
            robot_speed=robot_speed,
            inspection_time=inspection_time,
        )
    if method == "Greedy-PADS":
        return PriorityCostTaskAllocator(
            grid_map=grid_map,
            tasks=tasks,
            start_pos=start_pos,
            robot_speed=robot_speed,
            inspection_time=inspection_time,
            **BALANCED_WEIGHTS,
        )
    if method == "Priority-Greedy":
        return PriorityGreedyAllocator(
            grid_map=grid_map,
            tasks=tasks,
            start_pos=start_pos,
            robot_speed=robot_speed,
            inspection_time=inspection_time,
        )
    if method == "RH-PADS":
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
    if method == "RH-PADS-L":
        return RHProposedAllocatorV2(
            grid_map=grid_map,
            tasks=tasks,
            start_pos=start_pos,
            robot_speed=robot_speed,
            inspection_time=inspection_time,
            **BALANCED_WEIGHTS,
            horizon=3,
            beam_width=5,
            candidate_pool_size=6,
        )
    if method == "A-RH-PADS":
        return AdaptiveRHPADSAllocator(
            grid_map=grid_map,
            tasks=tasks,
            start_pos=start_pos,
            robot_speed=robot_speed,
            inspection_time=inspection_time,
            horizon=4,
            beam_width=8,
            candidate_pool_size=10,
        )
    if method == "A-RH-PADS-L":
        return AdaptiveRHPADSAllocator(
            grid_map=grid_map,
            tasks=tasks,
            start_pos=start_pos,
            robot_speed=robot_speed,
            inspection_time=inspection_time,
            horizon=3,
            beam_width=5,
            candidate_pool_size=6,
        )
    raise ValueError(f"Unsupported method: {method}")


def build_ablation_allocator(method, grid_map, tasks, start_pos, robot_speed, inspection_time):
    kwargs = {
        "grid_map": grid_map,
        "tasks": tasks,
        "start_pos": start_pos,
        "robot_speed": robot_speed,
        "inspection_time": inspection_time,
        "horizon": 4,
        "beam_width": 8,
        "candidate_pool_size": 10,
    }
    if method == "A-RH-PADS-Full":
        return AdaptiveRHPADSAllocator(**kwargs)
    if method == "A-RH-PADS-FixedLambda":
        return AdaptiveRHPADSAllocator(**kwargs, fixed_lambda=0.55)
    if method == "A-RH-PADS-NoUrgencyPressure":
        return AdaptiveRHPADSAllocator(**kwargs, use_urgency_pressure=False)
    if method == "A-RH-PADS-NoAbnormalPressure":
        return AdaptiveRHPADSAllocator(**kwargs, use_abnormal_pressure=False)
    if method == "A-RH-PADS-NoPathPressure":
        return AdaptiveRHPADSAllocator(**kwargs, use_path_pressure=False)
    if method == "A-RH-PADS-ResponseOnly":
        return AdaptiveRHPADSAllocator(**kwargs, fixed_lambda=0.85)
    if method == "A-RH-PADS-CostOnly":
        return AdaptiveRHPADSAllocator(**kwargs, fixed_lambda=0.25)
    if method == "A-RH-PADS-NoFinishTimeResponse":
        return AdaptiveRHPADSAllocator(**kwargs, no_finish_time_response=True)
    raise ValueError(f"Unsupported ablation method: {method}")


def extract_lambda_stats(result):
    sequence = result.get("lambda_sequence", []) or []
    if sequence:
        return {
            "lambda_mean": statistics.mean(sequence),
            "lambda_min": min(sequence),
            "lambda_max": max(sequence),
            "lambda_std": statistics.stdev(sequence) if len(sequence) > 1 else 0.0,
        }
    return {
        "lambda_mean": float(result.get("lambda_mean", 0.0) or 0.0),
        "lambda_min": float(result.get("lambda_min", 0.0) or 0.0),
        "lambda_max": float(result.get("lambda_max", 0.0) or 0.0),
        "lambda_std": float(result.get("lambda_std", 0.0) or 0.0),
    }


def normalize_result(result, tasks, task_finish_times, task_sequence):
    result.setdefault("completed_task_num", len(task_sequence))
    result.setdefault("task_sequence", task_sequence)
    result.setdefault(
        "high_priority_avg_response_time",
        compute_high_priority_avg_response_time(tasks, task_finish_times),
    )
    result.setdefault(
        "priority_weighted_completion_time",
        compute_priority_weighted_completion_time(tasks, task_finish_times),
    )
    result.setdefault(
        "high_priority_top5_rate",
        compute_high_priority_top5_rate(tasks, task_sequence),
    )
    for key, value in extract_lambda_stats(result).items():
        result.setdefault(key, value)
    return result


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
    runtime_ms = (time.perf_counter() - start_time) * 1000.0
    result = normalize_result(
        result,
        tasks_copy,
        allocator.task_finish_times,
        allocator.task_sequence,
    )
    result["algorithm_runtime_ms"] = runtime_ms
    return result


def select_for_allocator(allocator, method):
    if method == "AStarOnly":
        candidates = []
        for task in allocator.get_unfinished_tasks():
            path_info = allocator.planner.plan(allocator.current_pos, task.position)
            if not path_info.get("reachable", False):
                continue
            cost = (
                path_info["path_length"]
                + 0.5 * path_info["turn_count"]
                + 0.2 * path_info["obstacle_nearby_count"]
            )
            candidates.append((cost, task.task_id, task, path_info))
        if not candidates:
            return None, None, None
        cost, _, task, path_info = sorted(candidates, key=lambda item: (item[0], item[1]))[0]
        return task, path_info, {
            "task_id": task.task_id,
            "selected_task_id": task.task_id,
            "astar_only_cost": cost,
            "path_length": path_info["path_length"],
            "planned_path_length": path_info["path_length"],
            "priority": task.priority,
            "risk": task.risk,
            "abnormal_weight": task.abnormal_weight,
            "turn_count": path_info["turn_count"],
            "obstacle_nearby_count": path_info["obstacle_nearby_count"],
        }

    if isinstance(allocator, AdaptiveRHPADSAllocator):
        task, record = allocator.select_next_task(
            allocator.current_pos,
            allocator.total_inspection_time,
        )
        if task is None:
            return None, None, None
        path_info = record.pop("path_info")
        return task, path_info, record

    if isinstance(allocator, PriorityGreedyAllocator):
        return allocator.select_next_task()

    return allocator.select_next_task(allocator.current_pos)


def execute_selected_task(allocator, selected_task, path_info, record):
    path_length = float(path_info["path_length"])
    travel_time = path_length / allocator.robot_speed
    finish_time = allocator.total_inspection_time + travel_time + allocator.inspection_time

    allocator.total_path_length += path_length
    allocator.total_inspection_time = finish_time
    selected_task.mark_completed()
    allocator.current_pos = selected_task.position
    allocator.task_sequence.append(selected_task.task_id)
    allocator.task_finish_times[selected_task.task_id] = finish_time

    if isinstance(allocator, AdaptiveRHPADSAllocator):
        allocator.lambda_sequence.append(record.get("lambda_t", 0.0))

    stored_record = dict(record)
    stored_record["finish_time"] = finish_time
    stored_record["travel_time"] = travel_time
    stored_record.setdefault("path_length", path_length)
    stored_record.setdefault("planned_path_length", path_length)
    allocator.selection_records.append(stored_record)


def sample_abnormal_tasks(allocator, seed, abnormal_task_num=4):
    remaining = [task for task in allocator.tasks if task.status == 0]
    sample_n = min(abnormal_task_num, len(remaining))
    rng = random.Random(seed + 5000)
    return rng.sample(remaining, sample_n) if sample_n else []


def update_abnormal_weights(abnormal_tasks):
    for task in abnormal_tasks:
        if task.status == 0:
            task.abnormal_weight = 1.0


def compute_abnormal_metrics(task_sequence, abnormal_task_ids, trigger_time, task_finish_times, total_time):
    if not abnormal_task_ids:
        return 0.0, 0.0
    post_sequence = task_sequence[3:]
    first_after_trigger = post_sequence[: len(abnormal_task_ids)]
    abnormal_priority_rate = (
        len(set(first_after_trigger) & set(abnormal_task_ids))
        / len(abnormal_task_ids)
        * 100.0
    )
    response_times = []
    for task_id in abnormal_task_ids:
        if task_id in task_finish_times:
            response_times.append(task_finish_times[task_id] - trigger_time)
        else:
            response_times.append(max(0.0, total_time - trigger_time))
    return abnormal_priority_rate, sum(response_times) / len(response_times)


def run_method_with_abnormal(
    method,
    grid_map,
    tasks,
    start_pos,
    seed,
    robot_speed,
    inspection_time,
    ablation=False,
):
    tasks_copy = copy.deepcopy(tasks)
    allocator = (
        build_ablation_allocator(method, grid_map, tasks_copy, start_pos, robot_speed, inspection_time)
        if ablation
        else build_allocator(method, grid_map, tasks_copy, start_pos, robot_speed, inspection_time)
    )

    abnormal_task_ids = []
    abnormal_trigger_time = 0.0
    replanning_count = 0
    lambda_before_abnormal = 0.0
    lambda_after_abnormal = 0.0
    trigger_done = False

    start_time = time.perf_counter()
    while allocator.get_unfinished_tasks():
        selected_task, path_info, record = select_for_allocator(allocator, method)
        if selected_task is None:
            print(f"No reachable unfinished tasks. Stop {method}.")
            break
        execute_selected_task(allocator, selected_task, path_info, record)

        if len(allocator.task_sequence) == 3 and not trigger_done:
            trigger_done = True
            abnormal_trigger_time = allocator.total_inspection_time
            if isinstance(allocator, AdaptiveRHPADSAllocator) and allocator.lambda_sequence:
                lambda_before_abnormal = allocator.lambda_sequence[-1]
            abnormal_tasks = sample_abnormal_tasks(allocator, seed)
            abnormal_task_ids = [task.task_id for task in abnormal_tasks]
            update_abnormal_weights(abnormal_tasks)
            replanning_count = 0 if method == "AStarOnly" else 1

        if (
            trigger_done
            and lambda_after_abnormal == 0.0
            and isinstance(allocator, AdaptiveRHPADSAllocator)
            and len(allocator.lambda_sequence) >= 4
        ):
            lambda_after_abnormal = allocator.lambda_sequence[-1]

    runtime_ms = (time.perf_counter() - start_time) * 1000.0
    if isinstance(allocator, AdaptiveRHPADSAllocator) and lambda_after_abnormal == 0.0:
        after_values = allocator.lambda_sequence[3:]
        lambda_after_abnormal = after_values[0] if after_values else 0.0

    abnormal_priority_rate, abnormal_avg_response_time = compute_abnormal_metrics(
        allocator.task_sequence,
        abnormal_task_ids,
        abnormal_trigger_time,
        allocator.task_finish_times,
        allocator.total_inspection_time,
    )
    lambda_mean, lambda_std, lambda_min, lambda_max = (
        allocator.lambda_stats()
        if isinstance(allocator, AdaptiveRHPADSAllocator)
        else (0.0, 0.0, 0.0, 0.0)
    )
    return {
        "task_sequence": allocator.task_sequence,
        "completed_task_num": len(allocator.task_sequence),
        "total_path_length": allocator.total_path_length,
        "total_inspection_time": allocator.total_inspection_time,
        "high_priority_avg_response_time": compute_high_priority_avg_response_time(
            allocator.tasks,
            allocator.task_finish_times,
        ),
        "priority_weighted_completion_time": compute_priority_weighted_completion_time(
            allocator.tasks,
            allocator.task_finish_times,
        ),
        "high_priority_top5_rate": compute_high_priority_top5_rate(
            allocator.tasks,
            allocator.task_sequence,
        ),
        "abnormal_task_ids": ",".join(abnormal_task_ids),
        "abnormal_priority_rate": abnormal_priority_rate,
        "abnormal_avg_response_time": abnormal_avg_response_time,
        "replanning_count": replanning_count,
        "lambda_before_abnormal": lambda_before_abnormal,
        "lambda_after_abnormal": lambda_after_abnormal,
        "lambda_change": lambda_after_abnormal - lambda_before_abnormal,
        "algorithm_runtime_ms": runtime_ms,
        "lambda_mean": lambda_mean,
        "lambda_min": lambda_min,
        "lambda_max": lambda_max,
        "lambda_std": lambda_std,
        "selection_records": allocator.selection_records,
    }


def save_csv(rows, path, fieldnames):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def summarize(rows, methods, metrics):
    groups = {}
    for row in rows:
        groups.setdefault(row["method"], []).append(row)
    summary_rows = []
    for method in methods:
        items = groups.get(method, [])
        summary = {"method": method}
        for metric in metrics:
            values = [float(item.get(metric, 0.0) or 0.0) for item in items]
            summary[f"{metric}_mean"] = statistics.mean(values) if values else 0.0
            summary[f"{metric}_std"] = statistics.stdev(values) if len(values) > 1 else 0.0
        summary_rows.append(summary)
    return summary_rows


def percent_change(new_value, base_value):
    if float(base_value) == 0.0:
        return 0.0
    return (float(new_value) - float(base_value)) / float(base_value) * 100.0


def print_counts(rows, methods):
    print(f"Total experiment rows: {len(rows)}")
    for method in methods:
        print(f"{method}: {sum(1 for row in rows if row['method'] == method)} rows")


def print_summary_table(summary_rows, metrics):
    print("\nSummary:")
    header = "Method | " + " | ".join(f"{metric} mean" for metric in metrics)
    print(header)
    for row in summary_rows:
        values = [f"{float(row.get(metric + '_mean', 0.0)):.2f}" for metric in metrics]
        print(f"{row['method']} | " + " | ".join(values))


def print_change_block(summary_map, target_method, base_method, metrics):
    if target_method not in summary_map or base_method not in summary_map:
        return
    print(f"\n{target_method} vs {base_method} mean change:")
    target = summary_map[target_method]
    base = summary_map[base_method]
    for metric in metrics:
        change = percent_change(target[f"{metric}_mean"], base[f"{metric}_mean"])
        print(f"- {metric}: {change:.2f}%")
