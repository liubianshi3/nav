import copy
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


def print_tasks(tasks):
    print("Tasks:")
    for task in tasks:
        print(
            f"Task {task.task_id}: pos=({task.x},{task.y}), priority={task.priority:.2f}, risk={task.risk:.2f}"
        )


def run_method(name, allocator_class, grid_map, tasks, start_pos):
    tasks_copy = copy.deepcopy(tasks)
    allocator = allocator_class(
        grid_map=grid_map,
        tasks=tasks_copy,
        start_pos=start_pos,
        robot_speed=0.6,
        inspection_time=5.0,
    )
    results = allocator.run()
    return results


def main():
    seed = 42
    width = 30
    height = 30
    obstacle_ratio = 0.2
    task_num = 20
    start_pos = (2, 2)

    grid_map = create_grid_map(width, height, obstacle_ratio, start_pos, seed)
    tasks = create_tasks(grid_map, start_pos, task_num, seed)

    print(f"seed: {seed}")
    print(f"map size: {width}x{height}")
    print(f"obstacle ratio: {obstacle_ratio}")
    print(f"task num: {task_num}")
    print(f"start position: {start_pos}")
    print()
    print_tasks(tasks)
    print()

    methods = [
        ("FS", FixedSequenceAllocator),
        ("NNF", NearestNeighborAllocator),
        ("AStarOnly", AStarOnlyAllocator),
        ("Proposed", PriorityCostTaskAllocator),
    ]

    all_results = {}
    for method_name, allocator_class in methods:
        all_results[method_name] = run_method(method_name, allocator_class, grid_map, tasks, start_pos)

    print("Method | Completed | Total Path | Total Time | High Priority Avg Response")
    for method_name in ["FS", "NNF", "AStarOnly", "Proposed"]:
        result = all_results[method_name]
        print(
            f"{method_name} | {result['completed_task_num']} | {result['total_path_length']:.2f} | "
            f"{result['total_inspection_time']:.2f} | {result['high_priority_avg_response_time']:.2f}"
        )

    for method_name in ["FS", "NNF", "AStarOnly", "Proposed"]:
        print(f"\n{method_name} sequence:")
        print(" -> ".join(all_results[method_name]["task_sequence"]))

    if all_results["AStarOnly"]["total_path_length"] > 0:
        path_change = (
            (all_results["Proposed"]["total_path_length"] - all_results["AStarOnly"]["total_path_length"])
            / all_results["AStarOnly"]["total_path_length"]
            * 100
        )
    else:
        path_change = 0.0

    if all_results["AStarOnly"]["total_inspection_time"] > 0:
        time_change = (
            (all_results["Proposed"]["total_inspection_time"] - all_results["AStarOnly"]["total_inspection_time"])
            / all_results["AStarOnly"]["total_inspection_time"]
            * 100
        )
    else:
        time_change = 0.0

    if all_results["AStarOnly"]["high_priority_avg_response_time"] > 0:
        response_change = (
            (
                all_results["Proposed"]["high_priority_avg_response_time"]
                - all_results["AStarOnly"]["high_priority_avg_response_time"]
            )
            / all_results["AStarOnly"]["high_priority_avg_response_time"]
            * 100
        )
    else:
        response_change = 0.0

    print("\nProposed vs AStarOnly:")
    print(f"- total_path_length change: {path_change:.2f}%")
    print(f"- total_inspection_time change: {time_change:.2f}%")
    print(f"- high_priority_avg_response_time change: {response_change:.2f}%")


if __name__ == "__main__":
    main()
