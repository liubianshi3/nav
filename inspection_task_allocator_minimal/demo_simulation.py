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


def print_grid_map(grid_map, start_pos, tasks):
    task_positions = {task.position for task in tasks}
    sx, sy = start_pos
    print("Grid map:")
    for y, row in enumerate(grid_map):
        line = []
        for x, value in enumerate(row):
            if (x, y) == (sx, sy):
                line.append("S")
            elif (x, y) in task_positions:
                line.append("T")
            elif value == 1:
                line.append("#")
            else:
                line.append(".")
        print("".join(line))


def print_tasks(tasks):
    print("Tasks:")
    for task in tasks:
        print(
            f"Task {task.task_id}: pos=({task.x},{task.y}), "
            f"priority={task.priority:.2f}, risk={task.risk:.2f}, abnormal={task.abnormal_weight:.2f}"
        )


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
    print_grid_map(grid_map, start_pos, tasks)
    print()
    print_tasks(tasks)
    print()

    allocator = PriorityCostTaskAllocator(
        grid_map=grid_map,
        tasks=tasks,
        start_pos=start_pos,
        robot_speed=0.6,
        inspection_time=5.0,
    )

    results = allocator.run()

    print("Final task sequence:")
    print(" -> ".join(results["task_sequence"]))
    print()
    print(f"Total path length: {results['total_path_length']:.2f}")
    print(f"Total inspection time: {results['total_inspection_time']:.2f} s")
    print(f"High-priority average response time: {results['high_priority_avg_response_time']:.2f} s")
    print(f"Completed task num: {results['completed_task_num']} / {task_num}")
    print()

    for idx, record in enumerate(results["selection_records"], start=1):
        print(f"Step {idx}:")
        print(f"  selected task: {record['task_id']}")
        print(f"  score: {record['score']:.4f}")
        print(f"  path_length: {record['path_length']}")
        print(f"  distance_cost: {record['distance_cost']:.4f}")
        print(f"  complexity_cost: {record['complexity_cost']:.4f}")
        print(f"  energy_cost: {record['energy_cost']:.4f}")
        print(f"  priority: {record['priority']:.4f}")
        print(f"  risk: {record['risk']:.4f}")
        print(f"  abnormal_weight: {record['abnormal_weight']:.4f}")
        print(f"  finish_time: {record['finish_time']:.2f}")
        print()


if __name__ == "__main__":
    main()
