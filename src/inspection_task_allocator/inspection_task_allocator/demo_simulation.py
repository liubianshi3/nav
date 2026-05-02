import os
import random
import sys

import numpy as np

if __package__ is None or __package__ == "":
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from inspection_task_allocator.task_allocator import PriorityCostTaskAllocator
    from inspection_task_allocator.task_model import InspectionTask
else:
    from .task_allocator import PriorityCostTaskAllocator
    from .task_model import InspectionTask


def generate_map(width, height, obstacle_ratio, start, seed=42):
    rng = random.Random(seed)
    grid = np.zeros((height, width), dtype=int)
    protected = {start}

    candidates = [(x, y) for y in range(height) for x in range(width) if (x, y) not in protected]
    rng.shuffle(candidates)
    obstacle_count = int(width * height * obstacle_ratio)
    for x, y in candidates[:obstacle_count]:
        grid[y][x] = 1
    return grid


def reachable_cells(grid, start):
    height, width = grid.shape
    stack = [start]
    visited = {start}
    while stack:
        x, y = stack.pop()
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= nx < width and 0 <= ny < height and grid[ny][nx] == 0 and (nx, ny) not in visited:
                visited.add((nx, ny))
                stack.append((nx, ny))
    visited.discard(start)
    return list(visited)


def create_tasks(grid, start, count=20, seed=7):
    rng = random.Random(seed)
    reachable = reachable_cells(grid, start)
    rng.shuffle(reachable)
    tasks = []
    for idx, (x, y) in enumerate(reachable[:count]):
        tasks.append(
            InspectionTask(
                task_id=f"P{idx + 1}",
                x=x,
                y=y,
                priority=round(rng.random(), 3),
                risk=round(rng.random(), 3),
                abnormal_weight=0.0,
            )
        )
    return tasks


def main():
    start = (2, 2)
    width = height = 30

    rng = random.Random(123)
    task_points = []
    while len(task_points) < 20:
        p = (rng.randint(0, width - 1), rng.randint(0, height - 1))
        if p != start and p not in task_points:
            task_points.append(p)

    grid = generate_map(width, height, obstacle_ratio=0.20, start=start)
    tasks = create_tasks(grid, start=start, count=20)

    print(f"Initial position: {start}")
    allocator = PriorityCostTaskAllocator(
        grid_map=grid.tolist(),
        start=start,
        tasks=tasks,
        robot_speed=0.6,
        inspection_time=5.0,
    )
    result = allocator.allocate()

    for detail in result.task_details:
        print(
            f"Selected task: {detail['task_id']}, order={detail['order']}, "
            f"score={detail['score']:.3f}, path_length={detail['path_length']:.1f}"
        )

    print(f"Final sequence: {' -> '.join(result.sequence)}")
    print(f"Total path length: {result.total_path_length:.1f}")
    print(f"Total inspection time: {result.total_inspection_time:.1f} s")
    print(f"High-priority average response time: {result.high_priority_avg_response_time:.1f} s")


if __name__ == "__main__":
    main()
