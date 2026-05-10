import random
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from astar_planner import AStarPlanner
from task_model import InspectionTask


def _wall_grid(width, height):
    return [[1 for _ in range(width)] for _ in range(height)]


def _carve_rect(grid_map, x0, y0, x1, y1):
    height = len(grid_map)
    width = len(grid_map[0])
    for y in range(max(0, y0), min(height, y1)):
        for x in range(max(0, x0), min(width, x1)):
            grid_map[y][x] = 0


def generate_corridor_map(width=50, height=30):
    grid_map = _wall_grid(width, height)
    mid_y = height // 2
    _carve_rect(grid_map, 2, mid_y - 1, width - 2, mid_y + 2)
    for x in range(7, width - 5, 8):
        branch_top = 3 if (x // 8) % 2 == 0 else mid_y
        branch_bottom = mid_y if (x // 8) % 2 == 0 else height - 3
        _carve_rect(grid_map, x - 1, min(branch_top, branch_bottom), x + 2, max(branch_top, branch_bottom) + 1)
    start_pos = (2, mid_y)
    grid_map[start_pos[1]][start_pos[0]] = 0
    return grid_map, start_pos


def generate_room_corridor_map(width=50, height=40):
    grid_map = _wall_grid(width, height)
    corridor_y = height // 2
    _carve_rect(grid_map, 2, corridor_y - 1, width - 2, corridor_y + 2)
    room_w = 10
    room_h = 9
    for idx, x0 in enumerate((4, 18, 32)):
        _carve_rect(grid_map, x0, 4, x0 + room_w, 4 + room_h)
        _carve_rect(grid_map, x0, height - 4 - room_h, x0 + room_w, height - 4)
        door_x = x0 + room_w // 2
        _carve_rect(grid_map, door_x - 1, 4 + room_h, door_x + 2, corridor_y + 1)
        _carve_rect(grid_map, door_x - 1, corridor_y, door_x + 2, height - 4 - room_h)
    start_pos = (2, corridor_y)
    grid_map[start_pos[1]][start_pos[0]] = 0
    return grid_map, start_pos


def generate_bottleneck_map(width=50, height=40):
    grid_map = [[0 for _ in range(width)] for _ in range(height)]
    for y in range(height):
        grid_map[y][width // 3] = 1
        grid_map[y][2 * width // 3] = 1
    for y in (height // 4, height // 2, 3 * height // 4):
        _carve_rect(grid_map, width // 3, y - 1, width // 3 + 1, y + 2)
    for y in (height // 3, 2 * height // 3):
        _carve_rect(grid_map, 2 * width // 3, y - 1, 2 * width // 3 + 1, y + 2)
    for x in range(width):
        grid_map[0][x] = 1
        grid_map[height - 1][x] = 1
    for y in range(height):
        grid_map[y][0] = 1
        grid_map[y][width - 1] = 1
    start_pos = (2, height // 2)
    grid_map[start_pos[1]][start_pos[0]] = 0
    return grid_map, start_pos


def _free_cells(grid_map):
    return [
        (x, y)
        for y, row in enumerate(grid_map)
        for x, value in enumerate(row)
        if value == 0
    ]


def sample_tasks_on_free_cells(grid_map, task_num, seed, start_pos=None):
    rng = random.Random(seed + 7000)
    if start_pos is None:
        start_pos = (2, len(grid_map) // 2)
    planner = AStarPlanner(grid_map)
    cells = [
        cell
        for cell in _free_cells(grid_map)
        if cell != start_pos and planner.plan(start_pos, cell).get("reachable", False)
    ]
    if len(cells) < task_num:
        cells = [cell for cell in _free_cells(grid_map) if cell != start_pos]
    selected = rng.sample(cells, min(task_num, len(cells)))
    tasks = []
    for i, (x, y) in enumerate(selected):
        tasks.append(
            InspectionTask(
                task_id=f"P{i + 1}",
                x=x,
                y=y,
                priority=rng.random(),
                risk=rng.random(),
                abnormal_weight=0.0,
                status=0,
            )
        )
    return tasks


def render_map_with_tasks(grid_map, tasks, start_pos, output_path):
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 6), dpi=300)
    ax.imshow(grid_map, cmap="gray_r", origin="upper")
    ax.scatter([start_pos[0]], [start_pos[1]], c="tab:green", s=80, marker="s", label="Start")
    for task in tasks:
        color = "tab:red" if task.priority >= 0.75 else "tab:blue"
        ax.scatter([task.x], [task.y], c=color, s=35)
        ax.text(task.x + 0.2, task.y + 0.2, task.task_id, fontsize=6)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=300, facecolor="white", bbox_inches="tight")
    plt.close(fig)
