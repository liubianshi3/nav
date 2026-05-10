import math
from heapq import heappop, heappush


class AStarPlanner:
    def __init__(self, grid_map):
        self.grid_map = grid_map
        self.height = len(grid_map)
        self.width = len(grid_map[0]) if self.height else 0

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def is_free(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height and self.grid_map[y][x] == 0

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def plan(self, start, goal):
        if not self.is_free(*start) or not self.is_free(*goal):
            return [], float("inf"), 0, 0

        open_set = []
        heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        closed = set()

        while open_set:
            _, current = heappop(open_set)
            if current in closed:
                continue
            if current == goal:
                path = self.reconstruct_path(came_from, current)
                path_length = max(0, len(path) - 1)
                turn_count = self.count_turns(path)
                obstacle_nearby_count = self.count_nearby_obstacles(path, self.grid_map)
                return path, float(path_length), turn_count, obstacle_nearby_count

            closed.add(current)
            cx, cy = current
            for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                if not self.is_free(nx, ny):
                    continue
                tentative_g = g_score[current] + 1
                neighbor = (nx, ny)
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    f_score[neighbor] = f
                    heappush(open_set, (f, neighbor))

        return [], float("inf"), 0, 0

    @staticmethod
    def count_turns(path):
        if len(path) < 3:
            return 0
        turns = 0
        prev_dir = (path[1][0] - path[0][0], path[1][1] - path[0][1])
        for i in range(2, len(path)):
            cur_dir = (path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            if cur_dir != prev_dir:
                turns += 1
            prev_dir = cur_dir
        return turns

    @staticmethod
    def count_nearby_obstacles(path, grid_map):
        if not path:
            return 0
        height = len(grid_map)
        width = len(grid_map[0]) if height else 0
        count = 0
        for x, y in path:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < width and 0 <= ny < height and grid_map[ny][nx] == 1:
                        count += 1
        return count
