import heapq
import math


class AStarPlanner:
    def __init__(self, grid_map):
        self.grid_map = grid_map
        self.height = len(grid_map)
        self.width = len(grid_map[0]) if self.height > 0 else 0

    def in_bounds(self, node):
        x, y = node
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, node):
        if not self.in_bounds(node):
            return False
        x, y = node
        return self.grid_map[y][x] == 0

    def heuristic(self, node, goal):
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    def get_neighbors(self, node):
        x, y = node
        neighbors = []
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if self.is_free((nx, ny)):
                neighbors.append((nx, ny))
        return neighbors

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def count_turns(self, path):
        if len(path) < 3:
            return 0
        turns = 0
        for i in range(1, len(path) - 1):
            x1, y1 = path[i - 1]
            x2, y2 = path[i]
            x3, y3 = path[i + 1]
            dir1 = (x2 - x1, y2 - y1)
            dir2 = (x3 - x2, y3 - y2)
            if dir1 != dir2:
                turns += 1
        return turns

    def count_nearby_obstacles(self, path):
        obstacle_count = 0
        for x, y in path:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = x + dx, y + dy
                    if self.in_bounds((nx, ny)) and self.grid_map[ny][nx] == 1:
                        obstacle_count += 1
        return obstacle_count

    def plan(self, start, goal):
        if not self.is_free(start) or not self.is_free(goal):
            return {
                "path": [],
                "path_length": math.inf,
                "turn_count": 0,
                "obstacle_nearby_count": 0,
                "reachable": False,
            }

        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start))
        came_from = {}
        g_score = {start: 0}
        closed_set = set()

        while open_set:
            _, current_g, current = heapq.heappop(open_set)
            if current in closed_set:
                continue
            closed_set.add(current)

            if current == goal:
                path = self.reconstruct_path(came_from, current)
                path_length = len(path) - 1
                turn_count = self.count_turns(path)
                obstacle_nearby_count = self.count_nearby_obstacles(path)
                return {
                    "path": path,
                    "path_length": path_length,
                    "turn_count": turn_count,
                    "obstacle_nearby_count": obstacle_nearby_count,
                    "reachable": True,
                }

            for neighbor in self.get_neighbors(current):
                tentative_g = current_g + 1
                if tentative_g < g_score.get(neighbor, math.inf):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))

        return {
            "path": [],
            "path_length": math.inf,
            "turn_count": 0,
            "obstacle_nearby_count": 0,
            "reachable": False,
        }
