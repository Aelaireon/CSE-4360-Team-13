#!/usr/bin/env pybricks-micropython
from path_follower import PathFollower
import heapq


class PathFinder:
    def __init__(self, position, parent=None):
        self.position = position  # (row, col)
        self.parent = parent

        self.g = 0
        self.h = 0
        self.f = 0  # g + h

    def __lt__(self, other):
        return self.f < other.f

    @staticmethod
    def heuristic(current, goal):
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

    @staticmethod
    def astar(grid, start, end):
        if not grid or not grid[0]:
            return None

        if start == end:
            return [start]

        rows = len(grid)
        cols = len(grid[0])

        if not (0 <= start[0] < rows and 0 <= start[1] < cols):
            return None
        if not (0 <= end[0] < rows and 0 <= end[1] < cols):
            return None
        if grid[start[0]][start[1]] != 0 or grid[end[0]][end[1]] != 0:
            return None

        start_node = PathFinder(start)
        open_list = []
        closed_set = set()
        open_positions = {start: 0}

        heapq.heappush(open_list, start_node)

        while open_list:
            current_node = heapq.heappop(open_list)
            current_pos = current_node.position

            if current_pos in closed_set:
                continue

            if current_pos == end:
                path = []
                while current_node is not None:
                    path.append(current_node.position)
                    current_node = current_node.parent
                return path[::-1]

            closed_set.add(current_pos)

            for delta_row, delta_col in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
                neighbor = (current_pos[0] + delta_row, current_pos[1] + delta_col)

                if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                    continue
                if grid[neighbor[0]][neighbor[1]] != 0:
                    continue
                if neighbor in closed_set:
                    continue

                child = PathFinder(neighbor, current_node)
                child.g = current_node.g + 1
                child.h = PathFinder.heuristic(neighbor, end)
                child.f = child.g + child.h

                existing_cost = open_positions.get(neighbor)
                if existing_cost is not None and existing_cost <= child.g:
                    continue

                heapq.heappush(open_list, child)
                open_positions[neighbor] = child.g

        return None