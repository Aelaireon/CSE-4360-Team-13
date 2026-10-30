#!/usr/bin/env pybricks-micropython
import heapq

# Grid conventions (shared with the rest of the project):
#   - Planning grid is 1 tile per cell: grid[ix][iy], ix = 0..15 (x), iy = 0..9 (y).
#   - Course data (start, goal, obstacle centers) is in METERS; convert() maps it to cells.
TILE_M = 0.305      # 1 floor tile = 0.305 m (4.88m / 16 = 0.305, 3.05m / 10 = 0.305)
GRID_X_TILES = 16   # 4.88 m workspace width
GRID_Y_TILES = 10   # 3.05 m workspace height
GOAL_RADIUS_TILES = 1  # Goal circle radius is 0.305 m (1 tile)


class Node:
    def __init__(self, position, parent=None):
        self.position = position #(row, col)
        self.parent = parent

        self.g = 0
        self.h = 0
        self.f = 0 #g+h

    def __lt__(self, other):
        return self.f < other.f

def heuristic(current, goal):
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

def astar(grid, start, end):
    """
    A* on a 4-connected grid (grid[ix][iy], 0 = free, 1 = blocked).
    start and end are (ix, iy) cells. Returns the list of (ix, iy) tuples from start to
    end inclusive, or None if there is no path.
    """
    start = tuple(start)
    end = tuple(end)
    start_node = Node(start)
    end_node = Node(end)

    open_list = []
    closed_set = set()

    heapq.heappush(open_list, start_node)
    open_positions = {start: 0}

    rows, cols = len(grid), len(grid[0])
    #children nodes
    adjacent_movements = [(0, -1), (0, 1), (-1, 0), (1, 0)]

    while open_list:
        current_node = heapq.heappop(open_list)
        current_pos = current_node.position

        # Stale heap entry check
        if current_pos in closed_set:
            continue

        if current_pos in open_positions and open_positions[current_pos] == current_node.g:
            del open_positions[current_pos]

        #check if found goal
        if heuristic(current_pos, end_node.position) <= GOAL_RADIUS_TILES:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  #return reversed path

        closed_set.add(current_pos)

        for new_position in adjacent_movements:
            node_position = (current_pos[0] + new_position[0], current_pos[1] + new_position[1])

            #range checker
            if not (0 <= node_position[0] < rows and 0 <= node_position[1] < cols):
                continue

            #traversable terrain checker
            if grid[node_position[0]][node_position[1]] != 0:
                continue

            #if already evaluated
            if node_position in closed_set:
                continue

            #child node creation
            child = Node(node_position, current_node)
            child.g = current_node.g + 1
            child.h = heuristic(child.position, end_node.position)
            child.f = child.g + child.h

            #node exception
            if node_position in open_positions and open_positions[node_position] <= child.g:
                continue

            #add child to open list
            heapq.heappush(open_list, child)
            open_positions[node_position] = child.g

    return None


def to_cell(x_m, y_m):
    """Meters -> (ix, iy) grid cell tuple. Raises ValueError if outside workspace."""
    # 0.305
    ix = int(round(x_m / TILE_M))
    iy = int(round(y_m / TILE_M))
    if not (0 <= ix < GRID_X_TILES and 0 <= iy < GRID_Y_TILES):
        raise ValueError("(%.3f, %.3f) m is outside the %d x %d tile grid" % (x_m, y_m, GRID_X_TILES, GRID_Y_TILES))
    return (ix, iy)

def convert(obstacle, start, end):
    """
    Converts course data from meters to flat integer numbers
    Returns (grid, start_cell, end_cell) where grid[ix][iy] is 1 for an obstacle, else 0.
    """
    # 0 initializing
    grid = [[0 for _ in range(GRID_Y_TILES)] for _ in range(GRID_X_TILES)]

    # changing to 1 if obstacle
    for ox, oy in obstacle:
        ix, iy = to_cell(ox, oy)
        grid[ix][iy] = 1

    # converting start and end points to integer grid cells
    start_cell = to_cell(start[0], start[1])
    end_cell = to_cell(end[0], end[1])

    return grid, start_cell, end_cell


def find_waypoints(start, goal, obstacle):
    """
    Orchestrator function for main.py:
    1. Converts meters to cells and runs A*.
    2. Converts path cells back to meters (excluding start, locking final to exact goal).
    Returns a list of (x_m, y_m) waypoint tuples, or None if no path found.
    """
    grid, start_cell, goal_cell = convert(obstacle, start, goal)
    cells = astar(grid, start_cell, goal_cell)
    
    # Convert cells to meter waypoints, skipping the starting cell
    waypoints = [(ix * 0.305, iy * 0.305) for (ix, iy) in cells[1:]]

    if waypoints:
        waypoints[-1] = goal
        
    return waypoints

# not used rn, but honestly a good way to visualize the grid and solved path
def format_map(grid, path, start_cell, end_cell):
    """Text picture of the grid, y increasing upward: # obstacle, * path, S start, G goal."""
    on_path = set(path) if path else set()
    lines = []
    for iy in range(len(grid[0]) - 1, -1, -1):
        row = ""
        for ix in range(len(grid)):
            if (ix, iy) == start_cell:
                row += "S "
            elif heuristic((ix, iy), end_cell) <= GOAL_RADIUS_TILES:
                row += "G "
            elif grid[ix][iy] == 1:
                row += "# "
            elif (ix, iy) in on_path:
                row += "* "
            else:
                row += ". "
        lines.append("%2d  %s" % (iy, row))
    lines.append("    " + "".join("%-2d" % (ix % 10) for ix in range(len(grid))))
    return "\n".join(lines)


if __name__ == '__main__':
    obstacle = [
        (0.61, 2.743),
        (0.915, 2.743),
        (1.219, 2.743),
        (1.829, 1.219),
        (1.829, 1.524),
        (1.829, 1.829),
        (1.829, 2.134),
        (2.743, 0.305),
        (2.743, 0.61),
        (2.743, 0.915),
        (2.743, 2.743),
        (3.048, 2.743),
        (3.353, 2.743),
    ]

    start = (0.305, 1.219)  # start location (meters)
    goal = (3.658, 1.829)   # goal center location (meters)

    waypoints = find_waypoints(start, goal, obstacle)
    print("Waypoints (meters):", waypoints)