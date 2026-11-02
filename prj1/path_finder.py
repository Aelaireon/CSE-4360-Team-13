#!/usr/bin/env pybricks-micropython
import heapq

# Grid conventions (shared with the rest of the project):
#   - Planning grid is grid[ix][iy] with ix = 0 ... GRID_X_CELLS (x) and iy = 0 ... GRID_Y_CELLS (y).
#   - Neighboring grid points are CELL_M apart (CELLS_PER_TILE = 2), so start/goal/obstacle centers (tile intersections) land exactly on points.
#   - grid values: 0 = free, 1 = obstacle itself, 2 = padding around an obstacle / near a wall.
#   - Course data (start, goal, obstacle centers) is in METERS; convert() maps it to specific cells.
TILE_M = 0.305      # 1 floor tile = 0.305 m = 1 ft (rounded from 0.3048)
GRID_X_TILES = 16   # 4.88 m workspace width
GRID_Y_TILES = 10   # 3.05 m workspace height
GOAL_RADIUS_TILES = 1  # Goal circle radius is 0.305 m (1 tile)
CELLS_PER_TILE = 2  # grid points per tile in each direction (2 = tile split into 4 quadrants). Must be even.
CELL_M = TILE_M / CELLS_PER_TILE  # distance between neighboring grid points in meters
GRID_X_CELLS = GRID_X_TILES * CELLS_PER_TILE  # largest ix (the right wall)
GRID_Y_CELLS = GRID_Y_TILES * CELLS_PER_TILE  # largest iy (the top wall)
OBSTACLE_PADDING_CELLS = 1  # extra blocked CELLS (not tiles) around each obstacle and along the walls

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
    A* on a 4-connected grid (grid[ix][iy], 0 = free, anything else = blocked).
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
        if heuristic(current_pos, end_node.position) <= GOAL_RADIUS_TILES * CELLS_PER_TILE:
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
    ix = int(round(x_m / CELL_M))
    iy = int(round(y_m / CELL_M))
    if not (0 <= ix <= GRID_X_CELLS and 0 <= iy <= GRID_Y_CELLS):
        raise ValueError("(%.3f, %.3f) m is outside the %d x %d tile grid" % (x_m, y_m, GRID_X_TILES, GRID_Y_TILES))
    return (ix, iy)

# preprocessing function
def convert(obstacle, start, end, padding=OBSTACLE_PADDING_CELLS):
    """
    Converts course data from meters to flat integer numbers
    Returns (grid, start_cell, end_cell) where grid[ix][iy] is 1 for an obstacle, 2 for the
    padding around it (and along the walls), else 0. padding is in cells (CELL_M each).
    """
    # 0 initializing
    grid = [[0 for _ in range(GRID_Y_CELLS + 1)] for _ in range(GRID_X_CELLS + 1)]

    # an obstacle covers half a tile (half cells) to each side of its center
    half = CELLS_PER_TILE // 2

    # keeping the same clearance from the walls as from obstacles
    wall = half + padding
    for ix in range(GRID_X_CELLS + 1):
        for iy in range(GRID_Y_CELLS + 1):
            if ix < wall or ix > GRID_X_CELLS - wall or iy < wall or iy > GRID_Y_CELLS - wall:
                grid[ix][iy] = 2

    # marking the obstacle itself as 1 and the padding ring around it as 2
    for ox, oy in obstacle:
        cx, cy = to_cell(ox, oy)
        for dx in range(-(half + padding), half + padding + 1):
            for dy in range(-(half + padding), half + padding + 1):
                ix, iy = cx + dx, cy + dy
                if 0 <= ix <= GRID_X_CELLS and 0 <= iy <= GRID_Y_CELLS:
                    if abs(dx) <= half and abs(dy) <= half:
                        grid[ix][iy] = 1
                    elif grid[ix][iy] == 0:
                        grid[ix][iy] = 2

    # converting start and end points to integer grid cells
    start_cell = to_cell(start[0], start[1])
    end_cell = to_cell(end[0], end[1])

    return grid, start_cell, end_cell


def find_waypoints(start, goal, obstacle):
    """
    Orchestrator function for main.py:
    1. Converts meters to cells and runs A*, first with the full obstacle padding. If that
       finds no path, it retries with less padding, down to none.
    2. Adds a final leg from where A* stopped to the goal center, then runs a shortcut pass: cells are
       dropped whenever a straight line over free grid points can skip them (this also merges straight
       runs and creates diagonals), so the robot makes fewer stops and turns.
    3. Converts the remaining cells back to meters (excluding start, locking final to exact goal).
    Returns (waypoints, grid, cells):
        waypoints: list of (x_m, y_m) tuples, or None if no path found
        grid:      the grid the path was planned on (for format_map)
        cells:     the full A* path as (ix, iy) cells, or None
    """
    cells = None
    for padding in range(OBSTACLE_PADDING_CELLS, -1, -1):
        grid, start_cell, goal_cell = convert(obstacle, start, goal, padding)
        cells = astar(grid, start_cell, goal_cell)
        if cells is not None:
            break

    if cells is None:
        return None, grid, None

    # A* stops once it is inside the goal circle, so finish with a straight leg to the goal itself
    route = cells
    if route[-1] != goal_cell:
        route = cells + [goal_cell]

    # shortcut pass: from each kept cell, jump to the farthest later cell that can be reached in a
    # straight line over free grid points, dropping the cells in between
    corners = route[:1]
    i = 0
    while i < len(route) - 1:
        j = len(route) - 1
        while j > i + 1:
            ax, ay = route[i]
            bx, by = route[j]
            steps = 4 * max(abs(bx - ax), abs(by - ay))
            # with reduced padding (fallback) only straight axis-aligned shortcuts are allowed,
            # so the robot keeps the clearance of the grid line instead of cutting corners
            clear = padding > 0 or ax == bx or ay == by
            for k in range(1, steps if clear else 1):
                px = int(round(ax + (bx - ax) * k / steps))
                py = int(round(ay + (by - ay) * k / steps))
                if (px, py) != route[i] and (px, py) != route[j] and grid[px][py] != 0:
                    clear = False
                    break
            if clear:
                break
            j -= 1
        corners.append(route[j])
        i = j

    # post processing formatting for path_follower
    # convert cells to meter waypoints, skipping the starting cell
    waypoints = [(ix * CELL_M, iy * CELL_M) for (ix, iy) in corners[1:]]

    if waypoints:
        waypoints[-1] = goal

    return waypoints, grid, cells

# a good way to visualize the grid and solved path
def format_map(grid, path, start_cell, end_cell):
    """Text picture of the grid, y increasing upward: # obstacle, + padding, * path, S start, G goal."""
    on_path = set(path) if path else set()
    lines = []
    for iy in range(len(grid[0]) - 1, -1, -1):
        row = ""
        for ix in range(len(grid)):
            if (ix, iy) == start_cell:
                row += "S "
            elif heuristic((ix, iy), end_cell) <= GOAL_RADIUS_TILES * CELLS_PER_TILE:
                row += "G "
            elif grid[ix][iy] == 1:
                row += "# "
            elif grid[ix][iy] == 2:
                row += "+ "
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

    waypoints, grid, cells = find_waypoints(start, goal, obstacle)
    print(format_map(grid, cells, to_cell(start[0], start[1]), to_cell(goal[0], goal[1])))
    print("Waypoints (meters):", waypoints)
