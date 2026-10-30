#!/usr/bin/env pybricks-micropython
import heapq
import numpy

class Node:
    def __init__(self, position, parent):
        self.position = position #(row, col)
        self.parent = parent

        self.g = 0
        self.h = 0
        self.f = 0 #g+h

    def __It__(self, other):
        return self.f < other.f

def heuristic(current, goal):
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

def astar(grid, start, end):
    start_node = Node(start)
    end_node = Node(end)

    open_list = []
    closed_set = set()

    heapq.heappush(open_list, start_node)

    open_positions = {start: 0}

    rows, cols = len(grid), len(grid[0])

    while open_list:
        current_node = heapq.heappop(open_list)
        current_pos = current_node.position

        if current_pos in open_positions and open_positions[current_pos] == current_node.g:
            del open_positions[current_pos]

        #found goal
        if current_pos == end_node.position:
            path = []
            current = current_node    
            while current_node is not None:
                path.append(current.position)
                current_path = current.parent
            return path[::-1] #return reversed path

    closed_set.add(current_pos)

    #children nodes
    adjacent_movements = [(0,-1),(0,1),(-1,0),(1,0)]

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

def convert(obstacle, start, end):
    grid = [[0 for _ in range(10)] for _ in range(16)]
    start = round(start/0.305)
    end = round(end/0.305)
    obstacle = round(obstacle/0.305)
    for x in grid:
        for y in grid:
            if(x == obstacle[x][y] & y == obstacle[x][y] & obstacle[x][y] > 0):
                grid[x][y] = 1
            else:
                grid[x][y] = 0
    return grid, start, end

if __name__ == '__main__':
    MAX_OBSTACLES = 25  # maximum number of obstacles
    num_obstacles = 13  # number of obstacles

    # obstacle locations
    obstacle = [
        [0.61, 2.743],
        [0.915, 2.743],
        [1.219, 2.743],
        [1.829, 1.219],
        [1.829, 1.524],
        [1.829, 1.829],
        [1.829, 2.134],
        [2.743, 0.305],
        [2.743, 0.61],
        [2.743, 0.915],
        [2.743, 2.743],
        [3.048, 2.743],
        [3.353, 2.743],
        [-1, -1],
        [-1, -1],
        [-1, -1],
        [-1, -1],
        [-1, -1],
        [-1, -1],
        [-1, -1],
        [-1, -1],
        [-1, -1],
        [-1, -1],
        [-1, -1],
        [-1, -1],
    ]

    start = [0.305, 1.219]  # start location

    goal = [3.658, 1.829]  # goal location

    #0=empty 1=wall
    # maze = [
    #     [0,0,0,0,1,0],
    #     [1,1,0,1,1,0],
    #     [0,0,0,0,0,0],
    #     [0,1,1,1,1,0],
    #     [0,0,0,0,0,0]
    # ]
    # start_point = (0,0)
    # end_point = (4,5)


    shortest_path = astar(convert(obstacle, start, goal))
    print("Path found:", shortest_path)


# =============================================================================
# PSEUDOCODE / IMPLEMENTATION PLAN  (comments only - nothing below is executed)
# Legend:  [EXISTING] = in the code above     [PROPOSED] = does not exist yet
# =============================================================================
#
# RESPONSIBILITY
#   Turn the compile-time course (start, goal, obstacle centers, all in METERS,
#   defined in main.py) into an ordered list of (x, y) waypoints in METERS that
#   PathFollower.follow_path() can consume directly. Project constraints (PDF):
#   never enter an obstacle tile, stay inside the 4.88 x 3.05 m (16 x 10 tile)
#   workspace, find a path whenever one exists (a >= 0.61 m / 2-tile-wide path
#   is guaranteed). Speed and path length are not important.
#
# CURRENT STATUS OF THE CODE ABOVE
#   It compiles (with the grid line fix) but does not run yet. Errors, in the
#   order they would be hit, with suggested fixes (NOT applied):
#   1. convert(): `start / 0.305` divides a list by a float (TypeError - this is
#      the first error when the file is run). round() must be applied per
#      coordinate. The nested loops iterate over grid rows rather than over
#      indices, and `&` binds tighter than `==`, so the obstacle test is wrong.
#      Suggested rewrite (TILE_M = 0.3048 keeps it consistent with the rest of
#      the project; 0.305 also works for values up to 16 tiles):
#          def convert(obstacle, start, end):
#              grid = [[0 for _ in range(10)] for _ in range(16)]   # grid[ix][iy]
#              for ox, oy in obstacle:
#                  if ox < 0 or oy < 0: continue                    # skip (-1, -1) padding
#                  grid[round(ox / TILE_M)][round(oy / TILE_M)] = 1
#              s = (round(start[0] / TILE_M), round(start[1] / TILE_M))
#              g = (round(end[0] / TILE_M), round(end[1] / TILE_M))
#              return grid, s, g
#   2. __main__ calls astar(convert(...)), but convert returns a 3-tuple:
#      use astar(*convert(obstacle, start, goal)).
#   3. Node.__init__(self, position, parent) requires `parent`, but astar calls
#      Node(start) and Node(end). Use parent=None as the default.
#   4. `__It__` (capital i) should be `__lt__`; heapq needs it to order nodes.
#   5. The path rebuild never terminates: `current_path = current.parent` never
#      updates `current`, and the loop tests current_node. Use:
#          while current is not None:
#              path.append(current.position); current = current.parent
#   6. `closed_set.add(current_pos)` and the `for new_position ...` neighbor loop
#      are dedented outside `while open_list:`, so the search never expands.
#      Indent both one level so they run for every popped node.
#   7. Skip nodes popped again after being closed (stale heap entries):
#      `if current_pos in closed_set: continue` right after the pop.
#   8. `import numpy` is unused and is likely not available in the EV3's
#      MicroPython. Remove it so this file can be imported on the robot.
#   Checked: with fixes 1-8 applied to a scratch copy, the example course (start
#   cell (1, 4), goal cell (12, 6)) gives a 16-cell path that avoids every
#   obstacle cell and returns None when the goal is walled in. That was run on a
#   PC, not on the robot.
#
# GRID CONVENTIONS (follows from the grid line above)
#   grid[ix][iy]: ix = 0..15 is the x tile index, iy = 0..9 the y tile index, so
#   astar positions are (ix, iy). The `#(row, col)` comments mean (x, y) here.
#   Index 0 is the wall line; obstacle, start and goal centers fall on
#   intersections 1..15 (x) and 1..9 (y).
#   astar returns grid indices, NOT meters; PathFollower needs meters.
#
# OPEN DESIGN DECISION - CLEARANCE (grid resolution)
#   With 1-tile cells, the example path runs along the cells directly next to
#   obstacle cells, e.g. (5,3) -> (6,3) -> (7,3) beside the obstacle at (6,4).
#   That is only 0.5 tile (0.15 m) from the obstacle edge. For a robot about 1/3
#   tile wide and a bit over 1/2 tile long, that leaves roughly 6 cm of slack
#   for odometry drift. Padding obstacles by one 1-tile cell would close the
#   guaranteed 2-tile corridors (their centerline is a half-tile position), so
#   the safer option is a half-tile grid:
#       CELL_M = TILE_M / 2        # 0.1524 m
#       grid is 33 x 21 (ix 0..32, iy 0..20); cell = round(meters / CELL_M)
#       obstacle footprint = cells with |dx| <= 1 and |dy| <= 1 from its center
#       padding k extra cells (try k = 1 first, fall back to k = 0 if no path)
#       cells within (1 + k) of the workspace walls are blocked too
#   Only convert() and the meters conversion change; astar() works for any grid.
#   Needs a team decision. Keep the 1-tile grid if you prefer simplicity.
#
# [PROPOSED] cell_to_meters(ix, iy) -> (x_m, y_m)
#   return (ix * CELL_M, iy * CELL_M)      # CELL_M = TILE_M for the 1-tile grid
#
# [PROPOSED] find_waypoints(start, goal, obstacle) -> list of (x_m, y_m) or None
#   INPUTS : start, goal = (x, y) meters; obstacle = list of (x, y) meters
#   OUTPUT : ordered waypoints in METERS, excluding the start (the robot is
#            already there) and ending exactly at goal; [] if start == goal;
#            None if there is no path
#   steps:
#     1. Validate that start and goal lie inside the workspace (else ValueError)
#        and that neither lies inside an obstacle footprint (else None).
#     2. grid, s, g = convert(obstacle, start, goal); force s and g to be free.
#        (With padding, retry with less padding if astar returns None.)
#     3. cells = astar(grid, s, g); if cells is None: return None
#     4. Optional later QOL: merge collinear cells so the follower makes fewer
#        stops and turns (belongs here, not in path_follower.py).
#     5. waypoints = [cell_to_meters(ix, iy) for (ix, iy) in cells[1:]]
#        waypoints[-1] = goal               # land on the real goal, not a rounded cell
#     6. return waypoints
#   COMMUNICATION: main.py passes the result unchanged to
#   PathFollower.follow_path(), which rejects out-of-workspace waypoints as a
#   second safety check. This file should not import path_follower or pybricks,
#   so it stays testable on a PC.
#
# EDGE CASES TO TEST
#   start == goal; goal walled in (None); a minimum-width corridor is the only
#   way through; obstacle touching a wall; start or goal inside a padded zone;
#   the (-1, -1) padding entries; 0.305-rounded values (0.305, 1.219, 2.743,
#   3.658) landing in the right cell; ix/iy not swapped.
