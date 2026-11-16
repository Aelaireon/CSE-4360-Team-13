import heapq
import numpy

class Node:
    def __init__(self, position, parent):
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
    start_node = Node(tuple(start), None)
    end_node = Node(tuple(end), None)

    open_list = []
    closed_set = set()

    heapq.heappush(open_list, start_node)

    open_positions = {tuple(start): 0}

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
            while current is not None:
                path.append(current.position)
                current = current.parent
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
    grid = [[0 for _ in range(16)] for _ in range(10)]
    start = [round(start[1]/0.305), round(start[0]/0.305)]
    end = [round(end[1]/0.305), round(end[0]/0.305)]

    # obstacle = round(obstacle/0.305)
    for obj in obstacle:
        obj[0] = round(obj[0]/0.305)
        obj[1] = round(obj[1]/0.305)

    for obj in obstacle:
        x = obj[0]
        y = obj[1]

        if 0 <= x < len(grid[0]) and 0 <= y < len(grid):
            grid[y][x] = 1

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

    grid, start, end = convert(obstacle, start, goal)
    print("Start:", start)
    print("Goal:", end)
    print("Start grid value:", grid[start[0]][start[1]])
    print("Goal grid value:", grid[end[0]][end[1]])
    for row in grid:
        print(row)
    shortest_path = astar(grid, start, end)
    print("Path found:", shortest_path)
    
    if shortest_path:
        print("Path length: ", len(shortest_path) - 1)
    else:
        print("No path found")