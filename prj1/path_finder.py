#!/usr/bin/env pybricks-micropython
from path_follower import PathFollower
import heapq

class PathFinder:
    def __init__(self, position):
        self.position = position #(row, col)

        self.g = 0
        self.h = 0
        self.f = 0 #g+h

    def __It__(self, other):
        return self.f < other.f

    def heuristic(current, goal):
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

    def astar(grid, start, end):
        start_node = PathFinder(start)
        end_node = PathFinder(end)

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
            child = PathFinder(node_position, current_node)
            child.g = current_node.g + 1
            child.h = child.heuristic(child.position, end_node.position)
            child.f = child.g + child.h

            #node exception
            if node_position in open_positions and open_positions[node_position] <= child.g:
                continue

            #add child to open list
            heapq.heappush(open_list, child)
            open_positions[node_position] = child.g

        return None