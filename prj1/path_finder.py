#!/usr/bin/env pybricks-micropython
import math
from path_follower import follow_path

class PathFinder:
    def __init__(self, start, obstacles, goal):
        """
        Initializes the pathfinder with starting coordinates, obstacles, and goal coordinates.
        """
        # list size 2 {x, y}. makes a square
        self.start = start
        # list of obstacles, each represented as a list of size 2 {x, y}
        self.obstacles = obstacles
        # list size 2 {x, y}. makes a circle of diameter 0.305m
        self.goal = goal

    """
    Continue with the actual search algorithm implementation (possibly A*)
    Use path_follower.follow_path() to navigate through the waypoints generated.
    """