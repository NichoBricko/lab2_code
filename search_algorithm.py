import numpy as np
import math
import heapq

moved_cost = 0
# Priority Queue based on heapq
class PriorityQueue:
    def __init__(self):
        self.elements = []
    def isEmpty(self):
        return len(self.elements) == 0
    def add(self, item, priority):
        heapq.heappush(self.elements,(priority,item))
    def remove(self):
        return heapq.heappop(self.elements)[1]
    
def get_neighbors(current):
    # Assuming current is a tuple (x, y) representing the current node's position
    x, y = current

    # List of possible moves (up, down, left, right)
    neighbors = [
        (x - 1, y),  # Up
        (x + 1, y),  # Down
        (x, y - 1),  # Left
        (x, y + 1),  # Right
    ]

    # Filter out invalid neighbors (out of bounds or blocked)
    valid_neighbors = []
    for nx, ny in neighbors:
        # Check if the neighbor is within bounds of the map and not blocked (1 = free square, -1 = obstacle)
        if 0 <= nx < len(map) and 0 <= ny < len(map[0]) and map[nx][ny] != -1:
            valid_neighbors.append((nx, ny))

    return valid_neighbors

def cost_function(moving_cost):
    return moved_cost + moving_cost

# An example of search algorithm, feel free to modify and implement the missing part
def search(map, start, goal):

    # cost moving to another cell
    moving_cost = 1

    # open list
    frontier = PriorityQueue()
    # add starting cell to open list
    frontier.add(start, 0)

    # path taken
    came_from = {}

    # expanded list with cost value for each cell
    cost = {}

    # init. starting node
    start.parent = None
    start.g = 0

    # if there is still nodes to open
    while not frontier.isEmpty():
        current = frontier.remove()

        # check if the goal is reached
        if current == goal:
            break

        # for each neighbour of the current cell
        # Implement get_neighbors function (return nodes to expand next)
        # (make sure you avoid repetitions!)
        for next in get_neighbors(current):

            # compute cost to reach next cell
            # Implement cost function
            cost = cost_function(moving_cost)

            # add next cell to open list
            frontier.add(next, cost)
            
            # add to path
            came_from[next] = current

    return came_from, cost
