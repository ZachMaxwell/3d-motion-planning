from enum import Enum
from queue import PriorityQueue
import numpy as np


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTHWEST = (-1, -1, np.sqrt(2)) # move in x+/-1, y+/-1, and assign cost sqrt(2)
    NORTHEAST = (-1, 1, np.sqrt(2))
    SOUTHWEST = (1, -1, np.sqrt(2))
    SOUTHEAST = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    return valid_actions


def a_star(grid, h, start, goal):

    path = [] # will be updated on each iteration and will store the final path from start to goal
    path_cost = 0 # keep track of the total cost of the path found
    '''
    Initialize a PriorityQueue to manage the nodes to explore.
    Priority Queues ensures nodes (can be grid cells or points) are explored in optimal order
    because it prioritizes the node with the lowest cost first.
    
    Total Cost: f(n) = g(n) + h(n)
        -where:
            -g(n) is the cost of the path to reach the current node from the start node
            -h(n) is the estimated cost to reach the goal node from the current node

    Priority Queue will store tuples of (cost, node) where cost is the total cost of the path to reach the node
    and node is the current node being explored.
    
    The Priority Queue will store the nodes in order of increasing total cost (lowest to highest), so nodes 
    with the lower total cost are explored first. 

    Priority Queue also ensures dyanmic reordering of nodes based on changes in the total cost of the path

    Retrieves the next best node in O(log n) time, where n is the number of nodes in the queue.
    
    '''
    queue = PriorityQueue() # See above
    queue.put((0, start)) # The starting node is added to the queue with an initial cost of 0
    visited = set(start) # Set to keep track of visited nodes

    '''
    Branch = 
        Dictionary to store information about each node:

        (315, 444): (1.0, (315, 445), <Action.WEST: (0, -1, 1)>) 
            -Key: current_node (x, y)
            -Value: (cost, parent_node, action)
                -cost: total cost of the path to reach this node
                -parent_node: the parent node from which this node was reached
                -action: the action taken to reach this node from the parent node
    '''
    branch = {}
    found = False
    
    '''
    Item = 
        Tuple of (cost, node)
            -cost: total cost of the path; start to current and current to goal
            -node: the current node being explored
    '''

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            # iterate over all possible actions at each node
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal # current node is set to goal
        path_cost = branch[n][0]
        path.append(goal) 
        while branch[n][1] != start: # while the parent node of the current node doesn't equal start
            path.append(branch[n][1]) # add the parent node of the current node to the path list
            n = branch[n][1] # update the current point to be the parent node of the current node, effectively moving backwards a step
        path.append(branch[n][1]) # at the end, add the final parent node (i.e. the start node) to the path
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    path = path[::-1]
    pruned_path = prune_path(path)
    return pruned_path, path_cost

def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

def prune_path(path):
    '''
    Param: path: List[]
    Returns: pruned_path: List[]

    Definition: function to prune the final path with a 2-D collinearity test 
    using matrix determinant method
    
    Description: 
        1. initialize with the first point in the path list
            -will be implemented on the reversed path, so path[0] is start_node
        2. iterate over the path, and check for collinearity
        3. if they aren't collinear, add the current point to the pruned_path because we need to use that point (i.e. can't prune it since it's not collinear).
        4. current_point becomes the last point in the pruned_path
        5. if they are collinear, don't add the current point to the pruned_path (i.e. skipping over it because can prune it)
        6. at the end, add the goal point to the pruned_path because we need that to finish
    ''' 

    # initialize the pruned path with the first point from path 
    pruned_path = [path[0]]
    # iterate over each point from the 2nd point to the 2nd to last point
    for i in range(1, len(path) - 1):
        # on each iteration, see if the last point in pruned_path (i.e. the last valid point) is collinear
        # with the current point being looking at in path and the next point to be looked at in the path list
        # if the points are not collinear, then add the current point in the path list to the pruned path because 
        # this point needs to be used (i.e. it cannot be pruned)
        # if the points are collinear, then skip over it and don't add it to the pruned path because the point is not needed
        if not is_collinear(pruned_path[-1], path[i], path[i + 1]):
            pruned_path.append(path[i])
    # add the last point in path (i.e. the goal position to the pruned path) because that point is always needed
    pruned_path.append(path[-1])

    return pruned_path 

def is_collinear(p1, p2, p3):
    def point(p):
        return np.array([p[0], p[1], 1.0])
    
    epsilon = 1e-2 # threshold for collinearity test
    matrix = np.vstack((point(p1), point(p2), point(p3)))
    determinant = np.linalg.det(matrix) # use matrix determinant method for determining collinearity
    return abs(determinant) < epsilon
    
    
    


    
    
