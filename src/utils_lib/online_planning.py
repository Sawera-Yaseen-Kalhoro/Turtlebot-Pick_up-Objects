import numpy as np
import copy
import math
import random
import time

def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.1, is_unknown_valid=True):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        # map sampling resolution (size of a cell))                            
        self.resolution = None
        # world position of cell (0, 0) in self.map                      
        self.origin = None
        # set method has been called                          
        self.there_is_map = False
        # radius arround the robot used to check occupancy of a given position                 
        self.distance = distance                    
        # if True, unknown space is considered valid
        self.is_unknown_valid = is_unknown_valid    
    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin):
        self.map = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
    
    # Given a pose, returs true if the pose is not in collision and false othewise.
    def is_valid(self, pose): 

        # TODO: convert world robot position to map coordinates using method __position_to_map__
        # TODO: check occupancy of the vicinity of a robot position (indicated by self.distance atribute). 
        # Return True if free, False if occupied and self.is_unknown_valid if unknown. 
        # If checked position is outside the map bounds consider it as unknown.
        # Create a copy of the map to avoid modifying the original map
        
        map_copy = copy.deepcopy(self.map)

        # Calculate the range around the pose to check
        lower_limit = tuple(i - self.distance for i in pose)
        upper_limit = tuple(i + self.distance for i in pose)

        # loop for all the surrounding pixels
        valid = True # initialize bool variable for output

        for x in np.arange(lower_limit[0],upper_limit[0],self.resolution):
            for y in np.arange(lower_limit[1],upper_limit[1],self.resolution):
                # convert coordinate into map index
                cell = self.__position_to_map__((x,y))

                # check if this cell is outside the map (considered unknown)
                if cell is None:
                    if self.is_unknown_valid == False: # unknown is considered False
                        valid = False
                        break

                else: # if it is inside the map
                    # check if the cell is an obstacle
                    is_obstacle = map_copy[cell] == 100
                    # check if the cell is unknown
                    is_unknown = map_copy[cell] == -1

                    # combine the conditions for False
                    if is_obstacle or (is_unknown and self.is_unknown_valid == False):
                        valid = False
                        break
        
        return valid

    # Given a path, returs true if the path is not in collision and false othewise.
    def check_path(self, path):

        # TODO: Discretize the positions between 2 waypoints with an step_size = 2*self.distance
        # TODO: for each point check if `is_valid``. If only one element is not valid return False, otherwise True.

        step_size = self.distance
    
        # Discretization of the position between 2 waypoints
        for i in range(len(path)-1):
            current_point = path[i]
            next_point = path[i + 1]
            
            # Direction and distance between current and next points
            direction = (next_point[0] - current_point[0], next_point[1] - current_point[1]) # By vector subtraction
            distance = math.sqrt(direction[0] ** 2 + direction[1] ** 2) # Euclidean Distance
            
            # Number of steps needed between current and next points
            num_steps = int(distance / step_size)
            
            # Discretization of the path
            for j in range(num_steps):
                x = current_point[0] + j * step_size * direction[0] / distance
                y = current_point[1] + j * step_size * direction[1] / distance
                # Checking Validity of each point
                if not self.is_valid((x, y)):
                    return False
        
        return True

    # Transform position with respect the map origin to cell coordinates
    def __position_to_map__(self, p):

        # TODO: convert world position to map coordinates. 
        
        # Convert position from world frame to grid map frame
        cell_x = int((p[0] - self.origin[0]) / self.resolution)
        cell_y = int((p[1] - self.origin[1]) / self.resolution)

        # Check if the computed cell index is within the grid map boundaries
        if 0 <= cell_x < self.map.shape[0] and 0 <= cell_y < self.map.shape[1]:
            return cell_x, cell_y
        else:
            return None
    
# Define Planner class (you can take code from Autonopmous Systems course!)
class Planner:
    def  __init__(self, state_validity_checker, max_time = 5.0, delta_q=2, p_goal=0.3, dominion=[-10, 10, -10, 10]):
        # define constructor
        self.state_validity_checker = state_validity_checker
        self.max_time = max_time
        self.delta_q = delta_q
        self.p_goal = p_goal
        self.dominion = dominion
    
    class Graph: # imported from AS lab
        def __init__(self, qstart):
            self.points = [qstart]
            self.branches = []
        
        def add_point(self,qnew):
            self.points.append(qnew)
            return

        def add_branch(self,qnear,qnew):
            self.branches.append((qnear,qnew))
            return
    
    def compute_path(self, q_start, q_goal):
        # Implement RRT algorithm.
        # Use the state_validity_checker object to see if a position is valid or not.

        G = self.Graph(q_start)
        start_time = time.time()
        while time.time() < start_time + self.max_time:
            qrand = self.rand_conf(self.state_validity_checker, self.p_goal, q_goal)
            qnear = self.nearest_vertex(qrand, G)
            qnew = self.new_conf(qnear, qrand, self.delta_q)
            if self.state_validity_checker.check_path([qnear,qnew]):
                G.add_point(qnew)
                G.add_branch(qnear, qnew)
                if qnew == q_goal:
                    path = self.fill_path(G)
                    path = self.smooth_path(path,self.state_validity_checker)

                    # convert to lists
                    path = [list(point) for point in path]

                    return path
                
        return "No solution found"

    def smooth_path(self, path, checker):

        last_v = len(path) - 1
        smooth_path = path

        while last_v > 0:           
            start = 0
            while start < last_v - 1 and not checker.check_path([smooth_path[start],smooth_path[last_v]]):                
                start += 1

            if start < last_v - 1:
                # Shorter path found
                smooth_path = smooth_path[:start+1] + smooth_path[last_v:]
                last_v = start
            else:
                last_v -= 1
        
        return smooth_path

    
    def rand_conf(self,checker, p, qgoal):
        """
            generates a random configuration qrand with probability (1 - p) 
            or generates qrand = qgoal with probability p.
        """
        n = np.random.rand() # generate a random number from 0-1

        if n <= p: # if the number is less than p, then it's the probability that qrand = qgoal
            qrand = qgoal
        else:
            # pick a random point inside the dominion
            qrand_x = random.uniform(self.dominion[0],self.dominion[1])
            qrand_y = random.uniform(self.dominion[2],self.dominion[3])
            qrand = (qrand_x,qrand_y)

            # check the validity of qrand; if it's not valid, keep picking a random point
            while not checker.is_valid(qrand):
                qrand_x = random.uniform(self.dominion[0],self.dominion[1])
                qrand_y = random.uniform(self.dominion[2],self.dominion[3])
                qrand = (qrand_x,qrand_y)

        return qrand


    def nearest_vertex(self, qrand, G):
        """
            runs through all vertices v in graph G, calculates the distance between 
            qrand and v using some measurement function (in our case the Euclidean distance) and
            returns the nearest vertex.
        """
        min_distance = 1000 # initialize as a very big number

        # loop through all points in the graph
        for point in G.points:
            # compute Euclidean distance
            dist = np.linalg.norm([qrand[0]-point[0],qrand[1]-point[1]])

            # store nearest point
            if dist < min_distance:
                min_distance = dist
                nearest_point = point
        
        return nearest_point


    def new_conf(self, qnear, qrand, Deltaq):
        """
            selects a new configuration qnew by moving an incremental distance Deltaq 
            from qnear in the direction of qrand without overshooting qrand.
        """
        # first check the distance between qnear and qrand
        dist = np.linalg.norm([qrand[0]-qnear[0],qrand[1]-qnear[1]])

        if Deltaq >= dist: # if Deltaq overshoot qrand
            qnew = qrand # qnew is the qrand
        else:
            # use trig properties to set qnew
            theta = math.atan2(qrand[1]-qnear[1],qrand[0]-qnear[0])
            dx = Deltaq*math.cos(theta)
            dy = Deltaq*math.sin(theta)

            qnew = (qnear[0]+dx, qnear[1]+dy)
        
        return qnew


    def fill_path(self,G):
        """
            Moves form the qgoal (last vertex in the graph G) to the qstart (initial vertex
            in G) and computes the path. 
            This function returns both the graph G (if you prefer as a list of 2D
            vertices and a list of edges containing the indexes to these vertices) and
            the path as a list of indexes to vertices.
        """
        branch_list = []
        T = [i for i in G.branches]
        start = G.points[0]

        while(True):
            if not branch_list :
                branch_list.append(T[-1])

            last_branch = T.pop()
            if (bool(T) is False or (last_branch[0] == start and last_branch[1] == branch_list[0][0])):
                branch_list.insert(0, last_branch)
                break
            if (branch_list[0][0] == last_branch[1]):
                branch_list.insert(0, last_branch)
        path = [i[0] for i in branch_list]
        path.append(branch_list[-1][1])
        return path


    def path_Euclidian_distance(self,path):
        """
            Calculates the Euclidian distance between each vertex of the given path and
            returns the sum of them.
        """
        from math import sqrt
        distance = 0
        for i in range(len(path)-1):
            distance += sqrt((path[i][0]-path[i+1][0])**2 +
                            (path[i][1]-path[i+1][1])**2)
        return distance


# Planner: This function has to plan a path from start_p to goal_p. To check if a position is valid the 
# StateValidityChecker class has to be used. The planning dominion must be specified as well as the maximum planning time.
# The planner returns a path that is a list of poses ([x, y]).
def compute_path(start_p, goal_p, state_validity_checker, bounds, max_time=1.0):
    
    # Creating a planner instance
    planner = Planner(state_validity_checker,max_time=max_time,dominion=bounds)

    # Converting start and goal points to tuples (To ensure consistency in data types)
    start = tuple(start_p)
    goal = tuple(goal_p)

    # Computing path using RRT algorithm only if the goal position is within the bounds.
    if (bounds[0] <= goal[0] <= bounds[1]) and (bounds[2] <= goal[1] <= bounds[3]):
        path = planner.compute_path(start, goal)
    else:
        path = "No solution found"

    # If a valid path is found, return it.
    if path != "No solution found":
        return path
    else:
        return []


# Controller: Given the current position and the goal position, this function computes the desired 
# lineal velocity and angular velocity to be applied in order to reah the goal.
def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    
    # Differences in position and orientation of goal and current points
    dx = goal[0] - current[0]
    dy = goal[1] - current[1]
    distance = np.sqrt(dx**2 + dy**2) # Pythagorus theorem
    
    # Desired angle (theta) towards the goal
    desired_theta = np.arctan2(dy, dx)
    
    # Orientation Correction (Angular Velocity)
    theta_error = wrap_angle(desired_theta - current[2])
    w = Kw * theta_error
    
    # Distance Correction (Linear Velocity)
    # Only move forward if the robot is already facing the goal (or close to it)
    if abs(w) < 0.05:
        v = Kv * distance
    else:
        v = 0

    return v, w
