from operator import itemgetter
import random
from tkinter import X
from tkinter.messagebox import NO
from turtle import st
import numpy as np

from rtree import index
from shapely.geometry import Polygon, Point


class RRTBase(object):

    def __init__(self, agent):

        '''
        Template RRT planner
        '''
        
        self.agent_ = agent
        self.X_ = agent.X_ # Search Space (SearchSpace)
        self.samples_taken_ = 0 # (int)
        self.max_samples_ = agent.max_samples_ # Max number of samples to take (int)
        self.Q_ = agent.Q_ # List of lengths of edges added to tree
        self.r_ = agent.r_ # Resolution of points to sample along edge when cheking for collision
        self.prc_ = agent.prc_ # Probability of checking whether there is a solution
        self.x_init_ = (agent.position_.x, agent.position_.y)
        self.x_goal_ = (agent.goal_.x, agent.goal_.y)
        self.trees_ = [] # [Tree]
        self.add_tree() # Add initial tree

    def add_tree(self):

        '''
        Create an empty tree and add to trees
        '''

        self.trees_.append(Tree(self.X_))

    def add_vertex(self, tree, v):

        '''
        Add vertex to corresponding tree

        Parameters:
            tree (int): tree to which to add vertex
            v (tuple): vertex to add
        '''

        self.trees_[tree].V_.insert(0, v + v, v)
        self.trees_[tree].V_count_ += 1 # Increment number of vertices in tree 
        self.samples_taken_ += 1 # Increment number of samples taken

    def add_edge(self, tree, child, parent):

        '''
        Add edge to corresponding tree

        Parameters:
            tree (int): tree which to add edge
            child (tuple): child vertex
            parent (tuple): parent vertex
        '''

        self.trees_[tree].E_[child] = parent

    def nearby(self, tree, x, n):

        '''
        Return nearby vertices

        Parameters:
            tree (int): tree being searched
            x (tuple): vertex around which searching
            n (int): max number of neighbors to return

        Return:
            list of nearby vertices
        '''

        return self.trees_[tree].V_.nearest(x, num_results=n, objects='raw')

    def get_nearest(self, tree, x):

        '''
        Return vertex nearest to x

        Parameters:
            tree (int): tree being searched
            x (tuple): vertex around which searching

        Return:
            tuple: nearest vertex to x
        '''

        return next(self.nearby(tree, x, 1))

    def bound_point(self, point):

        '''
        If a point is out-of-bounds, set to bound

        Return:
            point (tuple)
        '''

        point = np.maximum(point, self.X_.dimensions_lengths_[:,0])
        point = np.minimum(point, self.X_.dimensions_lengths_[:,1])

        return tuple(point)

    def new_and_near(self, tree, q):

        '''
        Return a new steered vertex and the vertex in the tree that is nearest

        Parameteres:
            tree (int): tree being searched
            q (tuple): lenght of edge when steering
        
        Return:
            vertex: new steered vertex
            vertex:nearest vertex in tree to new vertex
        '''

        x_rand = self.X_.sample_free()
        x_nearest = self.get_nearest(tree, x_rand)
        x_new = self.bound_point(steer(x_nearest, x_rand, q[0]))

        # Check if new point is in X_free and not already in V
        if not self.trees_[0].V_.count(x_new) == 0 or not self.X_.obstacle_free(x_new):
            return None, None

        self.samples_taken_ += 1

        return x_new, x_nearest

    def connect_to_point(self, tree, x_a, x_b):

        '''
        Connect vertex x_a in tree to vertex x_b

        Parameters:
            tree (int): tree to which to add edge
            x_a (tulpe): vertex
            x_b (tulpe): vertex

        Return:
            bool: True if able to add edge, False if prohibited by an obstacle
        '''

        if self.trees_[tree].V_.count(x_b) == 0 and self.X_.collision_free(x_a, x_b, self.r_):

            self.add_vertex(tree, x_b)
            self.add_edge(tree, x_b, x_a)

            return True

        return False

    def can_connect_to_goal(self, tree):

        '''
        Check if the goal can be connected to the graph

        Parameters:
            tree: rtree of all Vertices

        Return:
            True if can be added, False otherwise
        '''

        x_nearest = self.get_nearest(tree, self.x_goal_)

        if self.x_goal_ in self.trees_[tree].E_ and x_nearest in self.trees_[tree].E_[self.x_goal_]:

            # Tree is already connected to goal using nearest vertex
            return True

        if self.X_.collision_free(x_nearest, self.x_goal_, self.r_):
            return True

        return False

    def connect_to_goal(self, tree):

        '''
        Connect x_goal to graph

        Parameters:
            tree: stree of all Vertices
        '''

        x_nearest = self.get_nearest(tree, self.x_goal_)
        self.trees_[tree].E_[self.x_goal_] = x_nearest

    def reconstruct_path(self, tree, x_init, x_goal):

        '''
        Reconstruct path from start to goal

        Parameters:
            tree (int): tree in which to find path
            x_init (tulpe): starting vertex
            x_goal (tulpe): ending vertex

        Return:
            Path (list): Sequence of vertices from start to goal
        '''

        path = [x_goal]
        current = x_goal

        if x_init == x_goal:
            return path

        while not self.trees_[tree].E_[current] == x_init:

            path.append(self.trees_[tree].E_[current])
            current = self.trees_[tree].E_[current]

        path.append(x_init)
        path.reverse()

        return path

    def get_path(self):

        '''
        Return a path through tree from start to goal

        Return:
            Path (list) if posible, None otherwise
        '''

        if self.can_connect_to_goal(0):

            # print("Can connect to goal")
            
            self.connect_to_goal(0)

            return self.reconstruct_path(0, self.x_init_, self.x_goal_)

        # print("Could nor connect to goal")

        return None

    def check_solution(self):

        '''
        Probabilistically check if solution found
        '''

        if self.prc_ and random.random() < self.prc_:

            # print("Checking if can connect to goal at", str(self.samples_taken_), "sample")

            path = self.get_path()
            # print(self.trees_[0].V_)
            # input()

            if path is not None:
                return True, path

        # Check if can connect to goal after generating max_samples
        if self.samples_taken_ >= self.max_samples_:
            return True, self.get_path()

        return False, None

class RRT(RRTBase):

    def __init__(self, agent):

        super().__init__(agent)

    def rrt_search(self):

        '''
        Create and return a RRT, keeps expanding until can connect to goal

        Return:
            Path : E[child] = parent
        '''

        self.add_vertex(0, self.x_init_)
        self.add_edge(0, self.x_init_, None)

        while True:

            # Iterate over different edge lenghts until solution found or time out
            for q in self.Q_:

                # Iterate over number of edges of given lenght to add
                for _ in range(q[1]):

                    x_new, x_nearest = self.new_and_near(0, q)

                    if x_new is None:
                        continue

                    # Connect shortest valid edge
                    self.connect_to_point(0, x_nearest, x_new)

                    solution = self.check_solution()
                    if solution[0]:
                        return solution[1]

class RRTStar(RRTBase):

    def __init__(self, agent, rewireCount=None):

        '''
        RRT* Search

        Parameters:
            agent (Agent): All the agent information
            rewireCount (int): Number of nearby branches to rewire
        '''

        super().__init__(agent)

        self.rewire_count_ = rewireCount
        self.c_best_ = float('inf')

    def current_rewire_count(self, tree):

        '''
        Return rewire count

        Parameters:
            tree (int): Tree being rewired

        Returns:
            int: rewire count
        '''

        if self.rewire_count_ is None:
            return self.trees_[tree].V_count_

        return min(self.trees_[tree].V_count_, self.rewire_count_)

    def get_nearby_vertices(self, tree, x_init, x_new):

        '''
        Get nearby vertices to new vertex and their associated path costs from the root of tree as if new vertex
        is connected to each one separately.

        Parameters:
            tree (int): Tree in which to search
            x_init (tuple): Starting vertex used to calculate path cost
            x_new (tuple): vertex around which to find nearby vertices

        Returns:
            list: Nearby vertices and their costs, sorted in ascending order by cost
        '''

        X_near = self.nearby(tree, x_new, self.current_rewire_count(tree))
        # print(X_near)
        L_near = [(path_cost(self.trees_[tree].E_, x_init, x_near) + segment_cost(x_near, x_new), x_near)
                  for x_near in X_near]
        # print(L_near)
        L_near.sort(key=itemgetter(0))

        return L_near

    def rewire(self, tree, x_new, L_near):

        '''
        Rewire tree to shorten edges if possible
        Only rewires vertices according to rewire count

        Parameters:
            tree (int): Tree to rewire
            x_new (tuple): Newly edded vertex
            L_near (list): Nearby vertices used to rewire
        '''

        for c_near, x_near in L_near:

            curr_cost = path_cost(self.trees_[tree].E_, self.x_init_, x_near)
            tent_cost = path_cost(self.trees_[tree].E_, self.x_init_, x_new) + segment_cost(x_new, x_near)

            if tent_cost < curr_cost and self.X_.collision_free(x_near, x_new, self.r_):
                self.trees_[tree].E_[x_near] = x_new

    def connect_shortest_valid(self, tree, x_new, L_near):

        '''
        Connect to nearest vertex that has an unobstructed path

        Parameters:
            tree (int): Tree being added to 
            x_new (tuple): Vertex being added
            L_near (list): Nearby vertices
        '''

        for c_near, x_near in L_near:

            if c_near + cost_to_go(x_near, self.x_goal_) < self.c_best_ and self.connect_to_point(tree, x_near, x_new):
                break

    def rrt_search(self):

        '''
        Returns:
            set of Vertices; Edges in form: vertex: [neighbor_1, neighbor_2, ...]
        '''

        self.add_vertex(0, self.x_init_)
        self.add_edge(0, self.x_init_, None)

        while True:

            for q in self.Q_:

                for i in range(q[1]):

                    x_new, x_nearest = self.new_and_near(0, q)

                    if x_new is None:
                        continue

                    L_near = self.get_nearby_vertices(0, self.x_init_, x_new)
                    # print('L_near: ', L_near)

                    self.connect_shortest_valid(0, x_new, L_near)

                    if x_new in self.trees_[0].E_:
                        self.rewire(0, x_new, L_near)

                    solution = self.check_solution()
                    if solution[0]:
                        return solution[1]

class RRTKinodynamic(RRTBase):

    def __init__(self, agent, dynamic='free', theta=None):

        '''
        RRT Kinodynamic Search

        Parameters:
            dynamic (string): The method is able to solve for 3 different dynamics; carlike (car-like robot), ddr (differential drive robot) and
                              free (as a free particle).
            theta (float): Orientation of the robot
        '''
        
        super().__init__(agent)

        self.dynamic_ = dynamic
        self.delta_ = 0.1 # Exclusive time for the planner to set the new states
        self.controls_ = {}
        self.theta_ = theta

    def set_x_init(self):

        '''
        Based on the RRTBase which doesn't consider the orientation for this the
        RRTKinodynamic it sets the extra configuration 
        '''

        x = self.x_init_[0]
        y = self.x_init_[1]

        self.x_init_ = (x, y, self.theta_)

    def set_x_goal(self):

        '''
        The same as the "set_x_init" for x_goal
        '''

        x = self.x_goal_[0]
        y = self.x_goal_[1]

        self.x_goal_ = (x, y, self.theta_)

    def random_control(self, L=3.0, theta=0.0):

        '''
        Generates a control based on the dynamical contraints randomly

        Parameters:
            L (float): The robot length used for the "car-like" robot

        Returns:
            u (tuple): Array with the velocities in axis x and y.
        '''

        v_lin = random.random()*self.agent_.max_speed_ # Linear velocitie

        if self.dynamic_ == 'carlike':
            
            phi = (2*random.random() - 1)*np.pi/4 # The turn angle

            v_x = v_lin*np.cos(theta)*np.cos(phi)
            v_y = v_lin*np.sin(theta)*np.cos(phi)
            v_theta = v_lin*(np.tan(phi)/L)

            return (v_x, v_y, v_theta)

        elif self.dynamic_ == 'ddr':

            '''
            This model could be controled by the generation of the wheel velocities
            or accelerations
            '''

            v_x = v_lin*np.cos(theta)
            v_y = v_lin*np.sin(theta)
            v_theta = random.random()*(self.agent_.max_speed_/2)

            return (v_x, v_y, v_theta)            

        elif self.dynamic_ == 'free':

            angle = 2*(2*random.random() - 1)*np.pi

            v_x = v_lin*np.cos(angle)
            v_y = v_lin*np.sin(angle)

            return (v_x, v_y, None)

    def new_state(self, start, control):

        '''
        Calculates the new state based on the control
        '''

        x_new = start[0] + control[0]*self.delta_
        y_new = start[1] + control[1]*self.delta_

        if control[2] == None:
            return x_new, y_new, None

        theta_new = start[2] + control[2]*self.delta_

        return x_new, y_new, theta_new

    def new_and_near_constrained(self, tree, control):

        '''
        It is a variant of the "new_and_near" from the RRTBase, here the steer function
        is substituted by the extension of the control

        Parameters:
            tree (int): tree being searched
            control (tuple): random control to apply

        Return:
            vertex: new steered vertex
            vertex:nearest vertex in tree to new vertex
        '''

        x_rand = self.X_.sample_free()
        x_nearest = self.get_nearest(tree, x_rand)

        max_metric = dist_between_points(x_nearest[:2], x_rand[:2])
        x_new = self.new_state(x_nearest, control)

        if not dist_between_points(x_new, x_rand) <= max_metric:
            return None, None
        
        # Check if new point is in X_free and nor already in V.
        if not self.trees_[0].V_.count(x_new) == 0 or not self.X_.obstacle_free(x_new):
            return None, None

        self.samples_taken_ += 1

        return x_new, x_nearest

    def collision_free_constrained(self, start, end, control):

        '''
        Check if the positions generated by the control are in collision

        Returns:
            bool: True if the trajectory is collision free, False otherwise
        '''
        points = []
        step = self.delta_/30

        x = start[0]
        y = start[1]
        theta = start[2]

        points.append((x, y, theta))
        t = 0.0
        while t < self.delta_:

            x += control[0]*step
            y += control[1]*step
            if not theta is None:
                theta += control[2]*step

            points.append((x, y, theta))

            t += step

        for position in points:

            coll_free = self.X_.obstacle_free(position)

            if not coll_free:
                return coll_free

        return coll_free

    def connect_to_point_constrained(self, tree, x_a, x_b, control):
        
        '''
        Connect vertex x_a in tree to vertex x_b, based on the controls

        Returns:
            bool: Trei if able to add edge, False otherwise
        '''

        if self.trees_[tree].V_.count(x_b) == 0 and self.collision_free_constrained(x_a, x_b, control):

            self.add_vertex(tree, x_b)
            self.add_edge(tree, x_b, x_a)
            self.controls_[x_b] = control

            return True

        return False

    def can_connect_to_goal_based_control(self, tree, defaultGoalError=5):
        
        '''
        Check if the goal can be connected to the graph based a new control

        Parameters:
            tree: rtree of all Vertices

        Return:
            True if can be added, False otherwise
        '''

        x_nearest = self.get_nearest(tree, self.x_goal_)

        if self.x_goal_ in self.trees_[tree].E_ and x_nearest in self.trees_[tree].E_[self.x_goal_]:
            return True

        if self.dynamic_ == 'free':

            if self.X_.collision_free(x_nearest, self.x_goal_, self.r_):

                # Get the velocity to the goal

                x1 = x_nearest[0]
                y1 = x_nearest[1]
                x2 = self.x_goal_[0]
                y2 = self.x_goal_[1]

                goalVector = np.array((x2, y2)) - np.array((x1, y1))
                if goalVector@goalVector > 1.0:
                    goalVector /= abs(goalVector)

                self.controls_[x_nearest] = (goalVector[0], goalVector[1], None)

                return True

        control = self.random_control()
        x_new = self.new_state(x_nearest, control)

        if dist_between_points(x_new[:2], self.x_goal_[:2]) >= defaultGoalError:
            return False

        if self.collision_free_constrained(x_nearest, self.x_goal_, control):

            self.controls_[x_nearest] = control
            
            return True

        return False

    def reconstruct_path_and_controls(self, tree, x_init, x_goal):

        '''
        Reconstruct path from start to goal

        Parameters:
            tree (int): tree in which to find path
            x_init (tulpe): starting vertex
            x_goal (tulpe): ending vertex

        Return:
            Path (list): Sequence of vertices from start to goal
        '''

        path = [[x_goal, (0.0, 0.0, 0.0)]]
        current = x_goal

        if x_init == x_goal:
            return path

        while not self.trees_[tree].E_[current] == x_init:

            pos = self.trees_[tree].E[current]
            control = self.controls_[pos]
            path.append([pos, control])

            current = pos

        path.append([x_init, self.controls_[x_init]])
        path.reverse()

        return path

    def get_path_and_controls(self):

        '''
        Return a path through tree from start to goal

        Return:
            Path (list) if posible, None otherwise
        '''

        if self.can_connect_to_goal_based_control(0):

            # print("Can connect to goal by a control")

            self.connect_to_goal(0)

            return self.reconstruct_path_and_controls(0, self.x_init_, self.x_goal_)

        # print("Could nor connect to goal")

        return None

    def check_solution_with_dynamic(self):
        
        '''
        Probabilistically check if solution found
        '''

        if self.prc_ and random.random() < self.prc_:

            # print("Check if can connect to goal at", str(self.samples_taken_), "sample")

            path = self.get_path_and_control()
            # print(path)
            # input()

            if path is not None:
                return True, path

        # Check if can connect to goal after generating max_samples
        if self.samples_taken_ >= self.max_samples_:
            return True, self.get_path_and_controls()

        return False, None

    def rrt_search(self):

        '''
        Returns:
            set of Vertices and Controls
        '''

        self.add_vertex(0, self.x_init_)
        self.add_edge(0, self.x_init_, None)

        while True:

            # Iterate over different edge lenghts until solution found or time out
            for q in self.Q_:

                # Iterate over number of edges of given lenght to add
                for _ in range(q[1]):

                    control = self.random_control()
                    x_new, x_nearest = self.new_and_near_constrained(0, control)

                    if x_new is None:
                        continue

                    # Connect shortest valid edge
                    self.connect_to_point_constrained(0, x_nearest, x_new, control)

                    solution = self.check_solution_with_dynamic()
                    if solution[0]:
                        return solution[1]

class Tree(object):

    def __init__(self, X):
        
        '''
        Tree representation

        Parameter:
            X : Search Space
        '''

        p = index.Property()
        p.dimension = X.dimensions_
        self.V_ = index.Index(interleaved=True, properties=p) # vertices in a rtree
        self.V_count_ = 0
        self.E_ = {} # Edges in form E[child] = parent

class SearchSpace(object):

    def __init__(self, dimensionLengths, safetyRadius, obstacles=None):
        
        '''
        Initialize Search Space

        Parameters:
            Dimension lenghts (array): range of each dimension
            O (list): obstacles
        '''

        self.dimensions_ = len(dimensionLengths)
        self.dimensions_lengths_ = dimensionLengths
        self.safety_radius_ = safetyRadius
        self.obstacles_ = obstacles

    def sample(self):

        '''
        Return a random location within X

        Return:
            tuple: Random location within X
        '''

        x = np.random.uniform(self.dimensions_lengths_[:,0], self.dimensions_lengths_[:,1])

        return tuple(x)

    def obstacle_free(self, x):

        '''
        Check if a location resides inside of an obstacle

        Parameters:
            x (tuple): location to check

        Return:
            bool: True if not inside an obstacle, False otherwise
        '''

        if self.obstacles_ == None:
            return True

        # robot = Point(x[0], x[1])
        robot = Point(x[0], x[1]).buffer(self.safety_radius_)

        for obs in self.obstacles_:

            obstacle = Polygon(obs)

            if robot.intersects(obstacle):

                del obstacle
                del robot

                return False

            del obstacle
        del robot

        return True

    def sample_free(self):

        '''
        Sample a location within X_free

        Return:
            tuple: random location within X_free
        '''

        while True: # Sample until not inside of an obstacle

            x = self.sample()

            if self.obstacle_free(x):
                return x

    def collision_free(self, start, end, r):

        '''
        Check if a line segment intersects an obstacle

        Parameters:
            start (tuple): starting point of line
            end (tuple): ending point of line
            r (float): resoltion of points to sample along edge when checking for collision

        Returns:
            bool: True if line segment does not intersect an obstacle, False otherwise
        '''

        points = points_along_line(start, end, r)
        coll_free = all(map(self.obstacle_free, points))

        return coll_free

def steer(start, goal, d):

    '''
    Return a point in the direction of the goal, that is distance away from start

    Parameters:
        start (tuple): start location
        goal (tuple): goal location
        d (float): distance away from start

    Return:
        point (tuple): point in the direction of the goal, diatance away from start
    '''

    start, end = np.array(start), np.array(goal)

    v = end - start
    u = v/(np.sqrt(np.sum(v**2)))
    steeredPoint = start + u*d

    return tuple(steeredPoint)

def dist_between_points(a, b):

    '''
    Return the Euclidean distance between 2 points

    Parameters:
        a (tuple): first point
        b (tuple): second point

    Returns:
        float: Eclidean distance
    '''

    return np.linalg.norm(np.array(b) - np.array(a))

def points_along_line(start, end, r):

    '''
    Equally-spaced points along a line defined by start, end, with resolution r

    Parameters:
        start (tuple): starting point
        end (tuple): ending point
        r (float): maximum distance between points

    Returns:
        generator: yields points along line from start to end, separated by distance r
    '''

    d = dist_between_points(start, end)
    # n_points = int(np.ceil(d/r))
    n_points = int(30)

    if n_points > 1:

        step = d/(n_points - 1)

        for i in range(n_points):

            next_point = steer(start, end, i*step)
            yield next_point

def cost_to_go(a: tuple, b: tuple) -> float:

    '''
    Parameters:
        a (tuple): current location
        b (tuple): next location

    Returns:
        float: estimated segment_cost-to-go from a to b
    '''

    return dist_between_points(a, b)

def path_cost(E, a, b):

    '''
    Cost of the unique path from x_init to x

    Parameters:
        E (dictionary): edges, in form of E[child] = parent
        a (tuple): initial location
        b (tuple): goal location

    Returns:
        float: segment_cost of unique path from x_init to x
    '''

    cost = 0
    while not b == a:

        p = E[b]
        cost += dist_between_points(b, a)
        b = p

    return cost

def segment_cost(a, b):

    '''
    Cost function of the line between x_near and x_new

    Parameters:
        a (tuple): start of line
        b (tuple): end of line

    Returns:
        float: segment_cost function between a and b
    '''

    return dist_between_points(a, b)

if __name__ == '__main__':

    '''
    Main function to test...
    '''

    X_dimensions = np.array([(0, 100), (0, 100)]) # Dimensions of the Search Space