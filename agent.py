import math
import numpy as np
import random
import rrt
import math_ as rvo_math

from line import Line
from vector import Vector2
from shapely.geometry import Polygon, Point
# from scipy.spatial import ConvexHull

class Agent: 

    # Defines an agent in the simulation.

    def __init__(self, simulator):
        
        self.simulator_ = simulator
        self.id_ = 0
        self.leader_ = False
        self.agent_neighbors_ = [] # (float, Agent)
        self.obstacle_neighbors_ = [] # (float, Obstacle)
        self.position_ = Vector2()
        self.velocity_ = Vector2()
        self.acceleration_ = Vector2()
        self.alpha_ = 0.0
        self.formation_radius_ = 0.0
        self.goal_ = Vector2()
        self.max_neighbors_ = 0
        self.max_speed_ = 0.0
        self.neighbor_dist_ = 0.0
        self.radius_ = 0.0
        self.time_horizon_ = 0.0
        self.time_horizon_obst_ = 0.0

        # Consensu Control
        self.virtual_position_ = Vector2()
        self.graph_neighbors_ = []

        # RRT
        self.rrt_ = None
        self.X_ = None
        self.Q_ = []
        self.r_ = 0 
        self.max_samples_ = 0
        self.prc_ = 0.0
        self.current_path_ = []
        self.full_path_ = []

        # Hungarian Algorithm

        # Optimal Motion
        self.positions_transition_ = [] # Vector2
        self.positions_velocities_ = [] # Vector2
        self.positions_accelerations_ = [] # Vector2
        self.velocities_transition_ = [] # Vector2
        self.velocities_accelerations_ = [] # Vector2
        self.velocities_jerks_ = [] # Vector2

        # ORCA
        self.current_velocity_ = Vector2()
        self.orca_lines_ = [] # Line
        self.pref_velocity_ = Vector2()
        self.new_velocity_ = Vector2()

        # Sample Times
        self.consensus_t_ = 1.0 # Consensus sample time
        self.delta_t_ = 0.01 # Update time
        self.orca_t_ = 0.3

        # Save info
        self.all_positions_ = [] # Vector2
        self.all_velocities_ = [[0, 0]] # Vector2
        self.all_accelerations_ = [[0, 0]] # Vector2
        self.all_jerks_ = [[0, 0]]
        self.times_ = [0.0] # float
        self.times_discontinuous_ = [0.0]
        self.all_velocities_discontinuous_ = [] # (float, Vector2) time and velocity
        self.all_accelerations_discontinuous_ = [] # (float, Vector2) time and velocity
        self.all_errors_ = [[0, 0]]
        self.all_virtual_positions_ = [[0, 0]]

    def initialize_virtual_position(self):

        # Set the initial position of the virtual agent, according to the initial position.

        x = self.position_.x - (self.formation_radius_*math.cos(self.alpha_))
        y = self.position_.y - (self.formation_radius_*math.sin(self.alpha_))
        
        self.virtual_position_ = Vector2(x, y)

    def virtual_consensus_control(self):

        '''
        Computes the velocity of the virtual agent.

        Consensus equation:
                                     1
                x_i(k+1) = x_i(k) + ---    sum    ( x_j(k) - x_i(k) )*T
                                    N_i   j in N_i

                    x_i(k): agent current state
                    x_j(k): neighbors current states
                    N_i: number of the neighbors
                    T: sample time
        '''

        if not self.leader_:

            n = []
            for neighbor in self.simulator_.agents_:
                for i in self.graph_neighbors_:
                    if neighbor.id_ == i:
                        n.append(neighbor)

            vx = 1/2*( (n[0].virtual_position_.x - self.virtual_position_.x) + 
                        (n[1].virtual_position_.x - self.virtual_position_.x) )
            vy = 1/2*( (n[0].virtual_position_.y - self.virtual_position_.y) + 
                        (n[1].virtual_position_.y - self.virtual_position_.y) )

            # Speed saturation
            if abs(vx) > self.max_speed_*3:
                vx = self.max_speed_*(abs(vx)/vx)*3
            if abs(vy) > self.max_speed_*3:
                vy = self.max_speed_*(abs(vy)/vy)*3

            x = self.virtual_position_.x + vx*self.consensus_t_
            y = self.virtual_position_.y + vy*self.consensus_t_
            
            self.virtual_position_ = Vector2(x, y)

    def new_goal_based_consensus(self):

        '''
        Computes the current goal, based on the consensus of the virtual agent
        '''

        x = self.virtual_position_.x + self.formation_radius_*math.cos(self.alpha_)
        y = self.virtual_position_.y + self.formation_radius_*math.sin(self.alpha_)

        x, y = self.check_goal_collsion(x, y)

        self.goal_ = Vector2(x, y)

    def check_goal_collsion(self, x, y):

        '''
        Check if the current goal is in collision, if it is then it is generate a Convex-hull and 
        sample a position which will be the new goal

        Parameters:
            x (float): x coordinate of the goal
            y (float): y coordinate of the goal

        Returns:
            x (float): The new x coordinate
            y (float): The new y coordinate
        '''

        position = Point(x, y).buffer(self.radius_)
        collisionsFree = []

        for obs in self.simulator_.obstacles_[1]:

            obstacle = Polygon(obs)

            if not position.intersects(obstacle):
                collisionsFree.append(True)

            else:
                collisionsFree.append(False)

        if all(collisionsFree):
            return x, y

        else:
            return self.new_goal_based_convex_hull()

    def new_goal_based_convex_hull(self):

        '''
        Based on the position fo the neighbors a convex hull is generated

        Returns:
            list: The area constraints for the sample
        '''
        # print('Meta: ', self.id_, ' en colision')
        # input()
        n = []
        for neighbor in self.simulator_.agents_:
            for i in self.graph_neighbors_:
                if neighbor.id_ == i:
                    n.append(neighbor)

        p1 = (self.position_.x, self.position_.y)
        p2 = (n[0].position_.x, n[0].position_.y)
        p3 = (n[1].position_.x, n[1].position_.y)

        centroid_x = (p1[0] + p2[0] + p3[0])/3
        centroid_y = (p1[1] + p2[1] + p3[1])/3

        sigma = np.linalg.norm(np.array([centroid_x, centroid_y]) - np.array(p1))/2

        x = random.gauss(centroid_x, sigma)
        y = random.gauss(centroid_y, sigma)

        # Convex-hull
        convexSpace = Polygon([p1, p2, p3])
        newGoal = Point(x, y)

        if newGoal.within(convexSpace):
            return self.check_goal_collsion(x, y)
        else:
            return self.new_goal_based_convex_hull()
    
    def get_trajectory_motion_planning(self, tf, dt, x0, y0, xf, yf, resetMotion):

        '''
        Computes a Predefined-Time Trajectory for the agent, ensuring smoothness, 
        and getting the velocities and accelerations (The trajectory is going to be replaced at the half 
        to ensure the continuity between trajectories). 
        '''

        t = 0.0
        # tf = self.consensus_t_
        # dt = tf/100

        # x0 = self.position_.x
        # y0 = self.position_.y
        # xf = self.goal_.x
        # yf = self.goal_.y

        vx0 = self.velocity_.x 
        vy0 = self.velocity_.y
        vxf = 0.0 
        vyf = 0.0

        if resetMotion:
            self.positions_transition_ = [] # Vector2
            self.positions_velocities_ = [] # Vector2
            self.positions_accelerations_ = [] # Vector2

        while t <= tf:

            # Zero derivative
            y_x = (-(t/tf)**2 + (t/tf)**3)*vxf + (3*(t/tf)**2 - 2*(t/tf)**3)*xf + (1 - 3*(t/tf)**2 + 2*(t/tf)**3)*x0 + ((t/tf) - 2*(t/tf)**2 + (t/tf)**3)*vx0
            y_y = (-(t/tf)**2 + (t/tf)**3)*vyf + (3*(t/tf)**2 - 2*(t/tf)**3)*yf + (1 - 3*(t/tf)**2 + 2*(t/tf)**3)*y0 + ((t/tf) - 2*(t/tf)**2 + (t/tf)**3)*vy0
            self.positions_transition_.append(Vector2(y_x, y_y))

            # First derivative
            yP_x = (-2*(t/tf) + 3*(t/tf)**2)*vxf + (1/tf)*(6*(t/tf) - 6*(t/tf)**2)*xf + (1/tf)*(-6*(t/tf) + 6*(t/tf)**2)*x0 + (1 - 4*(t/tf) + 3*(t/tf)**2)*vx0
            yP_y = (-2*(t/tf) + 3*(t/tf)**2)*vyf + (1/tf)*(6*(t/tf) - 6*(t/tf)**2)*yf + (1/tf)*(-6*(t/tf) + 6*(t/tf)**2)*y0 + (1 - 4*(t/tf) + 3*(t/tf)**2)*vy0
            self.positions_velocities_.append(Vector2(yP_x, yP_y))

            # Second derivative
            yPP_x = (1/tf)*(-2 + 6*(t/tf))*vxf + (1/tf)**2*(6 - 12*(t/tf))*xf + (1/tf)**2*(-6 + 12*(t/tf)**2)*x0 + (1/tf)*(-4 + 6*(t/tf))*vx0
            yPP_y = (1/tf)*(-2 + 6*(t/tf))*vyf + (1/tf)**2*(6 - 12*(t/tf))*yf + (1/tf)**2*(-6 + 12*(t/tf)**2)*y0 + (1/tf)*(-4 + 6*(t/tf))*vy0
            self.positions_accelerations_.append(Vector2(yPP_x, yPP_y))

            self.times_.append(self.times_[-1] + dt)
            t += dt

        # self.position_for_planning_ = 

    def free_line_to_goal(self):

        '''
        Generates a line between the current position of the robot and its goal and check if it is free collision

        Returns:
            bool: True if it is free and False if it is not
        '''

        start = [self.position_.x, self.position_.y]
        stop = [self.goal_.x, self.goal_.y]

        positions = np.linspace(start, stop, 30)

        for pos in positions:

            position = Point(pos[0], pos[1]).buffer(self.radius_)
            
            for obs in self.simulator_.obstacles_[1]:

                obstacle = Polygon(obs)

                if position.intersects(obstacle):
                    return False

                del obstacle

            del position

        return True

    def path_RRT(self):

        '''
        Computes the path of the agent from the current position to the current goal.

        Parameters:
            RRTStar (bool): Bool variable to ask if use the optimal RRT called RRT Star (RRT*)
        '''

        self.current_path_ = []

        # Agregar el RRT como opcion
        # self.rrt_ = rrt.RRT(self) 
        self.rrt_ = rrt.RRTStar(self)
        self.current_path_ = self.rrt_.rrt_search()

        if self.current_path_ == None:
            self.current_path_ = [(self.position_.x, self.position_.y)]

        # Agregar un acumulador para saber todos los caminos
        del self.rrt_     

    def compute_neighbors(self):

        # Computes the neighbors of this agent.

        rangeSq = rvo_math.square(self.time_horizon_obst_*self.max_speed_ + self.radius_)
        self.obstacle_neighbors_ = []
        self.simulator_.kd_tree_.compute_obstacle_neighbors(self, 30)

        self.agent_neighbors_ = []
        if self.max_neighbors_ > 0:

            rangeSq = rvo_math.square(self.neighbor_dist_)
            self.simulator_.kd_tree_.compute_agent_neighbors(self, rangeSq)

    def compute_new_velocity(self):

		# Computes the new velocity of this agent.
        
        self.orca_lines_ = []

        invTimeHorizonObst = 1.0/self.time_horizon_obst_
        
        print(self.id_)
        print(self.obstacle_neighbors_)
        input()

		# Crete obstacle ORCA lines.
        for i in range(len(self.obstacle_neighbors_)):

            obstacle1 = self.obstacle_neighbors_[i][1]
            obstacle2 = obstacle1.next_

            relativePosition1 = obstacle1.point_ - self.position_
            relativePosition2 = obstacle2.point_ - self.position_

			# Check if velocity obstacle of obstacle is already take care of by previously constructed obstacle ORCA lines.
            alreadyCovered = False

            for j in range(len(self.orca_lines_)):

                det1 = rvo_math.det(invTimeHorizonObst*relativePosition1 - self.orca_lines_[j].point, self.orca_lines_[j].direction) 
                det2 = rvo_math.det(invTimeHorizonObst*relativePosition2 - self.orca_lines_[j].point, self.orca_lines_[j].direction) 

                if (det1 - invTimeHorizonObst*self.radius_ >= -rvo_math.EPSILON) and (det2 - invTimeHorizonObst*self.radius_ >= -rvo_math.EPSILON):

                    alreadyCovered = True
                    break

            if alreadyCovered:
                continue

			# Not covered. Check for collision.
            distSq1 = rvo_math.abs_sq(relativePosition1)
            distSq2 = rvo_math.abs_sq(relativePosition2)

            radiusSq = rvo_math.square(self.radius_)

            obstacleVector = obstacle2.point_ - obstacle1.point_
            s = (-relativePosition1@obstacleVector)/rvo_math.abs_sq(obstacleVector)
            distSqLine = rvo_math.abs_sq(-relativePosition1 - s*obstacleVector)

            line = Line()

            if s < 0.0 and distSq1 <= radiusSq:

				# Collision with left vertex. Ignore if non-convex.
                if obstacle1.convex_:

                    line.point = Vector2(0.0, 0.0)
                    line.direction = rvo_math.normalize(Vector2(-relativePosition1.y, relativePosition1.x))
                    self.orca_lines_.append(line)

                continue

            elif s > 1.0 and distSq2 <= radiusSq:

				# Collision with right vertex. Ignore if non-convex or if it will be taken care of by neighboring obstacle.
                if obstacle2.convex_ and rvo_math.det(relativePosition2, obstacle2.direction_) >= 0.0:

                    line.point = Vector2(0.0, 0.0)
                    line.direction = rvo_math.normalize(Vector2(-relativePosition2.y, relativePosition2.x))
                    self.orca_lines_.append(line)

                continue

            elif s >= 0.0 and s < 1.0 and distSqLine <= radiusSq:

				# Collision with obstacle segment.
                line.point = Vector2(0.0, 0.0)
                line.direction = -obstacle1.direction_
                self.orca_lines_.append(line)

                continue

			# No collision. Compute legs. When obliquely viewed, both legs can come from a single vertex. Legs extend cut-off line when non-convex vertex.
            leftLegDirection = None
            rightLegDirection = None

            if s < 0.0 and distSqLine <= radiusSq:

				# Obstacle viewed obliquely so that left vertex defines velocity obstacle.
                if not obstacle1.convex_:
					#Ignore obstacle.
                    continue

                obstacle2 = obstacle1

                leg1 = math.sqrt(distSq1 - radiusSq)
                leftLegDirection = Vector2(relativePosition1.x*leg1 - relativePosition1.y*self.radius_, relativePosition1.x*self.radius_ + relativePosition1.y*leg1)/distSq1
                rightLegDirection = Vector2(relativePosition1.x*leg1 + relativePosition1.y*self.radius_, -relativePosition1.x*self.radius_ + relativePosition1.y*leg1)/distSq1

            elif s > 1.0 and distSqLine <= radiusSq:

				# Obstacle viewed obliquely so that right vertex defines velocity obstacle.
                if not obstacle2.convex_:
					# Ignore obstacle.
                    continue

                obstacle1 = obstacle2

                leg2 = math.sqrt(distSq2 - radiusSq)
                leftLegDirection = Vector2(relativePosition2.x*leg2 - relativePosition2.y*self.radius_, relativePosition2.x*self.radius_ + relativePosition2.y*leg2)/distSq2
                rightLegDirection = Vector2(relativePosition2.x*leg2 + relativePosition2.y*self.radius_, -relativePosition2.x*self.radius_ + relativePosition2.y*leg2)/distSq2

            else:

                # Usual situation.
                if obstacle1.convex_:

                    leg1 = math.sqrt(distSq1 - radiusSq)
                    leftLegDirection = Vector2(relativePosition1.x*leg1 - relativePosition1.y*self.radius_, relativePosition1.x*self.radius_ + relativePosition1.y*leg1)/distSq1

                else:
					# Left vertex non-convex left leg extends cut-off line.
                    leftLegDirection = -obstacle1.direction_

                if obstacle2.convex_:

                    leg2 = math.sqrt(distSq2 - radiusSq)
                    rightLegDirection = Vector2(relativePosition2.x*leg2 + relativePosition2.y*self.radius_, -relativePosition2.x*self.radius_ + relativePosition2.y*leg2)/distSq2

                else:
					# Right vertex non-convex right leg extends cut-off line.
                    rightLegDirection = obstacle1.direction_

			# Legs can never point into neighboring edge when convex vertex, take cutoff-line of neighboring edge instead. If velocity projected on "foreign" leg, no constraint is added.

            leftNeighbor = obstacle1.previous_

            isLeftLegForeign = False
            isRightLegForeign = False

            if obstacle1.convex_ and rvo_math.det(leftLegDirection, -leftNeighbor.direction_) >= 0.0:

				# Left leg points into obstacle.

                leftLegDirection = -leftNeighbor.direction_
                isLeftLegForeign = True

            if obstacle2.convex_ and rvo_math.det(rightLegDirection, obstacle2.direction_) <= 0.0:

				# Right leg points into obstacle.

                rightLegDirection = obstacle2.direction_
                isRightLegForeign = True

			# Compute cut-off centers.
            leftCutOff = invTimeHorizonObst*(obstacle1.point_ - self.position_)
            rightCutOff = invTimeHorizonObst*(obstacle2.point_ - self.position_)
            cutOffVector = rightCutOff - leftCutOff

			# Project current velocity on velocity obstacle.

			# Check if current velocity is projected on cutoff circles.
            t = 0.5 if obstacle1 == obstacle2 else ((self.velocity_ - leftCutOff)@cutOffVector)/rvo_math.abs_sq(cutOffVector)
            tLeft = (self.velocity_ - leftCutOff)@leftLegDirection
            tRight = (self.velocity_ - rightCutOff)@rightLegDirection

            if (t < 0.0 and tLeft < 0.0) or (obstacle1 == obstacle2 and tLeft < 0.0 and tRight < 0.0):

				# Project on left cut-off circle.
                unitW = rvo_math.normalize(self.velocity_ - leftCutOff)
                line.direction = Vector2(unitW.y, -unitW.x)
                line.point = leftCutOff + self.radius_*invTimeHorizonObst*unitW
                self.orca_lines_.append(line)

                continue

            elif t > 1.0 and tRight < 0.0:

				# Project on right cut-off circle.
                unitW = rvo_math.normalize(self.velocity_ - rightCutOff)
                line.direction = Vector2(unitW.y, -unitW.x)
                line.point = rightCutOff + self.radius_*invTimeHorizonObst*unitW
                self.orca_lines_.append(line)

                continue

			# Project on left, right leg, or cut-off line, whichever is closest to velocity.
            distSqCutoff = math.inf if t < 0.0 or t > 1.0 or obstacle1 == obstacle2 else rvo_math.abs_sq(self.velocity_ - (leftCutOff + t*cutOffVector))
            distSqLeft = math.inf if tLeft < 0.0 else rvo_math.abs_sq(self.velocity_ - (leftCutOff + tLeft*leftLegDirection))
            distSqRight = math.inf if tRight < 0.0 else rvo_math.abs_sq(self.velocity_ - (rightCutOff + tRight*rightLegDirection))

            if distSqCutoff <= distSqLeft and distSqCutoff <= distSqRight:

				# Project on cut-off line.
                line.direction = -obstacle1.direction_
                line.point = leftCutOff + self.radius_*invTimeHorizonObst*Vector2(-line.direction.y, line.direction.x)
                self.orca_lines_.append(line)

                continue

            if distSqLeft <= distSqRight:

				# Project on left leg.
                if isLeftLegForeign:
                    continue

                line.direction = leftLegDirection
                line.point = leftCutOff + self.radius_*invTimeHorizonObst*Vector2(-line.direction.y, line.direction.x)
                self.orca_lines_.append(line)

                continue

			# Project on right leg.
            if isRightLegForeign:
                continue

            line.direction = -rightLegDirection
            line.point = rightCutOff+self.radius_*invTimeHorizonObst*Vector2(-line.direction.y, line.direction.x)
            self.orca_lines_.append(line)

        numObstLines = len(self.orca_lines_)

        invTimeHorizon = 1.0/self.time_horizon_

		# Crete agent ORCA lines.
        for i in range(len(self.agent_neighbors_)):

            other = self.agent_neighbors_[i][1]

            relativePosition = other.position_ - self.position_
            relativeVelocity = self.velocity_ - other.velocity_

            distSq = rvo_math.abs_sq(relativePosition)
            combinedRadius = self.radius_ + other.radius_
            combinedRadiusSq = rvo_math.square(combinedRadius)

            line = Line()
            u = Vector2()

            if distSq > combinedRadiusSq:

				# No collision.
                w = relativeVelocity - invTimeHorizon*relativePosition

				# Vector from cutoff center to relative velocity.
                wLengthSq = rvo_math.abs_sq(w)
                dotProduct1 = w@relativePosition

                if dotProduct1 < 0.0 and rvo_math.square(dotProduct1) > combinedRadiusSq*wLengthSq:

					# Project on cut-off circle.
                    wLength = math.sqrt(wLengthSq)
                    unitW = w/wLength

                    line.direction = Vector2(unitW.y, -unitW.x)
                    u = (combinedRadius*invTimeHorizon - wLength)*unitW

                else:

					# Project on legs.
                    leg = math.sqrt(distSq - combinedRadiusSq)

                    if rvo_math.det(relativePosition, w) > 0.0:
						# Project on left leg.
                        line.direction = Vector2(relativePosition.x*leg - relativePosition.y*combinedRadius, relativePosition.x*combinedRadius + relativePosition.y*leg)/distSq

                    else:
						# Project on right leg.
                        line.direction = -Vector2(relativePosition.x*leg + relativePosition.y*combinedRadius, -relativePosition.x*combinedRadius + relativePosition.y*leg)/distSq

                    dotProduct2 = relativeVelocity@line.direction
                    u = dotProduct2*line.direction - relativeVelocity

            else:

				# Collision. Project on cut-off circle of time timeStep.
                invTimeStep = 1.0/self.simulator_.time_step_

				# Vector from cutoff center to relative velocity.
                w = relativeVelocity - invTimeStep*relativePosition

                wLength = abs(w)
                unitW = w/wLength

                line.direction = Vector2(unitW.y, -unitW.x)
                u = (combinedRadius*invTimeStep - wLength) * unitW

            line.point = self.velocity_ + 0.5*u
            self.orca_lines_.append(line)

        lineFail, self.new_velocity_ = self.linear_program2(self.orca_lines_, self.max_speed_, self.pref_velocity_, False, self.new_velocity_)

        if lineFail < len(self.orca_lines_):
            self.new_velocity_ = self.linear_program3(self.orca_lines_, numObstLines, lineFail, self.max_speed_, self.new_velocity_)

        # print(self.obstacle_neighbors_)
        # input()

    def linear_program1(self, lines, lineNo, radius, optVelocity, directionOpt):

        '''
		Solves a 1-dimensional linear program on a specified line sibject to linear constraints defined by lines and circular constraint.

		Args:
			lines (list): Lines defining the linear constraints.
			lineNo (int): The specified line constraint.
			radius (float): The radius of the circular constraint.
			optVelocity (Vector2): The optimization velocity.
			directionOpt (bool): True if the direction should be optimized.

		Returns:
			bool: True if successful.
			Vector 2: A reference to the result of the linear program.
		'''

        dotProduct = lines[lineNo].point@lines[lineNo].direction
        discriminant = rvo_math.square(dotProduct) + rvo_math.square(radius) - rvo_math.abs_sq(lines[lineNo].point)

        if discriminant < 0.0:
			# Max speed circle fully invalidates line lineNo.
            return False, None

        sqrtDiscriminant = math.sqrt(discriminant)
        tLeft = -dotProduct - sqrtDiscriminant
        tRight = -dotProduct + sqrtDiscriminant

        for i in range(lineNo):

            denominator = rvo_math.det(lines[lineNo].direction, lines[i].direction)
            numerator = rvo_math.det(lines[i].direction, lines[lineNo].point - lines[i].point)

            if abs(denominator) <= rvo_math.EPSILON:

				# Lines lineNo and i are (almost) parallel.
                if numerator < 0.0:
                    return False, None

                continue

            t = numerator/denominator

            if denominator >= 0.0:
				# Line i bounds line lineNo on the right.
                tRight = min(tRight, t)

            else:
				# Line i bounds line lineNo on the left.
                tLeft = max(tLeft, t)

            if tLeft > tRight:
                return False, None

        if directionOpt:

			# Optimize direction.
            if optVelocity@lines[lineNo].direction > 0.0:
				# Take right extreme.
                result = lines[lineNo].point + tRight*lines[lineNo].direction

            else:
				# Take left extreme.
                result = lines[lineNo].point + tLeft*lines[lineNo].direction

        else:

			# Optimize closest point.
            t = lines[lineNo].direction@(optVelocity - lines[lineNo].point)

            if t < tLeft:
                result = lines[lineNo].point + tLeft*lines[lineNo].direction

            elif t > tRight:
                result = lines[lineNo].point + tRight*lines[lineNo].direction

            else:
                result = lines[lineNo].point + t*lines[lineNo].direction

        return True, result

    def linear_program2(self, lines, radius, optVelocity, directionOpt, result):

        '''
		Solves a 2-dimensional linear program subject to linear constraints defined by lines and a circular constraint.

		Args:
			lines (list): Lines defining the linear constraints.
			radius (float): The radius of the circular constraint.
			optVelocity (Vector2): The optimization velocity.
			directionOpt (bool): True if the direction should be optimized.
			result (Vector2): A reference to the reslt of the linear program.

		Returns:
			int: the number of the line it fails on, and the number of the lines if successful.
			Vector2: A reference to the result of the linear program.
		'''

        if directionOpt:
			# Optimize direction. Note that the optimization velocity is of unit length in this case.
            result = optVelocity*radius

        elif rvo_math.abs_sq(optVelocity) > rvo_math.square(radius):
			# Optimize closest point and iniside circle.
            result = rvo_math.normalize(optVelocity)*radius

        else:
			# Optimize closest point and inside circle.
            result = optVelocity

        for i in range(len(lines)):

            if rvo_math.det(lines[i].direction, lines[i].point - result) > 0.0:

				# Result does not satisfy constraint i. Compute new optimal result.
                tempResult = result
                success, result = self.linear_program1(lines, i, radius, optVelocity, directionOpt)

                if not success:

                    result = tempResult
                    return i, result

        return len(lines), result

    def linear_program3(self, lines, numObstLines, beginLine, radius, result):

        '''
		Solves a 2-dimensional linear program subject to linear constraints defined by lines and circular constraint.

		Args:
			lines (list): Lines defining the linear constraints.
			numObstLines (int): Count of obstacle lines.
			beginLine (int): The line on which the 2-d linear program failed.
			radius (float): The radius of the circular constraint.
			result (Vector2): A reference to the result of the linear program.

		Returns:
			Vector2: A reference to the result of the linear program.
		'''

        distance = 0.0

        for i in range(beginLine, len(lines)):

            if rvo_math.det(lines[i].direction, lines[i].point - result) > distance:

				# Result does not satisfy constraint of line i.

                projLines = []

                for ii in range(numObstLines):
                    projLines.append(lines[ii])

                for j in range(numObstLines, i):

                    line = Line()
                    determinant = rvo_math.det(lines[i].direction, lines[j].direction)

                    if abs(determinant) <= rvo_math.EPSILON:

						# Line i and line j are parallel.
                        if lines[i].direction@lines[j].direction > 0.0:
							# Line i and line j point in the same direction.
                            continue

                        else:
							# Line i and line j point in opposite direction.
                            line.point = 0.5*(lines[i].point + lines[j].point)

                    else:
                        line.point = lines[i].point + (rvo_math.det(lines[j].direction, lines[i].point - lines[j].point)/determinant)*lines[i].direction

                    line.direction = rvo_math.normalize(lines[j].direction - lines[i].direction)
                    projLines.append(line)

                tempResult = result
                lineFail, result = self.linear_program2(projLines, radius, Vector2(-lines[i].direction.y, lines[i].direction.x), True, result)	

                if lineFail < len(projLines):
					# This should in principle not happen. The result is by definition already in the feasible region of this linear program. If it fails, it is due to small floating point error, and the current result is kept.
                    result = tempResult

                distance = rvo_math.det(lines[i].direction, lines[i].point - result)

        return result
    
    def insert_agent_neighbor(self, agent, rangeSq):

        '''
		Inserts an agent neighbor into the set of neighbors of this agent.

		Args:
			agent (Agent): A pointer to the agent to be inserted.
			rangeSq (float): The squared range around this agent.
		'''

        if self != agent:

            distSq = rvo_math.abs_sq(self.position_ - agent.position_)

            if distSq < rangeSq:

                if len(self.agent_neighbors_) < self.max_neighbors_:
                    self.agent_neighbors_.append((distSq, agent))

                i = len(self.agent_neighbors_) - 1
                while i != 0 and distSq < self.agent_neighbors_[i - 1][0]:

                    self.agent_neighbors_[i] = self.agent_neighbors_[i -1]
                    i -= 1

                self.agent_neighbors_[i] = (distSq, agent)

                if len(self.agent_neighbors_) == self.max_neighbors_:
                    rangeSq = self.agent_neighbors_[len(self.agent_neighbors_) - 1][0]

        return rangeSq

    def insert_obstacle_neighbor(self, obstacle, rangeSq):

        '''
		Inserts a static obstacle neighbor into the set of neighbors of this agent.

		Args: 
			obstacle (Obstacle): The number of the static obstacle to be inserted.
			rangeSq (float): The squared range around this agents.
		'''

        nextObstacle = obstacle.next_
        distSq = rvo_math.dist_sq_point_line_segment(obstacle.point_, nextObstacle.point_, self.position_)

        if distSq < rangeSq:

            self.obstacle_neighbors_.append((distSq, obstacle))

            i = len(self.obstacle_neighbors_) - 1

            while i != 0 and distSq < self.obstacle_neighbors_[i - 1][0]:

                self.obstacle_neighbors_[i] = self.obstacle_neighbors_[i - 1]
                i -= 1

            self.obstacle_neighbors_[i] = (distSq, obstacle)

    def smooth_ORCA(self):

        '''
        Computes a Predefined-Time transition for the velocities, ensuring smoothness. 
        '''

        for _ in range(9):
            self.all_velocities_discontinuous_.append([self.new_velocity_.x, self.new_velocity_.y])
            # self.times_discontinuous_.append(self.times_discontinuous_[-1] + self.delta_t_*10)
        
        t = 0.0
        tf = 0.1
        dt = tf/10

        x0 = self.velocity_.x 
        y0 = self.velocity_.y
        xf = self.new_velocity_.x
        yf = self.new_velocity_.y

        vx0 = 0
        vy0 = 0
        vxf = 0
        vyf = 0

        self.velocities_transition_ = []
        self.velocities_accelerations_ = []
        self.velocities_jerks_ = []

        while t <= tf:

            # Zero derivative
            y_x = (-(t/tf)**2 + (t/tf)**3)*vxf + (3*(t/tf)**2 - 2*(t/tf)**3)*xf + (1 - 3*(t/tf)**2 + 2*(t/tf)**3)*x0 + ((t/tf) - 2*(t/tf)**2 + (t/tf)**3)*vx0
            y_y = (-(t/tf)**2 + (t/tf)**3)*vyf + (3*(t/tf)**2 - 2*(t/tf)**3)*yf + (1 - 3*(t/tf)**2 + 2*(t/tf)**3)*y0 + ((t/tf) - 2*(t/tf)**2 + (t/tf)**3)*vy0
            self.velocities_transition_.append(Vector2(y_x, y_y))

            # First derivative
            yP_x = (-2*(t/tf) + 3*(t/tf)**2)*vxf + (1/tf)*(6*(t/tf) - 6*(t/tf)**2)*xf + (1/tf)*(-6*(t/tf) + 6*(t/tf)**2)*x0 + (1 - 4*(t/tf) + 3*(t/tf)**2)*vx0
            yP_y = (-2*(t/tf) + 3*(t/tf)**2)*vyf + (1/tf)*(6*(t/tf) - 6*(t/tf)**2)*yf + (1/tf)*(-6*(t/tf) + 6*(t/tf)**2)*y0 + (1 - 4*(t/tf) + 3*(t/tf)**2)*vy0
            self.velocities_accelerations_.append(Vector2(yP_x, yP_y))

            # Second derivative
            yPP_x = (1/tf)*(-2 + 6*(t/tf))*vxf + (1/tf)**2*(6 - 12*(t/tf))*xf + (1/tf)**2*(-6 + 12*(t/tf)**2)*x0 + (1/tf)*(-4 + 6*(t/tf))*vx0
            yPP_y = (1/tf)*(-2 + 6*(t/tf))*vyf + (1/tf)**2*(6 - 12*(t/tf))*yf + (1/tf)**2*(-6 + 12*(t/tf)**2)*y0 + (1/tf)*(-4 + 6*(t/tf))*vy0
            self.velocities_jerks_.append(Vector2(yPP_x, yPP_y))

            self.times_.append(self.times_[-1] + dt)
            t += dt

        # print('Velociades iniciales: ',x0, y0)
        # print('Velociades finales: ',xf, yf)
        # print('Transicion: ')
        # for vel in self.velocities_transition_:
        #     print(vel.x, vel.y)
        # print('Transicion acc: ')
        # for acc in self.velocities_accelerations_:
        #     print(acc.x, acc.y)
        # print('Transicion jerk: ')
        # for jerk in self.velocities_jerks_:
        #     print(jerk.x, jerk.y)
        # input()

        # for acc in self.velocities_accelerations_:
        #     self.all_accelerations_.append([acc.x, acc.y])

    def update(self, i):

        '''
        Update the 2-D position and 2-D velocity of this agent.
        '''

        self.velocity_ = self.new_velocity_
        # print(self.velocity_)
        # self.velocity_ = self.current_velocity_
        # self.velocity_ = self.velocities_transition_[i]

        self.acceleration_ = self.velocities_accelerations_[i]
        self.jerk_ = self.velocities_jerks_[i]
        self.position_ += self.velocity_*(self.simulator_.time_step_/10)
        # self.position_ = self.positions_transition_[i]
        self.times_.append(self.times_[-1] + self.delta_t_)

        self.all_positions_.append(self.position_)
        self.all_velocities_.append([self.velocity_.x, self.velocity_.y])
        self.all_accelerations_.append([self.acceleration_.x, self.acceleration_.y])
        self.all_jerks_.append([self.jerk_.x, self.jerk_.y])

        goal = np.array((self.goal_.x, self.goal_.y))
        pos = np.array((self.position_.x, self.position_.y))
        error = goal - pos
        self.all_errors_.append(error)
        self.all_virtual_positions_.append([self.virtual_position_.x, self.virtual_position_.y])