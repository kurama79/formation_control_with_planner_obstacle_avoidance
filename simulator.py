import math_ as rvo_math
import numpy as np
import random
import time
import rrt as RRT

from kdtree import KdTree
from agent import Agent
from obstacle import Obstacle
from shapely.geometry import Polygon, Point
from vector import Vector2

class Simulator:

	# Defines the simulation.

	def __init__(self, method=1):

		# Constructs and initializes a simulation.

		self.agents_ = []
		self.default_agent_ = None
		self.kd_tree_ = KdTree(self)
		self.obstacles_ = [[], []]
		self.global_time_ = 0.0
		self.time_step_ = 0.1
		self.velocity_index_ = 0
		self.method_ = method
		self.smooth_ORCA_index_ = 0

		self.simulation_begins_ = True
		self.trajectory_time_ = []

	def add_agent(self, position):

		'''
		Adds a new agent with default properties to the simulation.

		Args:
			position (Vestor2): The 2-dimensional starting position of this agent.

		Returns:
			int: The number of the agent, or -1 when the agent defaults have not seen set.
		'''

		if self.default_agent_ is None:
			raise ArgumentError('Defualt agent not set. Call set_agent_defaults first.')

		agent = Agent(self)
		agent.id_ = len(self.agents_)
		agent.max_neighbors_ =self.default_agent_.max_neighbors_
		agent.max_speed_ = self.default_agent_.max_speed_
		agent.neighbor_dist_ = self.default_agent_.neighbor_dist_
		agent.position_ = position
		agent.radius_ = self.default_agent_.radius_
		agent.time_horizon_ = self.default_agent_.time_horizon_
		agent.time_horizon_obst_ = self.default_agent_.time_horizon_obst_
		agent.velocity_ = self.default_agent_.velocity_
		agent.alpha_ = self.default_agent_.alpha_*agent.id_
		agent.formation_radius_ = self.default_agent_.formation_radius_

		# RRT
		agent.X_ = self.default_agent_.X_
		agent.Q_ = self.default_agent_.Q_
		agent.r_ = self.default_agent_.r_
		agent.max_samples_ = self.default_agent_.max_samples_
		agent.prc_ = self.default_agent_.prc_

		self.agents_.append(agent)

		return agent.id_

	def add_obstacle(self, vertices):

		'''
		Adds a new obstacle to the simulation.

		Args:
			vertices (list): List of the vertices of the polygonal obstacle in counterclockwise order.

		Returns:
			int: The number of the first vertex of the obstacle, or -1 when the number of vertices is less than 2.

		Remark:
			To add a "negative" obstacle, e.g. a bounding polygon around the environment, the verticees should be listed in clockwise order.
		'''

		if len(vertices) < 2:
			raise ArgumentError('Must have at least 2 vertices.')

		obstacleNo = len(self.obstacles_[0])

		for i in range(len(vertices)):

			obstacle = Obstacle()
			obstacle.point_ = vertices[i]

			if i != 0:

				obstacle.previous_ = self.obstacles_[0][len(self.obstacles_[0]) - 1]
				obstacle.previous_.next_ = obstacle

			if i == len(vertices) - 1:

				obstacle.next_ = self.obstacles_[0][obstacleNo]
				obstacle.next_.previous_ = obstacle

			obstacle.direction_ = rvo_math.normalize(vertices[0 if i==len(vertices) - 1 else i+1] - vertices[i])

			if len(vertices) == 2:
				obstacle.convex_ = True

			else:

				obstacle.convex_ = rvo_math.left_of(
									vertices[len(vertices)-1 if i==0 else i-1],
									vertices[i],
									vertices[0 if i==len(vertices)-1 else i+1]) >= 0

			obstacle.id_ = len(self.obstacles_[0])
			self.obstacles_[0].append(obstacle)

		return obstacleNo

	def set_graph_neighbors_id(self):

		'''
		Save the id of the neighbors used to define the formation
		'''

		n = len(self.agents_)
		for agent in self.agents_:

			if agent.id_ == 0:
				agent.graph_neighbors_ = [1, n-1]

			elif agent.id_ == n - 1:
				agent.graph_neighbors_ = [agent.id_ - 1, 0]

			else:
				agent.graph_neighbors_ = [agent.id_ - 1, agent.id_ + 1]

	def get_virtual_positions(self):

		# Calculate the initial position of the virtual agents

		for agent in self.agents_:
			agent.initialize_virtual_position()

	def get_consensus_velocities(self):

		'''
		Compute the consensus references and trajectories
		'''

		for agent in self.agents_:
		
			# Consensus
			agent.virtual_consensus_control()
			agent.new_goal_based_consensus()
			# print(agent.goal_)
			# agent.get_trajectory_motion_planning(agent.consensus_t_, agent.consensus_t_/100)

	def get_trajectory_current_goal(self):

		'''
		Compute the paths and trajectories from the current position to the current goal by the RRT
		'''

		# self.timers_['trajectories'] = []
		t = time.time()

		for agent in self.agents_:

			# if agent.free_line_to_goal():
			if True:

				print('free line')

				tf = agent.consensus_t_
				dt = tf/20
				x0 = agent.position_.x
				y0 = agent.position_.y
				xf = agent.goal_.x
				yf = agent.goal_.y

				agent.get_trajectory_motion_planning(tf, dt, x0, y0, xf, yf, resetMotion=True)

				self.trajectory_time_.append(time.time() - t)
				print(self.trajectory_time_[-1])

				continue
				
			# Get the path by the RRT
			# agent.path_RRT()
			if len(agent.current_path_) <= 1:
				agent.path_RRT() # Bool: True is used the RRTStar, False, is used the RRT

			else:

				dist = self.two_points_distance2([agent.position_.x, agent.position_.y], 
								[agent.current_path_[1][0], agent.current_path_[1][1]])

				if dist <= agent.radius_:
					agent.path_RRT()
				
			# Get the trajectory based on the nodes of the path
			# tf = agent.consensus_t_/len(agent.current_path_)
			# dt = tf / (20/len(agent.current_path_))
			tf = agent.consensus_t_
			dt = tf/20
			# index = 0

			x0 = agent.position_.x
			y0 = agent.position_.y
				
			# for i in range(1, len(agent.current_path_)):

			# 	if i == 1:
					# resetMotion = True
			# 		x0 = agent.position_.x
			# 		y0 = agent.position_.y

			# 	else:
			# 		resetMotion = False
			# 		x0 = agent.positions_transition_[-1].x
			# 		y0 = agent.positions_transition_[-1].y
			if len(agent.current_path_) >= 2:
				xf = agent.current_path_[1][0]
				yf = agent.current_path_[1][1]
			else:
				xf = agent.current_path_[0][0]
				yf = agent.current_path_[0][1]

			agent.get_trajectory_motion_planning(tf, dt, x0, y0, xf, yf, resetMotion=True)
			# index = int(len(agent.positions_transition_)/2)

			# print('Posicion actual de agente:', agent.position_)
			# print('Meta actual de agente:', agent.goal_)
			# print("Posiciones del camino:")
			# for pos in agent.positions_transition_:
			# 	print(pos)
			# input()

			self.trajectory_time_.append(time.time() - t)
			print(self.trajectory_time_[-1])

	def set_velocities_to_formation(self, i, iterations):

		'''
		Performs a simulation step and set the velocities and positions of the agents to reach the formation.

		Args:
			i (int): index to select the current velocity
		'''

		self.kd_tree_.build_agent_tree()

		if self.smooth_ORCA_index_ >= 9 or iterations == 0:

			for agentNo in range(self.num_agents):

				self.agents_[agentNo].compute_neighbors()
				self.agents_[agentNo].compute_new_velocity()
				self.agents_[agentNo].smooth_ORCA()

				self.smooth_ORCA_index_ = 0

			newUpdate = True
			increase = 1
		
		else:
			newUpdate = False
			increase = 0

		for agentNo in range(self.num_agents):
			self.agents_[agentNo].new_velocity_ = self.agents_[agentNo].velocities_transition_[self.smooth_ORCA_index_]

		for agent in self.agents_:
			agent.update(self.smooth_ORCA_index_)

		self.global_time_ += self.time_step_
		self.smooth_ORCA_index_ += 1

		return newUpdate, increase

	def check_formation_collision(self):

		'''
		Check if the formation intersects an obstcles and sample to find a free collision location. Then a virtual agent takes the leader role
		and goes to this location.
		'''

		if self.global_time_ >= 200:

			collision = False
			formationShapeRadius = (self.default_agent_.radius_ + self.default_agent_.formation_radius_)*1.01

			# Check for collision
			centroid_x = self.agents_[0].virtual_position_.x 
			centroid_y = self.agents_[0].virtual_position_.y
			formation = Point(centroid_x, centroid_y).buffer(formationShapeRadius)

			for obs in self.obstacles_[1]:

				obstacle = Polygon(obs)

				if formation.intersects(obstacle):

					collision = True

					del obstacle
					del formation

					break

			if collision:

				self.agents_[0].leader_ = True
				collisionFree = []
				upperBound = self.default_agent_.X_.dimensions_lengths_[1,1]
				lowBound = self.default_agent_.X_.dimensions_lengths_[0,0]

				for _ in range(100):

					x = random.uniform(lowBound, upperBound)
					y = random.uniform(lowBound, upperBound)

					position = Point(x, y).buffer(formationShapeRadius)

					for obs in self.obstacles_[1]:

						obstacle = Polygon(obs)

						if not position.intersects(obstacle):
							collisionFree.append((x, y))

						del obstacle

					del position

				if len(collisionFree) > 0:

					distances = []
					for pos in collisionFree:
						dist = np.linalg.norm(np.array([centroid_x, centroid_y]) - np.array(pos))
						distances.append(dist)

					closest = 100000
					for index, dist in enumerate(distances):
						if dist < closest:
							closest = dist
							indexPosition = index

					self.agents_[0].virtual_position_ = Vector2(collisionFree[indexPosition][0], collisionFree[indexPosition][1])

				else:

					positionFound = False
					while not positionFound:

						x_sample = random.uniform(lowBound, upperBound)
						y_sample = random.uniform(lowBound, upperBound)

						position = Point(x_sample, y_sample).buffer(formationShapeRadius)

						obstaclesIntersections = []
						for obs in self.obstacles_[1]:

							obstacle = Polygon(obs)
						
							if position.intersects(obstacle):
								obstaclesIntersections.append(True)

							else:
								obstaclesIntersections.append(False)

							del obstacle
						
						del position

						if not any(obstaclesIntersections):

							self.agents_[0].virtual_position_ = Vector2(x_sample, y_sample)

							positionFound = True	

			else:

				del obstacle
				del formation

			return True

	def check_formation(self):

		minDistance = 0.1
		distances = []
		for agent in self.agents_:

			goal = np.array((agent.goal_.x, agent.goal_.y))
			pos = np.array((agent.position_.x, agent.position_.y))
			dist = np.linalg.norm(pos - goal)

			if dist <= minDistance:
				distances.append(True)
			else:
				distances.append(False)

		if all(distances):
			return True
		else:
			return False
	
	def method_based_hungarian(self):

		'''
		Performs a simulation step and updates the 2-D position and 2-D velocity of each agent, based on the
		RRT-kinodyanamic and Hungarian algorithm.
		'''

		for agent in self.agents_:
			agent.get_trajectory_rrt_kinodynamic()

	def method_based_ORCA(self):

		'''
		Performs a simulation step and updates the 2-dimensional position and 2-dimensional velocity of each agent.

		Returns:
			float: The global time after the simulation step.
		'''

		self.kd_tree_.build_agent_tree()
		iterationBeforeChange = self.time_step_/self.default_agent_.delta_t_ - 1

		# TODO: Try async/await here.
		# Performs a simulation step.
		if self.velocity_index_ >= iterationBeforeChange or self.velocity_index_ == 0:

			self.velocity_index_ = 0

			for agentNo in range(self.num_agents):

				self.agents_[agentNo].compute_neighbors()
				self.agents_[agentNo].compute_new_velocity()
				self.agents_[agentNo].smooth_transition()

		# TODO: Try async/await here.
		# Update the 2-dimensional position and 2-dimensional velocity of each agent.
		for agentNo in range(self.num_agents):
			self.agents_[agentNo].update(self.velocity_index_)

		self.velocity_index_ += 1

		# self.global_time_ += self.time_step_
		self.global_time_ += self.default_agent_.delta_t_
		self.default_agent_.times_.append(self.global_time_)

		return self.global_time_

	def method_based_ORCA_RTTk(self):

		'''
		Performs a simulation step and updates the 2-D position and 2-D velocity of each agent, based on the
		RRT-kinodyanamic and ORCA.
		'''

	@property
	def global_time(self):

		# Returns the global time of the simulation.

		return self.global_time_

	@property
	def num_agents(self):

		# Returns the count of agents in the simulation.

		return len(self.agents_)

	@property
	def num_obstacles(self):

		# Returns the count of obstacles in the simulation.

		return len(self.obstacles_[0])

	def process_obstacles(self):

		'''
		Processes the obstacles that have been added so that they are accounted for in the simulation.

		Remarks:
			Obstacles added to the simulation after this function has been called are not accounted for in the simulation.
		'''

		self.kd_tree_.build_obstacle_tree()

	def set_agent_pref_velocity(self, indexControl):

		'''
		Sets the 2-dimensional preferred velocity of a specified agent.

		Args:
			agentNo (int): The number of the agent whose 2-dimensional preferred velocity is to be modified.
			prefVelocity (Vector2): The replacement of the 2-dimensional preferred velocity.
		'''

		for agent in self.agents_:
			# print(agent.positions_velocities_)
			agent.pref_velocity_ = agent.positions_velocities_[indexControl]#*(agent.consensus_t_/50)

	def set_time_step(self, timeStep):

		'''
		Sets the time step of the simulation.

		Args:
			timeStep (float): The time step of the simulation. Must be positive
		'''

		self.time_step_ = timeStep

	def two_points_distance2(self, x1, x2):
		return np.sqrt( (x2[0] - x1[0])**2 + (x2[1] - x1[1])**2 )

	def set_agent_defaults(self, neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed, velocity, alpha, formRadius, limits):

		'''
		Sets the default properties for any new agent that is added.

		Args:
			neighborDist (float): The default maximum distance (center point to center) to other agents a new agent takes into account in the navigation. The larger this number, the longer he running time of the simulation. If the number is too low, the simulation will not be safe. Must be non-negative.
			maxNeighbors (int): The default maximum number of other agents a new agent takes into account in the navigation. the larger this number, the longer the running time of the simulation. If the number is too low, the simulation will not be safe.
			timeHorizon (float): The default minimal amount of the time for which a new agent's velocities that are computed by the simulation are safe with respect to other agents. The larger this number, the sooner an agent will respond to the presence of other agents, but the less freedom the agent has in choosing its velocities. Must be positive.
			timeHorizonObst (float): The default minimal amount of the time for which a new agent's velocities that are computed by the simulation are safe with respect obstacles. The larger this number, the sooner an agent will respond to the presence of obstacles, but the less freedom the agent has in choosing its velocities. Must be positive.
			radius (float): The default radius of a new agent. Must be non-negative.
			maxSpeed (float): The default maximum speed of a new agent. Must be non-negative.
			velocity (Vector2): The default initial 2-dimensional linear velocity of a new agent.
		'''

		if self.default_agent_ is None:
			self.default_agent_ = Agent(self)

		self.default_agent_.max_neighbors_ = maxNeighbors
		self.default_agent_.max_speed_ = maxSpeed
		self.default_agent_.neighbor_dist_ = neighborDist
		self.default_agent_.radius_ = radius
		self.default_agent_.time_horizon_ = timeHorizon
		self.default_agent_.time_horizon_obst_ = timeHorizonObst
		self.default_agent_.velocity_ = velocity
		self.default_agent_.alpha_ = alpha
		self.default_agent_.formation_radius_ = formRadius

		# RRT parameters
		dimensions = np.array([(limits[0]*1.15, limits[1]*1.15), (limits[0]*1.15, limits[1]*1.15)]) # Dimensions of the Search Space
		self.default_agent_.X_ = RRT.SearchSpace(dimensions, radius, self.obstacles_[1])
		self.default_agent_.Q_ = np.array([(8, 4)]) # Length of tree edges
		self.default_agent_.r_ = 1 # Length of smallest edge to check for intersection with obstacles
		self.default_agent_.max_samples_ = 1024 # Max number of samples to take before timing out
		self.default_agent_.prc_ = 0.1 # Probability of checking for a connection to goal

class Timer(object):

	def __init__(self):

		'''
		Constructs and initializes a timer to calculate a process time
		'''
		
		self.tic_ = timeit.default_timer()
		self.toc_ = None
		self.process_time_ = None

	@property
	def tic(self):
		return self.tic_

	@property
	def toc(self):
		return self.toc_

	@property
	def process_time(self):
		return self.process_time_

	def reset_tic(self):
		self.toc_ = timeit.default_timer()

	def set_toc(self):
		self.toc_ = timeit.default_timer()

	def set_process_time_tictoc(self):

		if self.toc_ == None:
			self.set_toc()

		self.process_time_ = self.toc_ - self.tic_
