import numpy as np
import math
from shapely.geometry import Polygon, Point

from vector import Vector2

class RRTKinodyn:

	def __init__(self, agent):

		self.agent_ = agent
		self.initial_position_ = (agent.position_.x, agent.position_.y)
		self.goal_position_ = (agent.goal_.x, agent.goal_.y)
		self.initial_velocity_ = (agent.velocity_.x, agent.velocity_.y)
		self.vel_max_ = agent.max_speed_
		self.robot_radius_ = agent.radius_

		self.height_ = agent.limits_[1] + abs(agent.limits_[0])
		self.width_ = agent.limits_[1] + abs(agent.limits_[0])

		self.delta_t_ = 1
		self.T_ = 10
		# self.L = 50 # Largo del carro

		# Solutions info
		self.path_ = [] # Vector2
		self.controls_ = [] # vector2
		self.time_ = 0.0
		self.vertices_ = {self.initial_position_: {'Parent': None, 'Control': self.initial_velocity_, 'Time': 0.0}}

	def random_state(self):

		# radius = self.calculate_metrics(self.initial_position_, self.goal_position_) * 1.5
		# point = radius * np.random.random_sample()
		# angle = 2 * (2 * np.random.random_sample() - 1) * np.pi

		# x = point * np.cos(angle) + self.initial_position_[0]
		# y = point * np.sin(angle) + self.initial_position_[1]

		mean_x = (self.goal_position_[0] + self.initial_position_[0])/2
		mean_y = (self.goal_position_[1] + self.initial_position_[1])/2
		sigma = self.calculate_metrics(self.initial_position_, self.goal_position_)/2

		x = np.random.normal(mean_x, sigma)
		y = np.random.normal(mean_y, sigma)

		# print('show the info in the sampling:\n goal: ', self.goal_position_, '\n init: ', self.initial_position_)
		# print('mean x: ', mean_x,' mean_y: ', mean_y , '\n sigma: ', sigma)
		# print('random sample: ',x, y)
		# input()

		# Orientacion (por ahora no aplica)
		# theta = (2 * np.random.random_sample() - 1) * np.pi 
		# random_state = (x, y, theta)

		# Sin orientacion
		random_state = (x, y)

		return random_state

	def check_if_valid(self, pos, currentPos):

		if self.twoPointsFree(pos, currentPos):
			return True

		else:
			return False

	def twoPointsFree(self, pos, currentPos):

		p1 = currentPos
		p2 = pos
		obs = self.agent_.simulator_.obstacles_[1]

		x = np.linspace(p1[0], p2[0], 30)
		y = np.linspace(p1[1], p2[1], 30)

		for i in range(len(x)):

			collisionFree = self.isCollisionFree(obs, [x[i], y[i]])

			if not collisionFree:
				return collisionFree

		return collisionFree

	def isCollisionFree(self, obs, point):

		pointToCheck = Point(point[0], point[1]).buffer(self.robot_radius_*1.2)

		for i in obs:

			current_obstacle = Polygon(i)

			if pointToCheck.within(current_obstacle) or pointToCheck.intersection(current_obstacle):

				del current_obstacle
				del pointToCheck

				return False

		del current_obstacle
		del pointToCheck

		return True

	def calculate_metrics(self, state1, state2, gamma=0): # Gamma peso (importancia) para el angulo

		# Pesos (importancia) para cada parametro
		alpha = 1 
		beta = 1

		# Considera orientacion (por ahora no aplica)
		metrics = pow(alpha * pow((state1[0] - state2[0]), 2) + beta * pow((state1[1] - state2[1]), 2), 0.5)
					 # + gamma * pow((state1[2] - state2[2]), 2), 0.5)

		return metrics

	def find_closest_state(self, pos):

		'''
		pos: xrand
		'''

		metrics_min = math.sqrt(self.width_**2 + self.height_**2)
		closest = self.initial_position_

		for key in self.vertices_.keys():

			metrics = self.calculate_metrics(pos, key)
			if metrics < metrics_min:

				closest = key 
				metrics_min = metrics

		return closest

	def random_control(self):

		v_lin = np.random.random_sample() * self.vel_max_
		angle = 2 * (2 * np.random.random_sample() - 1) * np.pi

		vx = v_lin * np.cos(angle)
		vy = v_lin * np.sin(angle)

		# Considerando la orientacion
		# turn_angle = (2 * np.random.random_sample() - 1) * np.pi/4
		# return v_lin, turn_angle

		return [vx, vy]

	def new_state(self, u, xk):

		x_new = xk[0] + u[0]*self.delta_t_
		y_new = xk[1] + u[1]*self.delta_t_

		# x_new = xk[0] + u[0]*self.delta_t * np.sin(xk[2])
		# y_new = xk[1] - u[0]*self.delta_t * np.cos(xk[2])
		# theta_new = (u[0] / self.L) * np.tan(u[1]) * self.delta_t + xk[2]
		# new_state = (x_new, y_new, theta_new)

		new_state = (x_new, y_new)

		return new_state

	def extend(self, xnear, xrand):

		# metrics_ref = 2.5
		# metrics_max = self.calculate_metrics(xnear, xrand)
		current_state = xnear
		u = self.random_control()

		for iter in range(int(self.T_/self.delta_t_)):

			x_new = self.new_state(u, current_state)

			if self.check_if_valid(x_new, current_state):
				# if self.calculate_metrics(x_new, xrand) < metrics_max:
				# 	if self.calculate_metrics(x_new, xrand) <= metrics_ref or iter == self.T_/self.delta_t_-1:

				self.vertices_[x_new] = {}
				self.vertices_[x_new]['Parent'] = xnear
				self.vertices_[x_new]['Control'] = u

				# Obteniendo los tiempos
				aux = self.vertices_[x_new]['Parent']
				self.vertices_[x_new]['Time'] = self.delta_t_ + self.vertices_[aux]['Time']
				# print(iter)

				return x_new

				# current_state = x_new
			# metrics_max = self.calculate_metrics(current_state, xrand)

			else:
				return None

	def calculatePath(self, dt, parent, position):

		path = []

		x = np.linspace(parent[0], position[0], 30)
		y = np.linspace(parent[1], position[1], 30)

		for i in range(len(x)):
			path.append((x[i], y[i]))

		return path

	def closerNode(self, count):

		if count >= 100:
			return True
		else:
			return False

	def check_goal(self):

		'''
		Verify and update the goal position to be free collision
		'''

		pointToCheck = Point(self.goal_position_[0], self.goal_position_[1]).buffer(self.robot_radius_*1.2)

		for i in self.agent_.simulator_.obstacles_[1]:

			current_obstacle = Polygon(i)

			if pointToCheck.within(current_obstacle) or pointToCheck.intersection(current_obstacle):

				sample_points = []
				distances = []

				for j in range(100):

					mean_x = i[0][0]
					mean_y = i[0][1]
					sigma = 2.5

					x = np.random.normal(mean_x, sigma)
					y = np.random.normal(mean_y, sigma)

					if self.isCollisionFree([i], [x, y]):

						sample_points.append([x, y])
						distances.append(self.calculate_metrics(sample_points[-1], self.goal_position_))

				max = 100000
				newGoalIndex = 0
				for index, distance in enumerate(distances):
					# print(index, distance)
					if distance < max:
						newGoalIndex = index

				self.goal_position_ = sample_points[newGoalIndex]

	def search(self):

		endReached = False
		startReached = False
		prox = 5
		points = []
		controls = []
		self.path_ = []
		self.controls_ = []
		self.time_ = []
		count = 0

		self.check_goal()

		while not endReached:

			if self.twoPointsFree(self.goal_position_, self.initial_position_):

				self.agent_.get_trajectory_motion_planning()
				self.controls_ = self.agent_.positions_velocities_
				self.path_ = self.agent_.positions_transition_
				self.time_ = self.agent_.times_[-1]

				return

			else:

				closer = self.closerNode(count)
				xrand = self.random_state()
				xnear = self.find_closest_state(xrand)
				xnew = self.extend(xnear, xrand)
				# print(xnew)

				if xnew is not None:
					if self.calculate_metrics(xnew, self.goal_position_) <= prox:

						endReached = True 
						considered_node = xnew 
						controls.append(self.vertices_[considered_node]['Control'])
						self.time_ = self.vertices_[xnew]['Time']

					elif closer:

						endReached = True
						xnew = self.find_closest_state(self.goal_position_)
						considered_node = xnew 
						controls.append(self.vertices_[considered_node]['Control'])
						self.time_ = self.vertices_[xnew]['Time']

					count += 1
					# print(self.calculate_metrics(self.goal_position_, xnew))

		while not startReached:

			points = self.calculatePath(self.delta_t_, self.vertices_[considered_node]['Parent'], considered_node)
			considered_node = self.vertices_[considered_node]['Parent']
			controls.append(self.vertices_[considered_node]['Control'])

			if considered_node[0] == self.initial_position_[0] and considered_node[1] == self.initial_position_[1]:
				startReached = True

		controls.reverse()
		# print('vertices',self.vertices_)
		# print('posiciones',points)
		# print('controles',controls)

		for vel in controls:
			self.controls_.append(Vector2(vel[0], vel[1]))
		for position in points:
			self.path_.append(Vector2(position[0], position[1]))
			# print('Agente: ', self.agent_.id_, 'posicion:', position)

		# print(self.controls_)