from turtle import Vec2D
from numpy import pi, cos, sin

from vector import Vector2


class Obstacle:

	# Defines static obstacles in the simulation

	def __init__(self):

		self.next_ = None
		self.previous_ = None
		self.direction_ = None
		self.point_ = None
		self.id_ = -1
		self.convex_ = False

# Generate and define the obstacles
def obstacles_generation(formationSim):

	# Fixed obstacles and predefined
	obstacle_1 = []
	obstacle_1Vec = []
	obstacle_1.append((-0.1*formationSim.limits_[1], 0.4*formationSim.limits_[1]))
	obstacle_1.append((-0.4*formationSim.limits_[1], 0.4*formationSim.limits_[1]))
	obstacle_1.append((-0.4*formationSim.limits_[1], 0.1*formationSim.limits_[1]))
	obstacle_1.append((-0.1*formationSim.limits_[1], 0.1*formationSim.limits_[1]))
	for i in obstacle_1:
		obstacle_1Vec.append(Vector2(i[0], i[1]))
	formationSim.obstacles_[0].append(obstacle_1Vec)
	formationSim.obstacles_[1].append(obstacle_1)
	formationSim.simulator_.add_obstacle(obstacle_1Vec)
	formationSim.simulator_.obstacles_[1].append(obstacle_1)

	obstacle_2 = []
	obstacle_2Vec = []
	obstacle_2.append((0.1*formationSim.limits_[1], 0.4*formationSim.limits_[1]))
	obstacle_2.append((0.1*formationSim.limits_[1], 0.1*formationSim.limits_[1]))
	obstacle_2.append((0.4*formationSim.limits_[1], 0.1*formationSim.limits_[1]))
	obstacle_2.append((0.4*formationSim.limits_[1], 0.4*formationSim.limits_[1]))
	for i in obstacle_2:
		obstacle_2Vec.append(Vector2(i[0], i[1]))
	formationSim.obstacles_[0].append(obstacle_2Vec)
	formationSim.obstacles_[1].append(obstacle_2)
	formationSim.simulator_.add_obstacle(obstacle_2Vec)
	formationSim.simulator_.obstacles_[1].append(obstacle_2)

	obstacle_3 = []
	obstacle_3Vec = []
	obstacle_3.append((0.1*formationSim.limits_[1], -0.4*formationSim.limits_[1]))
	obstacle_3.append((0.4*formationSim.limits_[1], -0.4*formationSim.limits_[1]))
	obstacle_3.append((0.4*formationSim.limits_[1], -0.1*formationSim.limits_[1]))
	obstacle_3.append((0.1*formationSim.limits_[1], -0.1*formationSim.limits_[1]))
	for i in obstacle_3:
		obstacle_3Vec.append(Vector2(i[0], i[1]))
	formationSim.obstacles_[0].append(obstacle_3Vec)
	formationSim.obstacles_[1].append(obstacle_3)
	formationSim.simulator_.add_obstacle(obstacle_3Vec)
	formationSim.simulator_.obstacles_[1].append(obstacle_3)

	obstacle_4 = []
	obstacle_4Vec = []
	obstacle_4.append((-0.1*formationSim.limits_[1], -0.4*formationSim.limits_[1]))
	obstacle_4.append((-0.1*formationSim.limits_[1], -0.1*formationSim.limits_[1]))
	obstacle_4.append((-0.4*formationSim.limits_[1], -0.1*formationSim.limits_[1]))
	obstacle_4.append((-0.4*formationSim.limits_[1], -0.4*formationSim.limits_[1]))
	for i in obstacle_4:
		obstacle_4Vec.append(Vector2(i[0], i[1]))
	formationSim.obstacles_[0].append(obstacle_4Vec)
	formationSim.obstacles_[1].append(obstacle_4)
	formationSim.simulator_.add_obstacle(obstacle_4Vec)
	formationSim.simulator_.obstacles_[1].append(obstacle_4)

	formationSim.simulator_.process_obstacles()

# Generate and define the obstacles to enclose in local minima
def obstacles_generation_local_minima(formationSim):

	# Fixed obstacles and predefined
	# cornerDist = 0.1*formationSim.limits_[1]
	height, width = 0.03*formationSim.limits_[1], 0.2*formationSim.limits_[1]

	for agent in formationSim.simulator_.agents_:

		print('Obstaculo ', agent.id_)

		pos_x = agent.position_.x
		pos_y = agent.position_.y
		angle = (agent.alpha_) - pi/2
		# angle = (agent.alpha_) - 3*pi/4
		current_x = pos_x + (width/2)*cos(angle)
		current_y = pos_y + (width/2)*sin(angle)

		# Add first part of the obstacle
		obstacle_1 = []
		obstacle_1vec = []

		obstacle_1.append((current_x, current_y))
		current_x += height*cos(angle)
		current_y += height*sin(angle)
		obstacle_1.append((current_x, current_y))
		current_x += width*cos(angle - pi/2)
		current_y += width*sin(angle - pi/2)
		obstacle_1.append((current_x, current_y))
		current_x += height*cos(angle + pi)
		current_y += height*sin(angle + pi)
		obstacle_1.append((current_x, current_y))

		print("\fPared 1: ")
		for i in obstacle_1:
			print(i)
			obstacle_1vec.append(Vector2(i[0], i[1]))

		formationSim.obstacles_[0].append(obstacle_1vec)
		formationSim.obstacles_[1].append(obstacle_1)
		formationSim.simulator_.add_obstacle(obstacle_1vec)
		formationSim.simulator_.obstacles_[1].append(obstacle_1)

		# Add second part
		obstacle_2 = []
		obstacle_2vec = []

		obstacle_2.append((current_x, current_y))
		current_x += height*cos(angle + pi/2)
		current_y += height*sin(angle + pi/2)
		obstacle_2.append((current_x, current_y))
		current_x += width*cos(angle + pi)
		current_y += width*sin(angle + pi)
		obstacle_2.append((current_x, current_y))
		current_x += height*cos(angle - pi/2)
		current_y += height*sin(angle - pi/2)
		obstacle_2.append((current_x, current_y))

		print("\fPared 2: ")
		for i in obstacle_2:
			print(i)
			obstacle_2vec.append(Vector2(i[0], i[1]))

		formationSim.obstacles_[0].append(obstacle_2vec)
		formationSim.obstacles_[1].append(obstacle_2)
		formationSim.simulator_.add_obstacle(obstacle_2vec)
		formationSim.simulator_.obstacles_[1].append(obstacle_2)

		# Add third part
		obstacle_3 = []
		obstacle_3vec = []

		obstacle_3.append((current_x, current_y))
		current_x += height*cos(angle + pi)
		current_y += height*sin(angle + pi)
		obstacle_3.append((current_x, current_y))
		current_x += width*cos(angle + pi/2)
		current_y += width*sin(angle + pi/2)
		obstacle_3.append((current_x, current_y))
		current_x += height*cos(angle)
		current_y += height*sin(angle)
		obstacle_3.append((current_x, current_y))

		print("\fPared 3: ")
		for i in obstacle_3:
			print(i)
			obstacle_3vec.append(Vector2(i[0], i[1]))

		formationSim.obstacles_[0].append(obstacle_3vec)
		formationSim.obstacles_[1].append(obstacle_3)
		formationSim.simulator_.add_obstacle(obstacle_3vec)
		formationSim.simulator_.obstacles_[1].append(obstacle_3)

	formationSim.simulator_.process_obstacles()
	# input()
 
def obstacles_generation_U_obs(formationSim):

	# Fixed obstacles and predefined
	# cornerDist = 0.1*formationSim.limits_[1]
	height, width = 0.1*formationSim.limits_[1], 0.25*formationSim.limits_[1]
	center = [(formationSim.limits_[0]+formationSim.limits_[1])//2, (formationSim.limits_[0]+formationSim.limits_[1])//2]
	radius = formationSim.simulator_.default_agent_.formation_radius_ * 3

	positions = [[center[0] + radius*cos(0), center[1] + radius*sin(0), 0],
				 [center[0] + radius*cos(pi/2), center[1] + radius*sin(pi/2), pi/2],
				 [center[0] + radius*cos(pi), center[1] + radius*sin(pi), pi],
				 [center[0] + radius*cos(3*pi/2), center[1] + radius*sin(3*pi/2), 3*pi/2]]
 
	for pos in positions:

		pos_x = pos[0]
		pos_y = pos[1]
		angle = pos[2] - pi/2

		current_x = pos_x + (width/2)*cos(angle)
		current_y = pos_y + (width/2)*sin(angle)

		# Add first part of the obstacle
		obstacle_1 = []
		obstacle_1vec = []

		obstacle_1.append((current_x, current_y))
		current_x += height*cos(angle)
		current_y += height*sin(angle)
		obstacle_1.append((current_x, current_y))
		current_x += width*cos(angle - pi/2)
		current_y += width*sin(angle - pi/2)
		obstacle_1.append((current_x, current_y))
		current_x += height*cos(angle + pi)
		current_y += height*sin(angle + pi)
		obstacle_1.append((current_x, current_y))

		print("\fPared 1: ")
		for i in obstacle_1:
			print(i)
			obstacle_1vec.append(Vector2(i[0], i[1]))

		formationSim.obstacles_[0].append(obstacle_1vec)
		formationSim.obstacles_[1].append(obstacle_1)
		formationSim.simulator_.add_obstacle(obstacle_1vec)
		formationSim.simulator_.obstacles_[1].append(obstacle_1)

		# Add second part
		obstacle_2 = []
		obstacle_2vec = []

		obstacle_2.append((current_x, current_y))
		current_x += height*cos(angle + pi/2)
		current_y += height*sin(angle + pi/2)
		obstacle_2.append((current_x, current_y))
		current_x += width*cos(angle + pi)
		current_y += width*sin(angle + pi)
		obstacle_2.append((current_x, current_y))
		current_x += height*cos(angle - pi/2)
		current_y += height*sin(angle - pi/2)
		obstacle_2.append((current_x, current_y))

		print("\fPared 2: ")
		for i in obstacle_2:
			print(i)
			obstacle_2vec.append(Vector2(i[0], i[1]))

		formationSim.obstacles_[0].append(obstacle_2vec)
		formationSim.obstacles_[1].append(obstacle_2)
		formationSim.simulator_.add_obstacle(obstacle_2vec)
		formationSim.simulator_.obstacles_[1].append(obstacle_2)

		# Add third part
		obstacle_3 = []
		obstacle_3vec = []

		obstacle_3.append((current_x, current_y))
		current_x += height*cos(angle + pi)
		current_y += height*sin(angle + pi)
		obstacle_3.append((current_x, current_y))
		current_x += width*cos(angle + pi/2)
		current_y += width*sin(angle + pi/2)
		obstacle_3.append((current_x, current_y))
		current_x += height*cos(angle)
		current_y += height*sin(angle)
		obstacle_3.append((current_x, current_y))

		print("\fPared 3: ")
		for i in obstacle_3:
			print(i)
			obstacle_3vec.append(Vector2(i[0], i[1]))

		# formationSim.obstacles_[0].append(obstacle_3vec)
		# formationSim.obstacles_[1].append(obstacle_3)
		# formationSim.simulator_.add_obstacle(obstacle_3vec)
		# formationSim.simulator_.obstacles_[1].append(obstacle_3)

	formationSim.simulator_.process_obstacles()

'''
Gazebo Obstacles:
	Minima, first wall:
		

'''