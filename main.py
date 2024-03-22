'''
Multiagent Formation by Path Planning V-3.0.1

	Centro de Investigacion y de Estudios Avanzados del I.P.N.
					Unidad Guadalajara.

	By L. Enrique Ruiz-Fernandez, 2022-09.

	This version improves:
        - The RRT applying the RRT Kinodynamic which adds.
          a robot dynamical constraints (car-like robot, differential driverobot
          and free particle).
        - Adds the timer to check de process time.

	Version 2D.
'''

# Simulation and plot libraries.
from gettext import translation
import matplotlib.pyplot as plt
import matplotlib.cm as cmx
import gym.envs.classic_control.rendering as rendering
import math
import numpy as np

# Own package
from vector import Vector2
from simulator import Simulator
import obstacle
import random_config as r_conf

RVO_RENDER = True

class MultiAgent_Formation:

    def __init__(self, random=False):
        
        self.simulator_ = Simulator()
        self.obstacles_ = [[], []] # [Vector2, Vertex]
        self.agents_number_ = 0
        self.limits_ = [] # [min_limit, max_limit]
        self.random_config = random # Bool
        self.colors_ = None
        self.control_index_ = 0
        self.iterations_ = 0
        self.new_update_ = True
        self.plot_goal_ = False

        self.stop_ = False

    def setup_scenario(self, withObstacles=True, randomObstacles=False):

        # Specify the global time step of the simulation.
        self.simulator_.set_time_step(0.1)

        # Number of agents
        self.agents_number_ = 5

        # Limits of the environment
        self.limits_ = [-10, 10]
        mapLenght = abs(self.limits_[0]) + abs(self.limits_[1])

        # Formation shape (at the moment it is a cirlce)
        alpha = (2*math.pi)/self.agents_number_
        formationDistance = 0.1*mapLenght
        formationRadius = formationDistance/(2*math.sin(alpha/2))

        # Specify the default parameters for agents that are subsequently added.
        self.simulator_.set_agent_defaults(
            0.9*mapLenght, # Neighbor distance
            10, # Max neighbors
            0.05, # Time horizon (agents)
            0.05, # Time horizon (obstacles)
            0.02*mapLenght, # Radius
            0.02*mapLenght, # Max Speed
            Vector2(), # Velocity
            alpha, # Angle between agents
            formationRadius, # Distance from the centroid of the formation
            self.limits_ # Limits of the environment
        )

        if self.random_config:

            if withObstacles:
                # r_conf.obstacles_generation(self, 1, 0.1*mapLenght) # (Number of obstacles, obstacle range)
                r_conf.obstacles_generation_noconvex(self, 9, 0.1*mapLenght)
                
            r_conf.set_random_positions(self)
            
            # Set fixed positions
            # for i in range(self.agents_number_):
            #     self.simulator_.add_agent(self.limits_[1]*Vector2(math.cos(i*2.0*math.pi/(self.agents_number_)),
            #                                                       math.sin(i*2.0*math.pi/(self.agents_number_))))

        else:
            
            # Set fixed positions
            for i in range(self.agents_number_):
                self.simulator_.add_agent(self.limits_[1]*Vector2(math.cos(i*2.0*math.pi/(self.agents_number_)),
                                                                  math.sin(i*2.0*math.pi/(self.agents_number_))))
            if withObstacles:

                if not randomObstacles:
                    # obstacle.obstacles_generation(self)
                    # obstacle.obstacles_generation_local_minima(self)
                    obstacle.obstacles_generation_U_obs(self)
                
                else:
                    r_conf.obstacles_generation(self, 45, 0.017*mapLenght) # (Number of obstacles, obstacle range)

            # Set fixed positions
            # for i in range(self.agents_number_):
            #     self.simulator_.add_agent(self.limits_[1]*Vector2(math.cos(i*2.0*math.pi/(self.agents_number_)),
            #                                                       math.sin(i*2.0*math.pi/(self.agents_number_))))

            


        self.simulator_.set_graph_neighbors_id()
        self.simulator_.get_virtual_positions()
        self.colors_ = cmx.rainbow(np.linspace(0, 1, self.agents_number_))

    def update_visualization(self, viewer, showTree, pasue):

        if not RVO_RENDER:
            return

        for i in range(self.agents_number_):

            position = self.simulator_.agents_[i].position_
            color = self.colors_[i,:3]
            circle = viewer.draw_circle(radius=self.simulator_.default_agent_.radius_, color=color)
            circle.add_attr(rendering.Transform(translation=(position.x, position.y)))

            # goal = self.simulator_.agents_[i].goal_
            # color = self.colors_[i,:3]/2.5
            # circle_goal = viewer.draw_circle(radius=self.simulator_.default_agent_.radius_, color=color)
            # circle_goal.add_attr(rendering.Transform(translation=(goal.x, goal.y)))

            virtualPosition = self.simulator_.agents_[i].virtual_position_
            virtualColor = [0.7, 0.7, 0.7]
            virtualCircle = viewer.draw_circle(radius=self.simulator_.default_agent_.radius_*0.7, color=virtualColor)
            virtualCircle.add_attr(rendering.Transform(translation=(virtualPosition.x, virtualPosition.y)))

            # Show the nodes and tree of the RRT
            if showTree:

                # viewer.reset()

                for j in range(1, len(self.simulator_.agents_[i].current_path_)):

                    vertex = self.simulator_.agents_[i].current_path_[j]
                    parent = self.simulator_.agents_[i].current_path_[j-1]

                    # node = viewer.draw_circle(radius=self.simulator_.default_agent_.radius_/3, color=color)
                    # node.add_attr(rendering.Transform(translation=(vertex[0], vertex[1])))

                    line = rendering.Line((vertex), (parent))
                    line.set_color(color[0], color[1], color[2])
                    viewer.add_geom(line)
                    # viewer.add_geom(node)

            if self.plot_goal_:

                # Vertex to draw the goal
                goal = self.simulator_.agents_[i].goal_
                angle = 2*np.pi/4
                vertex = []
                for i in range(1, 5):
                    x = self.simulator_.default_agent_.radius_*np.cos(i*angle)
                    y = self.simulator_.default_agent_.radius_*np.sin(i*angle)
                    vertex.append((x, y))

                color = color/1.5
                square_goal = viewer.draw_polygon(vertex, color=color)
                square_goal.add_attr(rendering.Transform(translation=(goal.x, goal.y)))

        self.plot_goal_ = True

        for obstacle in self.obstacles_[0]:

            v = [(vec.x, vec.y) for vec in obstacle]
            viewer.draw_polygon(v=v, color=(0, 0, 0))

        viewer.render()

        if pasue:
            input()

    def reaching_formation(self, viewers, pause=False):

        '''
        Set the velocities of the agents to reach the formation, avoiding collision (if is required)
        '''

        if not self.new_update_:
            return

        # Get the consensus reference
        if self.iterations_ == 0 or self.control_index_ >= 12:
        
            self.control_index_ = 0
            self.simulator_.get_consensus_velocities()
            self.simulator_.get_trajectory_current_goal()
            # input()

            # # Comment this to have just one window!!!!
            # if self.iterations_ != 0:
            #     viewers.append(None)
            #     # input()

            # input()
        self.simulator_.set_agent_pref_velocity(self.control_index_)

        self.control_index_ += 1

        return pause

    def plot_all(self):

        fig, axs = plt.subplots(2, 1)
        fig.suptitle('Agents velocities with SORCA', fontsize=16)
        axs[0].set_ylabel('Velocity X \n [m/s]')
        axs[1].set_ylabel('Velocity Y \n [m/s]')
        axs[0].set_xlabel('Time [1x10^-2 sec]')
        axs[1].set_xlabel('Time [1x10^-2 sec]')

        fig_, axs_ = plt.subplots(2, 1)
        fig_.suptitle('Agents velocities with ORCA', fontsize=16)
        axs_[0].set_ylabel('Velocity X \n [m/s]')
        axs_[1].set_ylabel('Velocity Y \n [m/s]')
        axs_[0].set_xlabel('Time [1x10^-2 sec]')
        axs_[1].set_xlabel('Time [1x10^-2 sec]')

        fig_acc, axs_acc = plt.subplots(2, 1)
        fig_acc.suptitle('Agents accelerations', fontsize=16)
        axs_acc[0].set_ylabel('Acceleration X \n [m/s^2]')
        axs_acc[1].set_ylabel('Acceleration Y \n [m/s^2]')
        axs_acc[0].set_xlabel('Time [1x10^-2 sec]')
        axs_acc[1].set_xlabel('Time [1x10^-2 sec]')

        fig_jerk, axs_jerk = plt.subplots(2, 1)
        fig_jerk.suptitle('Agents Jerk', fontsize=16)
        axs_jerk[0].set_ylabel('Jerk X \n [m/s^3]')
        axs_jerk[1].set_ylabel('Jerk Y \n [m/s^3]')
        axs_jerk[0].set_xlabel('Time [1x10^-2 sec]')
        axs_jerk[1].set_xlabel('Time [1x10^-2 sec]')

        fig_errors, axs_errors = plt.subplots(2, 1)
        fig_errors.suptitle('Agents position error', fontsize=16)
        axs_errors[0].set_ylabel('Error X')
        axs_errors[1].set_ylabel('Error Y')
        axs_errors[0].set_xlabel('Time [1x10^-2 sec]')
        axs_errors[1].set_xlabel('Time [1x10^-2 sec]')

        fig_cons, axs_cons = plt.subplots(2, 1)
        fig_cons.suptitle('Virtual Agents Consensus', fontsize=16)
        axs_cons[0].set_ylabel('Position X')
        axs_cons[1].set_ylabel('Position Y')
        axs_cons[0].set_xlabel('Time [1x10^-2 sec]')
        axs_cons[1].set_xlabel('Time [1x10^-2 sec]')

        for i, agent in enumerate(self.simulator_.agents_):

            # Velocities
            vel = np.array(agent.all_velocities_)
            vel_ = np.array(agent.all_velocities_discontinuous_)
            x = vel[:,0]
            y = vel[:,1]
            x_ = vel_[:,0]
            y_ = vel_[:,1]

            # Acclerations
            acc = np.array(agent.all_accelerations_)
            x_acc = acc[:,0]
            y_acc = acc[:,1]

            # Jerks
            jerk = np.array(agent.all_jerks_)
            x_jerk = jerk[:,0]
            y_jerk = jerk[:,1]

            color = self.colors_[i,:3]
            t_smooth = np.linspace(0, agent.times_[-1], len(agent.all_velocities_))
            t_ = np.linspace(0, agent.times_[-1], len(agent.all_velocities_discontinuous_))

            axs[0].plot(t_smooth, x, color=color)
            axs[1].plot(t_smooth, y, color=color)

            axs_[0].plot(t_, x_, color=color)
            axs_[1].plot(t_, y_, color=color)

            axs_acc[0].plot(t_smooth, x_acc, color=color)
            axs_acc[1].plot(t_smooth, y_acc, color=color)

            axs_jerk[0].plot(t_smooth, x_jerk, color=color)
            axs_jerk[1].plot(t_smooth, y_jerk, color=color)

            # Errors and consensus
            error = np.array(agent.all_errors_)
            x_e = error[:,0]
            y_e = error[:,1]
            cons = np.array(agent.all_virtual_positions_)
            x_c = cons[:,0]
            y_c = cons[:,1]

            axs_errors[0].plot(t_smooth, x_e, color=color)
            axs_errors[1].plot(t_smooth, y_e, color=color)
            axs_cons[0].plot(t_smooth, x_c, color=color)
            axs_cons[1].plot(t_smooth, y_c, color=color)

        plt.show()

def main():

    recording = False
    pause = False
    viewers = [None]
    # formation = MultiAgent_Formation(True) # Bool: random configuration (default = False)
    formation = MultiAgent_Formation() # Bool: random configuration (default = Fals= []
		# obstacle_3vec = []

		# obstacle_3.append((current_x, current_y))
		# current_x += height*cos(angle + pi)
		# current_y += height*sin(angle + pi)
		# obstacle_3.append((current_x, current_y))
		# current_x += width*cos(angle + pi/2)
		# current_y += width*sin(angle + pi/2)
		# obstacle_3.append((current_x, current_y))
		# current_x += height*cos(angle)
		# current_y += height*sin(angle)
		# obstacle_3.append((current_x, current_y))

		# print("\fPared 3: ")
		# for i in obstacle_3:
		# 	print(i)
		# 	obstacle_3vec.append(Vector2(i[0], i[1]))

		# formationSim.obstacles_[0].append(obstacle_3vec)
		# formationSim.obstacles_[1].append(obstacle_3)
		# formationSim.simulator_.add_obstacle(obstacle_3vec)
		# formationSim.simulator_.obstacles_[1].append(obstacle_3)
    # Set up the scenario
    formation.setup_scenario(True) # Bool: with obstacles or not (default = False)
                                         # Bool: obstacles random (default = False)

    # Perform (and manipulate) the simulation
    count = 0
    while not formation.stop_:

        checkFormation = False
        
        if RVO_RENDER:
            if viewers[-1] is None:
                viewers[-1] = rendering.Viewer(750, 750)
                viewers[-1].set_bounds(formation.limits_[0]*1.2, formation.limits_[1]*1.2,
                                  formation.limits_[0]*1.2, formation.limits_[1]*1.2)

            formation.update_visualization(viewers[-1], False, pause) # bool: Ask if want to show the RRT Tree 

            if not recording:

                ans = input("Is it recording? [Y/N]: ")

                if ans == 'Y' or ans == 'y':
                    recording = True

        pause = formation.reaching_formation(viewers)
        formation.new_update_, increase = formation.simulator_.set_velocities_to_formation(formation.control_index_, formation.iterations_)
        checkFormation = formation.simulator_.check_formation_collision()
        if checkFormation:
            formation.stop_ = formation.simulator_.check_formation()
        formation.iterations_ += increase
        formation.simulator_.simulation_begins_ = False

        if formation.stop_:

            formation.stop_ = False
            if count >= 250:
                formation.stop_ = True
            count += 1


    sum_times = 0
    for time in formation.simulator_.trajectory_time_:
        sum_times += time
    average_times = sum_times/len(formation.simulator_.trajectory_time_)
    print('Time avergae: ', average_times)

    formation.plot_all()

if __name__ == '__main__':
    main()