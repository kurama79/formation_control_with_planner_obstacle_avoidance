from dis import dis
from operator import index
from random import randrange, uniform, gauss
from tracemalloc import start
from shapely.geometry import Polygon, Point, LineString
import numpy as np
from scipy.linalg import null_space

from vector import Vector2

# Define the initial positions randomly
def set_random_positions(formationSim):

    validPositions = False
    while not validPositions:

        auxPositions = []
        for i in range(formationSim.agents_number_):

            freePoint = False
            while not freePoint:

                x = uniform(formationSim.limits_[0]*1.1, formationSim.limits_[1]*1.1)
                y = uniform(formationSim.limits_[0]*1.1, formationSim.limits_[1]*1.1)

                if formationSim.obstacles_[1]:

                    point = Point(x, y).buffer(formationSim.simulator_.default_agent_.radius_*1.0)

                    for obs in formationSim.obstacles_[1]:

                        shapeObs = Polygon(obs)

                        if not point.within(shapeObs) and not point.intersection(shapeObs):
                            freePoint = True

                        else:
                            freePoint = False
                            break 

                        del shapeObs

                else:
                    freePoint = True

            auxPositions.append([x, y])
        # validPositions = True
        validPositions, initialPositions = neighbor_verification(formationSim, auxPositions, formationSim.simulator_.default_agent_.neighbor_dist_, formationSim.agents_number_)

    for position in initialPositions:
        formationSim.simulator_.add_agent(Vector2(position[0], position[1]))

# Check the connectivity
def neighbor_verification(formationSim, positions, radius, n):

    A = np.zeros((n, n)) # Adjacency matrix
    D = np.zeros((n, n)) # Incidence matrix

    starts = []
    for i in range(n):
        starts.append(positions[i])

    for i in range(n - 1):

        neighborCount = 0
        for j in range(i + 1, n):

            dist = formationSim.simulator_.two_points_distance2(starts[i], starts[j])
            if dist < radius:

                A[j][i] = 1
                A[i][j] = 1

                neighborCount += 1

                if neighborCount == 2:
                    break

    for i in range(n):
        D[i][i] = sum(A[i][:])

    # Laplacian
    L = D - A
    Lkernel = null_space(L)

    if Lkernel.shape[1] == 1:
        connected = True

    else:
        connected = False

    if connected:

        for i in range(n-2):

            distances = []
            for j in range(i, n):

                if j == n - 1:
                    break

                else:
                    dist = formationSim.simulator_.two_points_distance2(starts[i][:], starts[j+1][:])
                    distances.append(dist)

            index = distances.index(min(distances))

            aux = starts[i + 1][:]
            starts[i + 1][:] = starts[index + 1][:]
            starts[index + 1][:] = aux

    return connected, starts

def position_in_collision(x, obs, r):

    # Check if a point is in collision with an obstacle
    robot = Point(x.x, x.y).buffer(r)
    obstacle = Polygon(obs)

    if robot.within(obstacle) and robot.intersection(obstacle):
        return True

    return False

# Generate and define the obstacles
def obstacles_generation(formationSim, n, obstacleRange):

    for _ in range(n):

        collision = True
        while collision:

            # vertex_number = randrange(10) + 3
            centroid = [uniform(formationSim.limits_[0], formationSim.limits_[1]), 
                        uniform(formationSim.limits_[0], formationSim.limits_[1])]
            vertex = []

            # for j in range(vertex_number):

            #     aux = [gauss(centroid[0], centroid[0]*1.5),
            #            gauss(centroid[1], centroid[1]*1.5)]
            #     dist = formationSim.simulator_.two_points_distance2(aux, centroid)

            #     while dist > obstacleRange:
                    
            #         aux = [gauss(centroid[0], centroid[0]*1.5),
            #                gauss(centroid[1], centroid[1]*1.5)]
            #         dist = formationSim.simulator_.two_points_distance2(aux, centroid)

            #     vertex.append((aux[0], aux[1]))

            # if vertex:
            #     vertex = sort_vertex(vertex)
            
            angle = 2*(2*uniform(0, 1) - 1)*np.pi
            addAngle = np.pi/2
            distanceToVertex = obstacleRange
            for j in range(4):
                
                x = centroid[0] + distanceToVertex*np.cos(angle)
                y = centroid[1] + distanceToVertex*np.sin(angle)

                angle += addAngle
                vertex.append((x, y))

            positionCheck = []
            for agent in formationSim.simulator_.agents_:

                position = agent.position_
                positionCheck.append(position_in_collision(position, vertex, agent.radius_))

            if all(positionCheck):
                collision = False

        vertex_vec = []
        for j in vertex:
            vertex_vec.append(Vector2(j[0], j[1]))

        formationSim.obstacles_[0].append(vertex_vec)
        formationSim.obstacles_[1].append(vertex)
        formationSim.simulator_.add_obstacle(vertex_vec)
        formationSim.simulator_.obstacles_[1].append(vertex)

        formationSim.simulator_.process_obstacles()

def obstacles_generation(formationSim, n, obstacleRange):

    for _ in range(n):

        collision = True
        while collision:

            # vertex_number = randrange(10) + 3
            centroid = [uniform(formationSim.limits_[0], formationSim.limits_[1]), 
                        uniform(formationSim.limits_[0], formationSim.limits_[1])]
            vertex = []

            # for j in range(vertex_number):

            #     aux = [gauss(centroid[0], centroid[0]*1.5),
            #            gauss(centroid[1], centroid[1]*1.5)]
            #     dist = formationSim.simulator_.two_points_distance2(aux, centroid)

            #     while dist > obstacleRange:
                    
            #         aux = [gauss(centroid[0], centroid[0]*1.5),
            #                gauss(centroid[1], centroid[1]*1.5)]
            #         dist = formationSim.simulator_.two_points_distance2(aux, centroid)

            #     vertex.append((aux[0], aux[1]))

            # if vertex:
            #     vertex = sort_vertex(vertex)
            
            angle = 2*(2*uniform(0, 1) - 1)*np.pi
            addAngle = np.pi/2
            distanceToVertex = obstacleRange
            for j in range(4):
                
                x = centroid[0] + distanceToVertex*np.cos(angle)
                y = centroid[1] + distanceToVertex*np.sin(angle)

                angle += addAngle
                vertex.append((x, y))

            positionCheck = []
            for agent in formationSim.simulator_.agents_:

                position = agent.position_
                positionCheck.append(position_in_collision(position, vertex, agent.radius_))

            if all(positionCheck):
                collision = False

        vertex_vec = []
        for j in vertex:
            vertex_vec.append(Vector2(j[0], j[1]))

        formationSim.obstacles_[0].append(vertex_vec)
        formationSim.obstacles_[1].append(vertex)
        formationSim.simulator_.add_obstacle(vertex_vec)
        formationSim.simulator_.obstacles_[1].append(vertex)

        formationSim.simulator_.process_obstacles()
        
def obstacles_generation_noconvex(formationSim, n, obstacleRange):

    for _ in range(n):

        collision = True
        while collision:

            # vertex_number = randrange(10) + 3
            centroid = [uniform(formationSim.limits_[0], formationSim.limits_[1]), 
                        uniform(formationSim.limits_[0], formationSim.limits_[1])]
            vertex = []
            vertex_number = randrange(10) + 3

            for j in range(vertex_number):

                aux = [gauss(centroid[0], centroid[0]*1.5),
                       gauss(centroid[1], centroid[1]*1.5)]
                dist = formationSim.simulator_.two_points_distance2(aux, centroid)

                while dist > obstacleRange:
                    
                    aux = [gauss(centroid[0], centroid[0]*1.5),
                           gauss(centroid[1], centroid[1]*1.5)]
                    dist = formationSim.simulator_.two_points_distance2(aux, centroid)

                vertex.append((aux[0], aux[1]))

            if vertex:
                vertex = sort_vertex(vertex)
            
            # angle = 2*(2*uniform(0, 1) - 1)*np.pi
            # addAngle = np.pi/2
            # distanceToVertex = obstacleRange
            # for j in range(4):
                
            #     x = centroid[0] + distanceToVertex*np.cos(angle)
            #     y = centroid[1] + distanceToVertex*np.sin(angle)

            #     angle += addAngle
            #     vertex.append((x, y))

            positionCheck = []
            for agent in formationSim.simulator_.agents_:

                position = agent.position_
                positionCheck.append(position_in_collision(position, vertex, agent.radius_))

            if all(positionCheck):
                collision = False

        vertex_vec = []
        for j in vertex:
            vertex_vec.append(Vector2(j[0], j[1]))

        formationSim.obstacles_[0].append(vertex_vec)
        formationSim.obstacles_[1].append(vertex)
        formationSim.simulator_.add_obstacle(vertex_vec)
        formationSim.simulator_.obstacles_[1].append(vertex)

        formationSim.simulator_.process_obstacles()
        

def sort_vertex(vertex):

    centroid = np.mean(vertex, axis=0)
    vertex = np.array(vertex)
    d = vertex - centroid
    angles = np.arctan2(d[:,0], d[:,1])
    oder = vertex[np.argsort(angles)]

    for i in range(1, len(oder)-1):

        if np.arctan((centroid[1] - oder[i + 1][1])/(centroid[0] - oder[i + 1][0])) == np.arctan((centroid[1] - oder[i][1])/(centroid[0] - oder[i][0])):

            angle = np.arctan2(oder[i - 1][1] - oder[i][1], oder[i - 1][0] - oder[i][0])

            if angle == np.pi/2 or angle == -np.pi/2 or angle == np.pi or angle == -np.pi or angle == 0:
                continue

            else:

                temp = [oder[i + 1][0], oder[i + 1][1]]
                oder[i + 1], oder[i] = oder[i], temp

    return oder

