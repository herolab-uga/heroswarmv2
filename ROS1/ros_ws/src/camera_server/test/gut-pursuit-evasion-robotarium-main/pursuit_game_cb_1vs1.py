'''
Author: Qin Yang
05/08/2021
'''

#Import Robotarium Utilities
import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.graph import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
import numpy as np
from fractions import Fraction
import time
import random
import os

def alien_detector(numExplorer, x, id, sensing_distance):
    detector = False

    for i in range(numExplorer):
        if np.linalg.norm(x[:2,[i]] - x[:2,[id]]) < sensing_distance:
            detector = True

    return detector

def get_catchpoint(envaderPose, pursuerPose, oldpoint):
    bearing_angle = np.arctan(2 * ((envaderPose[1] - pursuerPose[1]) / (envaderPose[0] - pursuerPose[0])))
    theta = bearing_angle + np.arcsin(np.sin(envaderPose[2] - bearing_angle) / 1.5)
    # theta = bearing_angle + np.arcsin(np.sin(envaderPose[2] - bearing_angle) * 3)

    tanEnvader = np.tan(envaderPose[2])
    ctgEnvader = 1 / tanEnvader

    tanPursuer = np.tan(theta)
    # tanPursuer = np.tan(pursuerPose[2])
    ctgPursuer = 1 / tanPursuer

    catchpointX = (envaderPose[0] * tanEnvader - pursuerPose[0] * tanPursuer + pursuerPose[1] - envaderPose[1]) / (tanEnvader - tanPursuer)
    catchpointY = (envaderPose[1] * ctgEnvader - pursuerPose[1] * ctgPursuer + pursuerPose[0] - envaderPose[0]) / (ctgEnvader - ctgPursuer)

    # catchpointX = (envaderPose[0] * ctgPursuer + pursuerPose[0] * ctgEnvader + pursuerPose[1] - envaderPose[1]) / (ctgEnvader + ctgPursuer)
    # catchpointY = (envaderPose[1] * ctgPursuer + pursuerPose[1] * ctgEnvader + pursuerPose[0] - envaderPose[0]) / (ctgEnvader + ctgPursuer) 

    if np.isnan(catchpointX) or np.isnan(catchpointY):
        catchpoint = oldpoint
        # catchpoint = np.array([envaderPose[0], envaderPose[1]])
        # print(1)
        # print(catchpoint.shape)
    else:
        # catchpoint = np.array([[-3.31376639], [-0.85912109]])
        catchpoint = np.array([catchpointX, catchpointY])
        # print(catchpoint)

    oldpoint = catchpoint

    return (oldpoint, catchpoint)

def pursuit_game_cb():
    N = 2
    # r = robotarium.Robotarium(number_of_robots=N, show_figure=True, sim_in_real_time=True)

    initial_conditions = np.array([[-1.4, 1.333],[-0.8, 0.830],[0, 1]])

    r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions, sim_in_real_time=True)

    # How many iterations do we want (about N*0.033 seconds)
    iterations = 10000

    # sensing distance between explorer and alien
    sensing_distance = 0.3

    numActiveAlien = 0

    # initial agent's energy and hp
    agent_energy_level = []
    pursuer_evader_distance = []

    for i in range(N):
        if i == 1:
            agent_energy_level.append(150)
        else:
            agent_energy_level.append(100)
  
        pursuer_evader_distance.append(0)


    #Max_simum linear speed of robot specified by motors
    magnitude_limit = 0.1

    # We're working in single-integrator dynamics, and we don't want the robots
    # to collide or drive off the testbed.  Thus, we're going to use barrier certificates
    si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary()

    # Create SI to UNI dynamics tranformation
    si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()

    # Generated a connected graph Laplacian (for a cylce graph).
    L = cycle_GL(N)

    si_velocities = np.zeros((2, N))

    CM1 = np.random.rand(N,3)
    CM2 = np.random.rand(N,3)
    CM3 = np.random.rand(N,3)
    marker_size_goal = determine_marker_size(r,0.2)
    robot_marker_size_m = 0.35
    font_size_m = 0.1
    font_size = determine_font_size(r,font_size_m)
    font_size_m1 = 0.06
    font_size1 = determine_font_size(r,font_size_m1)
    font_size_m2 = 0.04
    font_size2 = determine_font_size(r,font_size_m2)
    marker_size_robot = determine_marker_size(r, robot_marker_size_m)
    line_width = 5

    # Plot Graph Connections
    x = r.get_poses() # Need robot positions to do this.
    old_x = []

    for i in range(N):
        old_x.append(initial_conditions[:2, [i]])

    pursuer_label = r.axes.text(x[0,0],x[1,0]+0.25,"pursuer",fontsize=font_size1, color='b',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=0)
    pursuer_energy_label = r.axes.text(x[0,0],x[1,0]+0.2,"NRG: ",fontsize=font_size2, color='c',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=0)
    pursuer_hp_label = r.axes.text(x[0,0],x[1,0]+0.15,"Dist: ",fontsize=font_size2, color='m',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=0)

    evader_label = r.axes.text(x[0,1],x[1,1]+0.25,"evader",fontsize=font_size1, color='r',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=0)
    evader_energy_label = r.axes.text(x[0,1],x[1,1]+0.2,"NRG: ",fontsize=font_size2, color='c',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=0)
    evader_hp_label = r.axes.text(x[0,1],x[1,1]+0.15,"Dist: ",fontsize=font_size2, color='m',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=0)

    r.step()

    oldpoint = np.array([[-1.4],[-0.8]])

    for k in range(iterations):

        # Get the poses of the robots and convert to single-integrator poses
        x = r.get_poses()
        # print(x)
        x_si = uni_to_si_states(x)

        waypoints = np.array([[random.uniform(-1.4, 1.4)], [random.uniform(-1.4, 1.4)]])
        # waypoints = np.array([[0.5], [0.5]])

        pursuer_label.set_position([x_si[0,0],x_si[1,0]+0.25])
        pursuer_label.set_fontsize(determine_font_size(r,font_size_m1))
        pursuer_energy_label.set_position([x_si[0,0],x_si[1,0]+0.2])
        pursuer_energy_label.set_fontsize(determine_font_size(r,font_size_m2))
        pursuer_energy_label.set_text("NRG: " + str(round(agent_energy_level[0], 2)))
        pursuer_hp_label.set_position([x_si[0,0],x_si[1,0]+0.15])
        pursuer_hp_label.set_fontsize(determine_font_size(r,font_size_m2))
        pursuer_hp_label.set_text("Dist: " + str(round(pursuer_evader_distance[0], 2)))

        evader_label.set_position([x_si[0,1],x_si[1,1]+0.25])
        evader_label.set_fontsize(determine_font_size(r,font_size_m1))
        evader_energy_label.set_position([x_si[0,1],x_si[1,1]+0.2])
        evader_energy_label.set_fontsize(determine_font_size(r,font_size_m2))
        evader_energy_label.set_text("NRG: " + str(round(agent_energy_level[1], 2)))
        evader_hp_label.set_position([x_si[0,1],x_si[1,1]+0.15])
        evader_hp_label.set_fontsize(determine_font_size(r,font_size_m2))
        evader_hp_label.set_text("Dist: " + str(round(pursuer_evader_distance[1], 2)))

        # Initialize the single-integrator control inputs
        #si_velocities = np.zeros((2, N))

        # For each robot...
        for i in range(N):
            # Get the neighbors of robot 'i' (encoded in the graph Laplacian)
            j = topological_neighbors(L, i)
            # Compute the cb algorithm
            if i == 0:
                oldpoint, catchpoint = get_catchpoint(x[:, j], x[:, i], oldpoint)

                si_velocities[:, i] = np.sum(catchpoint[:, 0, None] - x_si[:, i, None], 1)
                # si_velocities[:, i] = np.sum(get_catchpoint(x[:, j], x[:, i])[:, 0, None] - x_si[:, i, None], 1)
                # print(x_si[:, j])
                # si_velocities[:, i] = np.sum(x_si[:, j] - x_si[:, i, None], 1)
            if i == 1 and k%20 == 0:
                si_velocities[:, i] = np.sum(waypoints[:, 0, None] - x_si[:, i, None], 1)

        # #Keep single integrator control vectors under specified magnitude
        # # Threshold control inputs
        norms = np.linalg.norm(si_velocities, 2, 0)
        idxs_to_normalize = (norms > magnitude_limit)
        si_velocities[:, idxs_to_normalize] *= magnitude_limit/norms[idxs_to_normalize]

        # Use the barrier certificate to avoid collisions
        si_velocities = si_barrier_cert(si_velocities, x_si)

        # Transform single integrator to unicycle
        dxu = si_to_uni_dyn(si_velocities, x)

        for i in range(N):
            # if i  == 1: # evader
            #     dxu[:,i] = dxu[:,i] * 1.5
            if i == 0: # pursuer
                dxu[:,i] = dxu[:,i] * 1.05

            # if i==1 and k%100==0: # pursuer
            # if i ==1 and k > 50:
                # dxu[1,i] = random.random() * np.sign(random.uniform(-1, 1)) * 100
                # dxu[1,i] = random.uniform(-100, 100)


        # Set the velocities of agents 1,...,N
        r.set_velocities(np.arange(N), dxu)

        # Calculate agent energy cost
        for i in range(N):
            agent_energy_level[i] -= np.linalg.norm(old_x[i] - x[:2,[i]]) * 10
        # Calculate the distance between pursuer and envader
            if i == 0:
                pursuer_evader_distance[i] = np.linalg.norm(old_x[i] - x[:2,[1]]) * 10
            else:
                pursuer_evader_distance[i] = np.linalg.norm(old_x[i] - x[:2,[0]]) * 10

        # detect the number of aliens
        if alien_detector(N-1, x, -1, sensing_distance):
            numActiveAlien +=1

        # recode old position
        old_x.clear()

        for i in range(N):
            old_x.append(x[:2, [i]])

        if pursuer_evader_distance[0] <= 3:
            print('cost time is ' + str(k))
            os._exit(0)

        # Iterate the simulation
        r.step()


def main():
    pursuit_game_cb()

if __name__ == '__main__':
    main()