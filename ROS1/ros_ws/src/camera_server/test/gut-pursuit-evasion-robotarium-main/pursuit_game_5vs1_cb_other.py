'''
Author: Qin Yang
05/08/2021
'''

#Import Robotarium Utilities
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
import ServerWrapper
import cv2  
import signal
import sys


def evader_detector(numpursuer, x, id, sensing_distance):
    detector = False

    for i in range(numpursuer):
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

    if np.isnan(catchpointX) or np.isnan(catchpointY):
        catchpoint = oldpoint
        # catchpoint = np.array([envaderPose[0], envaderPose[1]])
        # print(1)
        # print(catchpoint.shape)
    else:
        catchpoint = np.array([[catchpointX], [catchpointY]])

    oldpoint = catchpoint

    return (oldpoint, catchpoint)


N=6 #Number of robots to use, this must stay 4 unless the Laplacian is changed.

wrapper = ServerWrapper.ServerWrapper(N)

def signal_handler(sig, frame):
    wrapper.stop()
    sys.exit()


signal.signal(signal.SIGINT, signal_handler)


def gut_pursuit_game():
    # Experiment Constants
    iterations = 50000 #Run the simulation/experiment for 5000 steps (5000*0.033 ~= 2min 45sec)
    close_enough = 0.03; #How close the leader must get to the waypoint to move to the next one.

    # sensing distance between pursuer and alien
    sensing_distance = 0.3

    # For computational/memory reasons, initialize the velocity vector
    dx_si = np.zeros((2,N))

    #Initialize agent state
    numActiveAlien = 0

    # initial agent's energy and hp
    agent_energy_level = []
    pursuer_evader_distance = []

    for i in range(N):
        # if i < N-1:
        #     agent_energy_level.append(100)
        #     agent_hp_level.append(100)
        # else:
        #     agent_energy_level.append(100)
        #     agent_hp_level.append(100)
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
    # L = cycle_GL(N)

    si_velocities = np.zeros((2, N))

    x =  np.asarray(wrapper.get_data("global_pos")).transpose()# Need robot positions to do this.
    old_x = []

    while len(x[0]) < N:
        x = np.asarray(wrapper.get_data("global_pos")).transpose()
        time.sleep(0.1)

    for i in range(N):
            old_x.append(x[:2, [i]])

    img = wrapper.get_image()
    print(img.shape)

    while img is None:
        img = wrapper.get_image()

    pixel_pos = wrapper.get_data("pixel_pos")

    while not len(pixel_pos) == N:
        pixel_pos = wrapper.get_data("pixel_pos") 

    for index,robot_tag in enumerate(pixel_pos):
        if index == 0:
            cv2.putText(img, "Evader", (robot_tag[0]+25, robot_tag[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        else:
            cv2.putText(img, "Pursuer", (robot_tag[0]+25, robot_tag[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(img, "Energy: " + str(round(agent_energy_level[index], 2)), (robot_tag[0]+25, robot_tag[1]+25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        cv2.putText(img, "Distance: " + str(round(pursuer_evader_distance[index], 2)), (robot_tag[0]+25, robot_tag[1]+50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    wrapper.pub_image(img)

    oldpoint = np.array([[-0.5],[0.121]])

    for k in range(iterations):

        # Get the poses of the robots and convert to single-integrator poses
        x = np.asarray(wrapper.get_data("global_pos")).transpose()
        x_si = uni_to_si_states(x)

        # system parameters
        explorer_system_energy_cost = 500

        waypoints = np.array([[random.uniform(0,2.4)], [random.uniform(0,1.7)]])

        pixel_pos = wrapper.get_data("pixel_pos")

        img = wrapper.get_image()

        while img is None:
            img = wrapper.get_image()

        
        while not len(pixel_pos) == N:
            pixel_pos = wrapper.get_data("pixel_pos") 

        for index,robot_tag in enumerate(pixel_pos):
            if index == 0:
                cv2.putText(img, "Evader", (robot_tag[0]+25, robot_tag[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            else:
                cv2.putText(img, "Pursuer", (robot_tag[0]+25, robot_tag[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(img, "Energy: " + str(round(agent_energy_level[index], 2)), (robot_tag[0]+25, robot_tag[1]+25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            cv2.putText(img, "Distance: " + str(round(pursuer_evader_distance[index], 2)), (robot_tag[0]+25, robot_tag[1]+50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        wrapper.pub_image(img)

        # For each robot...
        for i in range(N):
            # Get the neighbors of robot 'i' (encoded in the graph Laplacian)
            # Compute the pp algorithm
            if i == 0 and k%20 == 0:
                si_velocities[:,i] = np.sum(waypoints[:, 0, None] - x_si[:, i, None], 1)
            if i >= 1:
                oldpoint, catchpoint = get_catchpoint(x[:, 0], x[:, i], oldpoint)

                si_velocities[:, i] = np.sum(catchpoint[:, 0, None] - x_si[:, i, None], 1)

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
            # if i  == 0: # evader
            #     dxu[:,i] = dxu[:,i] * 1.5
            if i >= 1: # pursuer
                dxu[:,i] = dxu[:,i] * 1.05

            # if i==1 and k%100==0: # pursuer
            # if i ==1 and k > 50:
                # dxu[1,i] = random.random() * np.sign(random.uniform(-1, 1)) * 100
                # dxu[1,i] = random.uniform(-100, 100)


        # Set the velocities of agents 1,...,N
        wrapper.set_velocities(dxu.transpose())

        # Calculate agent energy cost
        for i in range(N):
            agent_energy_level[i] -= np.linalg.norm(old_x[i] - x[:2,[i]]) * 10
        # Calculate the distance between pursuer and envader
            if i == 0:
                pursuer_evader_distance[i] = np.linalg.norm(old_x[i] - x[:2,[1]]) * 10
            else:
                pursuer_evader_distance[i] = np.linalg.norm(old_x[i] - x[:2,[0]]) * 10

        # # detect the number of aliens
        # if evader_detector(N-1, x, -1, sensing_distance):
        #     numActiveAlien +=1

        # recode old position
        old_x.clear()

        for i in range(N):
            old_x.append(x[:2, [i]])

        # if (np.array(pursuer_evader_distance) <= 3.5).all():
        if (pursuer_evader_distance[1] + pursuer_evader_distance[2] + pursuer_evader_distance[3] + pursuer_evader_distance[4] + pursuer_evader_distance[5]) / 5 <= 3.3 or agent_energy_level[0] <= 0:
            tmpe = 0

            for i in range(N):
                if i >= 1:
                    tmpe += agent_energy_level[i]

            explorer_system_energy_cost -= tmpe

            print('System energy cost is ' + str(explorer_system_energy_cost))
            print('Time cost is ' + str(k * 0.033))
            wrapper.stop()
            # os._exit(0)

        # Iterate the simulation
        wrapper.step(120)
        time.sleep(.025)
    wrapper.stop()

def main():
    gut_pursuit_game()

if __name__ == '__main__':
    main()