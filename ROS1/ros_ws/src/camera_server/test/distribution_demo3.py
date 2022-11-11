#!/usr/env/bin python3
import numpy as np
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
from matplotlib import patches
from rps.utilities.graph import *
from ServerWrapper import ServerWrapper
import cv2
import sys
import signal

N = 6 #number of robots
wrapper = ServerWrapper(N)
iterations = 5000
si_barrier_cert = create_single_integrator_barrier_certificate()
# Create SI to UNI dynamics tranformation
si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()
# Generated a connected graph Laplacian (for a cylce graph).
L = completeGL(N)
# r = robotarium.Robotarium(number_of_robots=N, show_figure=True, sim_in_real_time=True)


x_min_robotarium = 0
x_max_robotarium = 3.178175

y_min_robotarium = 0
y_max_robotarium = 1.74

board_scale = 1000 # to align the units in cm
inverse_scale = 1/board_scale
light_sources = [
                    [0.2*x_max_robotarium*board_scale, 0.65*y_max_robotarium*board_scale],
                    [0.75*x_max_robotarium*board_scale, 0.42*y_max_robotarium*board_scale],
                    [0.3*x_max_robotarium*board_scale, 0.375*y_max_robotarium*board_scale]
                ]

decay = 3
luminosity = [500,500,500]
scale = 1
k=0.2 # learning rate

#BOARD_SIZE_X = 3 * board_scale
#BOARD_SIZE_Y = 2 * board_scale # 1.76

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    wrapper.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def NormalizeData(data):
    return (data - np.min(data)) / (np.max(data) - np.min(data))

# Distance between two points
def dist(pos1,pos2):
    return np.sqrt(np.square(pos1[0]-pos2[0]) + np.square(pos1[1]-pos2[1]))

def pop_sensor_mesh(h,w,iter=0):

    x = np.linspace(x_min_robotarium*board_scale, x_max_robotarium*board_scale, w)
    y = np.linspace(y_min_robotarium*board_scale, y_max_robotarium*board_scale, h)
    X, Y = np.meshgrid(x, y,sparse=False)
    Z = np.zeros((h,w))
    for k in range(len(light_sources)):
        dist = np.sqrt(np.square(X -light_sources[k][0] + int(iter/40)*20) + np.square(Y -light_sources[k][1] + int(iter/40)*20)) 
        Z += luminosity[k] - 20*decay*np.log(dist) #(luminosity[k]/(4 * np.pi * (dist ** decay)))
    return Z

image = wrapper.get_image()
h,width = image.shape[:2]
colormap = cv2.applyColorMap(cv2.flip(pop_sensor_mesh(h,width).astype(np.uint8),0), cv2.COLORMAP_JET)
img = cv2.addWeighted(colormap, 0.5, image, 0.5, 0)
wrapper.pub_image(img)
cols = width # 1080
rows = h

def get_sensor(x,y,iter): #x,y in cm
    svalue = 0
    for k in range(len(light_sources)) :
        dist = np.sqrt(np.square(x -light_sources[k][0] + int(iter/40)*20 ) + np.square(y -light_sources[k][1] + int(iter/40)*20) ) 
        if dist < 50:
            svalue = svalue + luminosity[k]
        else:
            # svalue = luminosity[k] - 20*decay*math.log(dist) #(luminosity/(4 * np.pi * (distance ** 2)))
            svalue = svalue + (luminosity[k]/(4 * np.pi * (dist ** decay))) # luminosity/(dist**2) #

    return svalue

c_v = np.zeros((N,2))
w = np.zeros((N))

x_res = np.linspace(x_min_robotarium,x_max_robotarium, cols)
y_res = np.linspace(y_min_robotarium,y_max_robotarium, rows)

for k in range(iterations):
    img = wrapper.get_image()
    colormap = cv2.applyColorMap(cv2.flip(pop_sensor_mesh(h,width,k).astype(np.uint8),0), cv2.COLORMAP_JET)
    img = cv2.addWeighted(colormap, 0.5, img, 0.5, 0)
    wrapper.pub_image(img)

    # Get the poses of the robots and convert to single-integrator poses
    current_pos = np.asarray(wrapper.get_data("global_pos"))
    # print(current_pos)
    current_pos = current_pos.transpose()
    # current_pos_si = uni_to_si_states(current_pos)#.transpose()    
    current_x = current_pos[0, :, None]        
    current_y = current_pos[1, :, None]
    # print(current_pos)
    dxi = np.zeros((2,N))
    c_x = 0    
    c_y = 0
    #print(current_x[0][0],current_x[1][0],current_x[2][0])
                
    # Below is a working version as of Nov 1, 2022
    for xi in np.arange(x_min_robotarium,x_max_robotarium,0.05):
        for yi in np.arange(y_min_robotarium,y_max_robotarium,0.05):
            sensor_value = get_sensor(xi*board_scale,yi*board_scale,k)
            # print("X: {x} | Y: {y} | Sensor: {s}".format(x=xi,y=yi, s=sensor_value))
            distances = np.zeros((N))
            for robot in range(N):
                distances[robot] =  np.sqrt(np.square(xi - current_x[robot][0]) + np.square(yi - current_y[robot][0]) ) 
            min_index = np.argmin(distances)
            c_v[min_index][0] += (xi * sensor_value)
            c_v[min_index][1] += (yi * sensor_value)
            w[min_index] += sensor_value
    #print(w)
    	            
    for robot in range(N):
        c_x = c_v[robot][0]/(w[robot])
        c_y = c_v[robot][1]/(w[robot])
        dxi[:,robot] = [k*(c_x - current_x[robot][0]), k*(c_y - current_y[robot][0] )] 
        #dxi[:,robot] = [goto_pos[robot][0] - current_pos_si[0, robot, None], goto_pos[robot][1] - current_pos_si[1, robot, None]]
        # print("Robot: " ,robot)
        # print("c_x :",c_x)
        # print("c_y :",c_y)
        # print(get_sensor(c_x*board_scale,c_y*board_scale))

    #dxi = si_barrier_cert(np.asarray(dxi).transpose(),np.asarray(current_pos_si).transpose())
    #dxi = si_barrier_cert(dxi, current_pos_si)
    dxi = si_to_uni_dyn(dxi, current_pos)

    # print(dxu)
    #print(k)
    wrapper.set_velocities(dxi.transpose())
    wrapper.step(60)

#Call at end of script to print debug information and for your script to run on the Robotarium server properly
wrapper.stop()


