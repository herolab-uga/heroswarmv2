import sys
import signal
import random
import math
from ServerWrapper import *
import numpy as np
from utilities.graph import *
from utilities.transformations import *
from utilities.barrier_certificates import *
from utilities.misc import *
from utilities.controllers import *

BOARD_SIZE_X = 2.4
BOARD_SIZE_Y = 1.7

si_barrier_cert = create_single_integrator_barrier_certificate()

# Create SI to UNI dynamics tranformation
si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()


num_robots = 1
wrapper = ServerWrapper(num_robots)

time.sleep(1)


def signal_handler(sig, frame):
    wrapper.stop()
    sys.exit()


signal.signal(signal.SIGINT, signal_handler)


xpos = BOARD_SIZE_X/2#BOARD_SIZE_X * random.random()
ypos = BOARD_SIZE_Y/2#BOARD_SIZE_Y * random.random()
light_source = np.array([xpos, ypos])
luminosity = 1 #random.randint(1,10)

print("Source Location: {x},{y}".format(x=light_source[0]+.1,y=light_source[1]))

rows = 720
cols = 1080
scale = 1
decay = 4
sensor_array = np.zeros((rows,cols))

def pop_sensor(sensor_array):
    for i in np.arange(0,BOARD_SIZE_X,BOARD_SIZE_X/cols):
        for j in np.arange(0,BOARD_SIZE_Y,BOARD_SIZE_Y/rows):
            distance = (np.sqrt(np.square(i - light_source[0]) + np.square(j - light_source[1])))*scale
            if distance < 0.001:
                sensor_array[int((i*rows)/BOARD_SIZE_X)][int((j*cols)/BOARD_SIZE_Y)] = luminosity
            else:
                sensor_array[int((i*rows)/BOARD_SIZE_X)][int((j*cols)/BOARD_SIZE_Y)] = luminosity - 20*decay*math.log(distance) #(luminosity/(4 * np.pi * (distance ** 2)))
    wrapper.set_intensity(intensity=sensor_array,max_intensity=luminosity)
    return sensor_array

iterations = 10000

sensor_array = pop_sensor(sensor_array)

print("Running")

for iteration in range(iterations):
    # print(iteration)
    try:
        # Get the position of the robots using the camera server
        current_pos = np.asarray(wrapper.get_data("global_pos"))
        current_pos_xy = np.asarray([x[:2] for x in current_pos]).transpose()
        # Initialize a velocity vector
        dxi = []
        for robot in range(num_robots):
            c_x = 0
            c_y = 0
            w_xy = 0
            for xi in range(0,cols,10):
                for yi in range(0,rows,10):
                    c_x += (xi * BOARD_SIZE_X/cols) * sensor_array[yi][xi] 
                    c_y += (yi * BOARD_SIZE_Y/rows) * sensor_array[yi][xi] 
                    w_xy += sensor_array[yi][xi]
            c_x = c_x/w_xy
            c_y = c_y/w_xy
            # print c_x and c_y
            print("Cx: {} | Cy: {}".format(c_x,c_y))
            print("Robot current pos: {} ".format(current_pos[robot]))

            dxi.append([c_x - current_pos[robot][0], c_y - current_pos[robot][1]])

        dxi = np.asarray(dxi).transpose()
        # print(dxi)
        # passing current pos remove theta
        dxi = si_barrier_cert(dxi,current_pos_xy)
        # vels = si_to_uni_dyn(np.asarray(vels).transpose(),np.asarray(current_pos).transpose()) # to use without barrier certificates
        dxu= si_to_uni_dyn(dxi, current_pos.transpose())

        # print(dxu)

        wrapper.set_velocities(dxu.transpose())
        wrapper.step(60)

    except Exception as e:
        print(e)
        print("Exiting Code")
        wrapper.stop()
        break

wrapper.stop()

# while True:
#     continue