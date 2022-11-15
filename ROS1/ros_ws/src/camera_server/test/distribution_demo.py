import sys
import signal
import random
import math
import traceback
from ServerWrapper import *
import numpy as np
from utilities.graph import *
from utilities.transformations import *
from utilities.barrier_certificates import *
from utilities.misc import *
from utilities.controllers import *

board_scale = 100

BOARD_SIZE_X = 3 * board_scale
BOARD_SIZE_Y = 1.7 * board_scale

si_barrier_cert = create_single_integrator_barrier_certificate()

# Create SI to UNI dynamics tranformation
si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()

rows = 720
cols = 1080
scale = 1
decay = 4
num_sources = 1

# Create a list of light sources
light_sources = []
light_source_pixels = []
for i in range(num_sources):

    xpos = random.uniform(0,2.5 * board_scale)
    ypos = random.uniform(0,1.7 * board_scale)
    light_sources.append([xpos,ypos])
    light_source_pixels.append([int(xpos * cols/BOARD_SIZE_X),int(ypos * rows/BOARD_SIZE_Y)])

luminosity = 10 #random.randint(1,10)

for source in range(len(light_sources)):
    print("Source: {source_number} | Source Location: {x},{y}".format(source_number=source, x=light_sources[source][0],y=light_sources[source][1]))

def NormalizeData(data):
    return (data - np.min(data)) / (np.max(data) - np.min(data))

def pop_sensor():
    sensor_array = np.zeros((cols,rows))
    for source in range(len(light_sources)):
        for xi in range(0,cols,1):
            for yi in range(0,rows,1):
                distance = (np.sqrt(np.square((xi * BOARD_SIZE_X/cols) - light_sources[source][0]) + np.square((yi * BOARD_SIZE_Y/rows) - light_sources[source][1])))*scale
                if distance < 1:
                    # print("X: {x} Y: {y} Distance: {d}".format(x=xi * BOARD_SIZE_X/cols,y=yi * BOARD_SIZE_Y/rows,d=distance))
                    sensor_array[xi][yi] = luminosity
                else:
                    sensor_array[xi][yi] = luminosity - 20*decay*math.log(distance) #(luminosity/(4 * np.pi * (distance ** 2)))
    sensor_array = NormalizeData(sensor_array)
    return sensor_array

iterations = 1000

sensor_array = pop_sensor()

num_robots = 3

wrapper = ServerWrapper(num_robots)

time.sleep(1)


def signal_handler(sig, frame):
    wrapper.stop()
    sys.exit()


signal.signal(signal.SIGINT, signal_handler)

wrapper.set_intensity(intensity=sensor_array)

# print("AFTER")
# for i in range(int(cols/2)-10,int(cols/2)+10):
#     for j in range(int(rows/2)-10,int(rows/2)+10):
#         print("{:.2f}".format(sensor_array[i][j]),end=" ")
#     print("\n")

# print("AFTER")
# for i in range(light_source_pixels[0]-10,light_source_pixels[0]+10):
#     for j in range(light_source_pixels[1]-10,light_source_pixels[1]+10):
#         print("{:.2f}".format(sensor_array[i][j]),end=" ")
#     print("\n")

print("Running")
inverse_scale = 1/board_scale
c_v = np.zeros((num_robots,2))
w = np.zeros((num_robots))

# Distance between two points
def dist(pos1,pos2):
    return np.sqrt(np.square(pos1[0]-pos2[0]) + np.square(pos1[1]-pos2[1]))

for iteration in range(iterations):

    # print("AFTER")
    # for i in range(light_source_pixels[0]-10,light_source_pixels[0]+10):
    #     for j in range(light_source_pixels[1]-10,light_source_pixels[1]+10):
    #         print("{:.2f}".format(sensor_array[i][j]),end=" ")
    #     print("\n")

    # print(iteration)
    try:
        # Get the position of the robots using the camera server
        current_pos = np.asarray(wrapper.get_data("global_pos"))
        current_pos_xy = np.asarray([x[:2] for x in current_pos]).transpose()
        # Initialize a velocity vector
        dxi = []
        c_x = 0
        c_y = 0
        w_xy = 0
        for xi in range(0,cols,10):
            for yi in range(0,rows,10):
                sensor_value = sensor_array[xi][yi]
                distances = np.zeros((num_robots))
                for robot in range(num_robots):
                    distances[robot] = dist(current_pos[robot]*board_scale,np.array([xi*BOARD_SIZE_X/cols,yi*BOARD_SIZE_Y/rows]))
                min_index = np.argmin(distances)
                c_v[min_index][0] += ((xi * BOARD_SIZE_X/cols) * sensor_value)
                c_v[min_index][1] += ((yi * BOARD_SIZE_Y/rows) * sensor_value)
                w[min_index] += sensor_value
        #w_xy = rows*cols/100

        for robot in range(num_robots):
            c_x = c_v[robot][0]/w[robot] * inverse_scale
            c_y = c_v[robot][1]/w[robot] * inverse_scale
            # print c_x and c_y
            # print("Cx: {} | Cy: {}".format(c_x,c_y))
            # print("Robot current pos: {} ".format(current_pos[robot]))

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

    except Exception:
        print(traceback.format_exc())
        print("Exiting Code")
        wrapper.stop()
        break

wrapper.stop()

# while True:
#     continue