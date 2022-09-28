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

board_scale = 100

BOARD_SIZE_X = 3 * board_scale
BOARD_SIZE_Y = 1.7 * board_scale

si_barrier_cert = create_single_integrator_barrier_certificate()

# Create SI to UNI dynamics tranformation
si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()


num_robots = 1

xpos = 200#BOARD_SIZE_X * random.random()
ypos = 150#BOARD_SIZE_Y * random.random()

rows = 720
cols = 1080
scale = 1
decay = 4

light_source = np.array([xpos, ypos])
light_source_pixels = np.array([int(xpos*cols/BOARD_SIZE_X),int(ypos*rows/BOARD_SIZE_Y)])
luminosity = 10 #random.randint(1,10)

print("Source Location: {x},{y}".format(x=light_source[0],y=light_source[1]))


# def pop_sensor_old(sensor_array):
#     for i in np.arange(0,BOARD_SIZE_X,BOARD_SIZE_X/cols):
#         for j in np.arange(0,BOARD_SIZE_Y,BOARD_SIZE_Y/rows):
#             distance = (np.sqrt(np.square(i - light_source[0]) + np.square(j - light_source[1])))*scale
#             if distance < 0.001:
#                 sensor_array[int((i*rows)/BOARD_SIZE_X)][int((j*cols)/BOARD_SIZE_Y)] = luminosity
#             else:
#                 sensor_array[int((i*rows)/BOARD_SIZE_X)][int((j*cols)/BOARD_SIZE_Y)] = luminosity - 20*decay*math.log(distance) #(luminosity/(4 * np.pi * (distance ** 2)))
#     wrapper.set_intensity(intensity=sensor_array,max_intensity=luminosity)
#     return sensor_array

# print("BEFORE")
# for i in range(int((rows/2))-20,int((rows/2))+20):
#     for j in range(int(cols/2)-20,int(cols/2)+20):
#         print(sensor_array[j][i],end=" ")
#     print("\n")
def NormalizeData(data):
    return (data - np.min(data)) / (np.max(data) - np.min(data))

def pop_sensor():
    sensor_array = np.zeros((cols,rows))
    for xi in range(0,cols,1):
        for yi in range(0,rows,1):
            distance = (np.sqrt(np.square((xi * BOARD_SIZE_X/cols) - light_source[0]) + np.square((yi * BOARD_SIZE_Y/rows) - light_source[1])))*scale
            if distance < 1:
                # print("X: {x} Y: {y} Distance: {d}".format(x=xi * BOARD_SIZE_X/cols,y=yi * BOARD_SIZE_Y/rows,d=distance))
                sensor_array[xi][yi] = luminosity
            else:
                #if luminosity - 20*decay*math.log(distance) < 0:
                #    sensor_array[xi][yi] = 0
                #else:
                sensor_array[xi][yi] = luminosity - 20*decay*math.log(distance) #(luminosity/(4 * np.pi * (distance ** 2)))
                #sensor_array[xi][yi] = luminosity/(4*np.pi*np.square(distance)) #(luminosity/(4 * np.pi * (distance ** 2)))
    #norm = np.linalg.norm(sensor_array)
    #print(norm)
    sensor_array = NormalizeData(sensor_array)
    return sensor_array

iterations = 1000

sensor_array = pop_sensor()

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

# while True:
#     continue

print("Running")
inverse_scale = 1/board_scale

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
        for robot in range(num_robots):
            c_x = 0
            c_y = 0
            w_xy = 0
            for xi in range(0,cols,5):
                for yi in range(0,rows,5):
                    sensor_value = sensor_array[xi][yi]
                    c_x += ((xi * BOARD_SIZE_X/cols) * sensor_value)
                    c_y += ((yi * BOARD_SIZE_Y/rows) * sensor_value)
                    w_xy += sensor_value
            #w_xy = rows*cols/100
            c_x = c_x/w_xy * inverse_scale
            c_y = c_y/w_xy * inverse_scale
            # print c_x and c_y
            print("Cx: {} | Cy: {}".format(c_x,c_y))
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

    except Exception as e:
        print(e)
        print("Exiting Code")
        wrapper.stop()
        break

wrapper.stop()

# while True:
#     continue