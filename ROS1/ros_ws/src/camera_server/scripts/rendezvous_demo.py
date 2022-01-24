from ServerWrapper import ServerWrapper
import numpy as np
from rps.utilities.graph import *
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

num_robots = 2

wrapper = ServerWrapper(num_robots)

iterations = 1000

for iteration in range(iterations):

    # Get the position of the robots using the camera server
    current_pos = wrapper.get_position_global()
    current_pos_xy = []
    current_pos_xy.append([x[:1] for x in current_pos])
    print(current_pos)

    vels = []
    for robot in range(num_robots):
        x_sum = 0
        y_sum = 0
        for i in range(num_robots):
            x_sum += current_pos[i][0] - current_pos[robot][0]
            y_sum += current_pos[i][1] - current_pos[robot][1]
        vels.append([x_sum,y_sum])
    
    # passing current pos remove theta
    vels = si_barrier_cert(vels,current_pos_xy)

    vels = si_to_uni_dyn(vels,current_pos_xy)

    wrapper.set_velocities(vels)
    wrapper.step()

