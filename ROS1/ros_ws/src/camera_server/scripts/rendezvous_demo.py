from ServerWrapper import ServerWrapper
import numpy as np
from rps.utilities.graph import *
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

num_robots = 4

wrapper = ServerWrapper(num_robots)

iterations = 1000

for iteration in range(iterations):

    # Get the position of the robots using the camera server
    current_pos = wrapper.get_position_global()

    vels = []
    for robot in range(num_robots):
        x_sum = 0
        y_sum = 0
        for i in range(num_robots):
            x_sum += current_pos[i][0] - current_pos[robot][0]
            y_sum += current_pos[i][1] - current_pos[robot][0]
        vels.append([x_sum,y_sum])
    
    vels = si_barrier_cert(vels,current_pos)

    vels = si_to_uni_dyn(vels,current_pos)

    wrapper.set_velocities(vels)
    wrapper.step()

