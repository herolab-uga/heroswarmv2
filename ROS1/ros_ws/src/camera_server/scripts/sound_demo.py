import sys
import signal
from ServerWrapper import *
import numpy as np
from rps.utilities.graph import *
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *


num_robots = 4

si_barrier_cert = create_single_integrator_barrier_certificate()

# Create SI to UNI dynamics tranformation
si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()

wrapper = ServerWrapper(num_robots)


def signal_handler(sig, frame):
    wrapper.stop()


signal.signal(signal.SIGINT, signal_handler)

iterations = 100    

for iteration in range(iterations):
    print(iteration)
    try:
        # Get the position of the robots using the camera server
        current_pos = np.asarray(wrapper.get_data("global_pos"))
        current_pos_xy = np.asarray([x[:2] for x in current_pos]).transpose()

        mic_data = wrapper.get_data("mic")
        print(mic_data)
        max_index = np.argmax(mic_data)
        print(max_index)

        dxi = []
        for robot in range(wrapper.get_num_active()):
            x_sum = 0
            y_sum = 0
            for i in range(wrapper.get_num_active()):
                x_sum += current_pos[max_index][0] - current_pos[robot][0]
                y_sum += current_pos[max_index][1] - current_pos[robot][1]
            dxi.append([x_sum, y_sum])

        # passing current pos remove theta
        dxi = si_barrier_cert(np.asarray(dxi).transpose(), current_pos_xy)
        # vels = si_to_uni_dyn(np.asarray(vels).transpose(),np.asarray(current_pos).transpose()) # to use without barrier certificates
        dxu = si_to_uni_dyn(dxi, current_pos.transpose())

        # print(vels)

        wrapper.set_velocities(dxu.transpose())
        wrapper.step(rate=10, time=500)

    except Exception as e:
        print(e)
        print("Exiting Code")
        wrapper.stop()
        break

wrapper.stop()
