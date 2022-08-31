import sys
import signal
from ServerWrapper import *
import numpy as np
from rps.utilities.graph import *
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *



num_robots = 5

si_barrier_cert = create_single_integrator_barrier_certificate()

# Create SI to UNI dynamics tranformation
si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()


# Some gains for this experiment.  These aren't incredibly relevant.
d = 0.25
ddiag = .4
formation_control_gain = 2

# Weight matrix to control inter-agent distances
weights = np.array([[0, d, ddiag, ddiag,d], 
                    [d, 0, d, ddiag,ddiag], 
                    [ddiag, d, 0, d,ddiag], 
                    [ddiag, ddiag, d, 0,d],
                    [d, ddiag, ddiag, d,0]
                    ])

wrapper = ServerWrapper(num_robots)


def signal_handler(sig, frame):
    wrapper.stop()


signal.signal(signal.SIGINT, signal_handler)

iterations = 50

for iteration in range(iterations):
    # print(iteration)
    try:
        # Get the position of the robots using the camera server
        current_pos = np.asarray(wrapper.get_data("gloabal_position"))
        current_pos_xy = np.asarray([x[:2] for x in current_pos]).transpose()
        # print(current_pos_xy)

        # Initialize a velocity vector
        dxi = np.zeros((2, num_robots))
        for robot in range(wrapper.get_num_active()):
            for i in range(wrapper.get_num_active()):
                error = current_pos_xy[:2,i] - current_pos_xy[:2,robot]
                dxi[:, robot] += formation_control_gain*(np.power(np.linalg.norm(error), 2)- np.power(weights[robot, i], 2)) * error

        # passing current pos remove theta
        dxi = si_barrier_cert(dxi,current_pos_xy)
        # vels = si_to_uni_dyn(np.asarray(vels).transpose(),np.asarray(current_pos).transpose()) # to use without barrier certificates
        dxu= si_to_uni_dyn(dxi, current_pos.transpose())

        # print(vels)

        wrapper.set_velocities(dxu.transpose())
        wrapper.step(rate=10, time=500)

    except Exception as e:
        print(e)
        print("Exiting Code")
        wrapper.stop()
        break

wrapper.stop()
