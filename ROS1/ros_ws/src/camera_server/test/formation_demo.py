import sys
import signal
from ServerWrapper import *
import numpy as np
from utilities.graph import *
from utilities.transformations import *
from utilities.barrier_certificates import *
from utilities.misc import *
from utilities.controllers import *

si_barrier_cert = create_single_integrator_barrier_certificate(safety_radius=0.15)

# Create SI to UNI dynamics tranformation
si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()


# Some gains for this experiment.  These aren't incredibly relevant.
d = 0.5
k = 1.0

ddiag_rect = np.sqrt(d**2 + k**2)

ddiag_five_pentagon = .4

ddiag_hexagon = 2*d
hex_internal = np.sqrt(3) * d

ddiag_square = d *np.sqrt(2)

center_six_pentagon = 0.425628265379

u_long_ddiag = .965925826289

d_nine_square_long = np.sqrt((2*d)**2+d**2)

d_nine_square_short = d * np.sqrt(2)

formation_control_gain = 2.0

triangle = [
    [0,d,d],
    [d,0,d],
    [d,d,0]
]

# Weight matrix to control inter-agent distances
weights_five_pentagon = np.array([[0, d, ddiag_five_pentagon, ddiag_five_pentagon,d], 
                    [d, 0, d, ddiag_five_pentagon,ddiag_five_pentagon], 
                    [ddiag_five_pentagon, d, 0, d,ddiag_five_pentagon], 
                    [ddiag_five_pentagon, ddiag_five_pentagon, d, 0,d],
                    [d, ddiag_five_pentagon, ddiag_five_pentagon, d,0]
                    ])

weights_square = np.array([[0,d, ddiag_square, d], 
                           [d, 0, d, ddiag_square], 
                           [ddiag_square, d, 0, d], 
                           [d, ddiag_square, d, 0],
                        ])

weights_hexagon = np.array([[0, d, hex_internal, ddiag_hexagon, hex_internal, d],
                            [d, 0, d, hex_internal, ddiag_hexagon, hex_internal],
                            [hex_internal, d, 0, d, hex_internal, ddiag_hexagon],
                            [ddiag_hexagon, hex_internal, d, 0, d, hex_internal],
                            [hex_internal, ddiag_hexagon, hex_internal, d, 0, d],
                            [d, hex_internal, ddiag_hexagon, hex_internal, d, 0]
                            ])

weights_rect = np.array([[0, k, ddiag_rect, d],
                        [k, 0, d, ddiag_rect],
                        [ddiag_rect, d, 0, k],
                        [d, ddiag_rect, k, 0]
                        ])

weights_six_pentagon = np.array([[0, d, ddiag_five_pentagon, ddiag_five_pentagon,d,center_six_pentagon], 
                    [d, 0, d, ddiag_five_pentagon,ddiag_five_pentagon,center_six_pentagon], 
                    [ddiag_five_pentagon, d, 0, d,ddiag_five_pentagon,center_six_pentagon], 
                    [ddiag_five_pentagon, ddiag_five_pentagon, d, 0,d,center_six_pentagon],
                    [d, ddiag_five_pentagon, ddiag_five_pentagon, d,0,center_six_pentagon],
                    [center_six_pentagon,center_six_pentagon,center_six_pentagon,center_six_pentagon,center_six_pentagon,0]
                    ])

weights_nine_square = [
    [0,d,2*d,d_nine_square_long,2*d*np.sqrt(2),d_nine_square_long,2*d,d,d_nine_square_short],

    [d,0,d,d_nine_square_short,d_nine_square_long,2*d,d_nine_square_long,d_nine_square_short,d],

    [2*d,d,0,d,2*d,d_nine_square_long,2*d*np.sqrt(2),d_nine_square_long,d_nine_square_short],

    [d_nine_square_long,d_nine_square_short,d,0,d,d_nine_square_short,d_nine_square_long,2*d,d],
    
    [2*d*np.sqrt(2),d_nine_square_long,2*d,d,0,d,2*d,d_nine_square_long,d_nine_square_short],
    [d_nine_square_long,2*d,d_nine_square_long,d_nine_square_short,d,0,d,d_nine_square_short,d],
    [2*d,d_nine_square_long,2*d*np.sqrt(2),d_nine_square_long,2*d,d,0,d,d_nine_square_short],
    [d,d_nine_square_short,d_nine_square_long,2*d,d_nine_square_long,d_nine_square_short,d,0,d],
    [d_nine_square_short,d,d_nine_square_short,d,d_nine_square_short,d,d_nine_square_short,d,0]
]

weigths_ten = [
    [0,.559,0,0,0,0,0,0,.559,.25],
    [.559,0,.25,0,0,0,0,0,0,.5],
    [0,.25,0,.25,0,0,0,0,0,0],
    [0,0,.25,0,.559,.5,0,0,0,0],
    [0,0,0,.559,0,.25,.559,0,0,0],
    [0,0,0,.5,.559,0,.5,0,0,0],
    [0,0,0,0,.559,.5,0,.25,0,0],
    [0,0,0,0,0,0,.25,0,.25,0],
    [.559,0,0,0,0,0,0,.25,0,.5],
    [.25,.5,0,0,0,0,0,0,.5,0]
]

shapes = [weights_five_pentagon, weights_square, weights_hexagon, weights_rect, weights_six_pentagon]

demo = weights_nine_square

num_robots = len(demo)
wrapper = ServerWrapper(num_robots)

time.sleep(1)


def signal_handler(sig, frame):
    wrapper.stop()
    sys.exit()


signal.signal(signal.SIGINT, signal_handler)

# for shape in shapes:
iterations = 10000
# print(num_robots)
time.sleep(1)

for iteration in range(iterations):
    print(iteration)
    try:
        # Get the position of the robots using the camera server
        current_pos = np.asarray(wrapper.get_data("global_pos"))
        current_pos_xy = np.asarray([x[:2] for x in current_pos]).transpose()
        # print(current_pos_xy)

        # Initialize a velocity vector
        dxi = np.zeros((2, num_robots))
        for robot in range(num_robots):
            for i in range(num_robots):
                error = current_pos_xy[:2,i] - current_pos_xy[:2,robot]
                dxi[:, robot] += formation_control_gain*(np.power(np.linalg.norm(error), 2)- np.power(demo[robot][i], 2)) * error

        # passing current pos remove theta
        dxi = si_barrier_cert(dxi,current_pos_xy)
        # vels = si_to_uni_dyn(np.asarray(vels).transpose(),np.asarray(current_pos).transpose()) # to use without barrier certificates
        dxu= si_to_uni_dyn(dxi, current_pos.transpose())

        # print(vels)

        wrapper.set_velocities(dxu.transpose())
        wrapper.step(15)

    except Exception as e:
        print(e)
        print("Exiting Code")
        wrapper.stop()
        break

wrapper.stop()
