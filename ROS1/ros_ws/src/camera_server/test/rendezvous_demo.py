from ServerWrapper import *
import numpy as np
from utilities.graph import *
from utilities.transformations import *
from utilities.barrier_certificates import *
from utilities.misc import *
from utilities.controllers import *

si_barrier_cert = create_single_integrator_barrier_certificate()

# Create SI to UNI dynamics tranformation
si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()

import signal
import sys


num_robots = 9

wrapper = ServerWrapper(num_robots)

def signal_handler(sig, frame):
	wrapper.stop()    

signal.signal(signal.SIGINT, signal_handler)

iterations = 10000

for iteration in range(iterations):
	print("Running iteration {}".format(iteration))
	try:
		# Get the position of the robots using the camera server
		current_pos = wrapper.get_data("global_pos")
		current_pos_xy = [x[:2] for x in current_pos]
		print(current_pos_xy)

		vels = []
		for robot in range(wrapper.get_num_active()):
			x_sum = 0
			y_sum = 0
			for i in range(wrapper.get_num_active()):
				x_sum += current_pos[i][0] - current_pos[robot][0]
				y_sum += current_pos[i][1] - current_pos[robot][1]
			vels.append([x_sum,y_sum])

		# passing current pos remove theta
		#print(np.asarray(vels).transpose())
		#print(np.asarray(current_pos_xy).transpose())
		vels = si_barrier_cert(np.asarray(vels).transpose(),np.asarray(current_pos_xy).transpose())
		# vels = si_to_uni_dyn(np.asarray(vels).transpose(),np.asarray(current_pos).transpose()) # to use without barrier certificates
		vels = si_to_uni_dyn(vels,np.asarray(current_pos).transpose())

		print(vels)


		wrapper.set_velocities(vels.transpose())
		wrapper.step(rate=10)

	except Exception as e:
		print(e)
		print("Exiting Code")
		wrapper.stop()
		break

wrapper.stop()

