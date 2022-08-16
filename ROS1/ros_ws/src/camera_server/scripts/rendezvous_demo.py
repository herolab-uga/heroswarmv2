from ServerWrapper import *
import numpy as np
from rps.utilities.graph import *
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

si_barrier_cert = create_single_integrator_barrier_certificate()

# Create SI to UNI dynamics tranformation
si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()

import signal
import sys


num_robots = 3

wrapper = ServerWrapper(num_robots)

def signal_handler(sig, frame):
	wrapper.stop()    

signal.signal(signal.SIGINT, signal_handler)

def main_algorithm():
	# Get the position of the robots using the camera server
	current_pos = wrapper.get_position_global()
	current_pos_xy = [x[:2] for x in current_pos]
	# print(current_pos_xy)

	vels = []
	for robot in range(wrapper.get_active()):
		x_sum = 0
		y_sum = 0
		for i in range(wrapper.get_active()):
			x_sum += current_pos[i][0] - current_pos[robot][0]
			y_sum += current_pos[i][1] - current_pos[robot][1]
		vels.append([x_sum,y_sum])

	# passing current pos remove theta
	#vels = si_barrier_cert(np.asarray(vels).transpose(),np.asarray(current_pos_xy).transpose())

	#vels = si_to_uni_dyn(np.asarray(vels).transpose(),np.asarray(current_pos).transpose()) # to use without barrier certificates
	vels = si_to_uni_dyn(vels,np.asarray(current_pos).transpose())

	wrapper.set_velocities(vels.transpose())
	wrapper.step(rate=10,time=100)

iterations = 100000

for iteration in range(iterations):
	try:
		while True:
			# Get the position of the robots using the camera server
			current_pos = wrapper.get_position_global()
			current_pos_xy = [x[:2] for x in current_pos]
			# print(current_pos_xy)

			vels = []
			for robot in range(wrapper.get_active()):
				x_sum = 0
				y_sum = 0
				for i in range(wrapper.get_active()):
					x_sum += current_pos[i][0] - current_pos[robot][0]
					y_sum += current_pos[i][1] - current_pos[robot][1]
				vels.append([x_sum,y_sum])

			# passing current pos remove theta
			#vels = si_barrier_cert(np.asarray(vels).transpose(),np.asarray(current_pos_xy).transpose())

			vels = si_to_uni_dyn(np.asarray(vels).transpose(),np.asarray(current_pos).transpose()) # to use without barrier certificates
			# vels = si_to_uni_dyn(vels,np.asarray(current_pos).transpose())

			wrapper.set_velocities(vels.transpose())
			wrapper.step(rate=10,time=100)
	except KeyboardInterrupt:
		print("Exiting Code")
		break
wrapper.stop()

