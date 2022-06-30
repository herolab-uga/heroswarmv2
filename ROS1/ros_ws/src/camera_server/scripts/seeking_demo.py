from ServerWrapper import *
import numpy as np
from rps.utilities.graph import *
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

def main():
    # Collision avoidance
    si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary()

    # Create SI to UNI dynamics tranformation
    si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()

    num_robots = 5 # Number of robots to use

    wrapper = ServerWrapper(num_robots) # Swarm warpper object

    robot_positions = wrapper.get_position_global() # Returns a list of robot positions and orientaions
    
    # Returns a list of list containing (rgbw) values
    # robot_light_sensor_data[0][0] returns the red value from the sensor of the first robot
    # robot_light_sensor_data[n][m < 4] returns a value (w,g,b or w ) from the sensor of robot N
    robot_light_sensor_data = warpper.get_light() 
    current_pos_xy = [x[:2] for x in current_pos] # Current (x,y) positions of the robots

    #The control function needs to return a list of velocites for the robots
    
    # Checks robot velocities for possible collisions
    vels = si_barrier_cert(np.asarray(vels).transpose(),np.asarray(current_pos_xy).transpose())
    
    vels = si_to_uni_dyn(vels,np.asarray(current_pos).transpose())
    
    wrapper.set_velocities(vels.transpose())
    wrapper.step()


if __name__ == "__main__":
    main()