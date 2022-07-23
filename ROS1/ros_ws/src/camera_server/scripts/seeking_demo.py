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

    # Create SI to UNI dynamics transformation
    si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()

    num_robots = 3 # Number of robots to use

    wrapper = ServerWrapper(num_robots) # Swarm wrapper object

    robot_positions = wrapper.get_position_global() # Returns a list of robot positions and orientations
    
    # Returns a list of list containing (rgbw) values
    # robot_light_sensor_data[0][0] returns the red value from the sensor of the first robot
    # robot_light_sensor_data[n][m < 4] returns a value (w,g,b or w ) from the sensor of robot N
    robot_light_sensor_data = wrapper.get_light() 
    current_pos_xy = [x[:2] for x in robot_positions] # Current (x,y) positions of the robots

    # Get index of max value in list+
    light_intensity = [x[3] for x in robot_light_sensor_data]
    light_intensity_max_index = light_intensity.index(max(light_intensity))

    # Get the position of the light source
    light_intensity_pos = current_pos_xy[light_intensity_max_index]

    # Generate velocities to move towards the light source
    vels = []
    for (index,robot) in enumerate(robot_positions):
        if not index == light_intensity_max_index: 
            vels.append([light_intensity_pos[0] - robot[0], light_intensity_pos[1] - robot[1]])
        else:
            vels.append([robot[0]+1,robot[1]])
    
    # Checks robot velocities for possible collisions
    vels = si_barrier_cert(np.asarray(vels).transpose(),np.asarray(current_pos_xy).transpose())
    
    vels = si_to_uni_dyn(vels,np.asarray(robot_positions).transpose())
    
    wrapper.set_velocities(vels.transpose())
    wrapper.step()


if __name__ == "__main__":
    main()