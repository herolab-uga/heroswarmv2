from curses import wrapper
from ServerWrapper import *
import time

wrapper = ServerWrapper(1) # Swarm wrapper object

while True:
    theta = wrapper.get_data("global_pos")[0][2]
    if theta < 0:
        theta = theta + 2*np.pi
    print("Angle:",theta)
    time.sleep(.25)