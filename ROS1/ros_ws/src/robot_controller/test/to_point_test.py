from ServerWrapper import *
import time

wrapper = ServerWrapper(4)

p1 = [1.0,0.75,0.0]
p2 = [0.5,1.0,0.0]
p3 = [0.36,0.97,0.0]
p4 = [0.1,1.25,0.0]

pos = [p1,p2,p3,p4]

wrapper.set_points([p1,p2,p3,p4])

for i in range(0,30):
    wrapper.step()
    time.sleep(1)

wrapper.set_velocities([[0.0,0.0,]]*4)
wrapper.step()