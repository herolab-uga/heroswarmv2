from ServerWrapper import *
import numpy as np
import csv
import getch
import threading
import queue

wrapper = ServerWrapper(1) # Swarm wrapper object

theta_queue = queue.Queue(1)

def go_back(theta):
    while abs(5.495-theta) > .1:
        x,y,theta = wrapper.get_data("global_pos")[0]
        if theta < 0:
            theta = theta + 2*np.pi
        omega = 2.1 * (5.495 - theta)/np.pi
        wrapper.set_velocities([[0,omega]])
        wrapper.step(rate=10, time=500)
    wrapper.stop()

def get_theta():
    while True:
        theta = wrapper.get_data("global_pos")[0][2]
        if theta < 0:
            theta = theta + 2*np.pi
        theta_queue.put(theta)

angle_thread = threading.Thread(target=get_theta, args=(), daemon=True)
angle_thread.start()

test_angles = [-3*np.pi/2,-7*np.pi/4]
for angle in test_angles:
    with open("angle_data_"+str(angle)+".csv","w") as file:
        writer = csv.writer(file)
        writer.writerow(["Set Arch Length","Odom Angle","Error Angle"])
        for trial in range(10):
            print("Trial:",trial)
            odom_init = wrapper.get_data("global_pos")[0][2]
            theta = 5.495
            final = 5.495 + angle
            print("Angle:",angle)
            print("Init: ",theta)
            print("Final: ",final)
            while abs(final-theta) > .1:
                theta = theta_queue.get()
                omega = 2.1 * (final - theta)/(2*np.pi)
                wrapper.set_velocities([[0,omega]])
                wrapper.step(rate=10, time=500)
            wrapper.stop()
            print("Done Trial")
            odom_angle = wrapper.get_data("odom_pos")[0][2] - odom_init
            # print("x: {x}, y: {y}, theta: {theta}".format(x=x,y=y,theta=theta))
            time.sleep(1)
            writer.writerow([angle,odom_angle,angle-odom_angle])
            go_back(theta)
