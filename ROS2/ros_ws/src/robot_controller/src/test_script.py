#! /usr/bin/python3

import math
import struct
import threading
import time

import numpy as np
import rospy

from geometry_msgs.msg import Quaternion, Twist, Vector3, Point
from nav_msgs.msg import Odometry
from robot_msgs.msg import Environment, Light, Robot_Pos
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16




class test_node():
    def __init__(self) -> None:
         rospy.init_node("test_script", anonymous=True)
         self.test_vel = rospy.Publisher("/swarmsteve1/cmd_vel",Twist,queue_size = 2)
    
    def twsit_pub(self,velo):
        msg = Twist()
        msg.angular.z = velo
        self.test_vel.publish(msg)
        print("Running")

    def twist_stop(self):
        msg = Twist()
        msg.angular.z = 0.0
        self.test_vel.publish(msg)
        print("Done")

if __name__ == '__main__':
    test = test_node()
    rate_time = 25
    run_time = 20
    rate = rospy.Rate(rate_time)
    start = time.time()
    for i in range(0,run_time*rate_time):
        test.twsit_pub(0.09)
        rate.sleep()
    test.twist_stop()