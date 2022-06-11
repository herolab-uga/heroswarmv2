#! /usr/bin/python3

import math
import struct
import threading
import time

import numpy as np
import rospy
import random

from geometry_msgs.msg import Quaternion, Twist, Vector3, Point
from nav_msgs.msg import Odometry
from robot_msgs.msg import Environment, Light, Robot_Pos
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16MultiArray




class test_node():
    def __init__(self) -> None:
         rospy.init_node("test_script", anonymous=True)
         self.test_vel = rospy.Publisher("/swarmbob1/neopixel",Int16MultiArray,queue_size = 1)
    
    def twsit_pub(self,color=[255,0,0]):
        msg = Int16MultiArray()
        msg.data = color
        self.test_vel.publish(msg)
        print(msg)

if __name__ == '__main__':
    test = test_node()
    rate = rospy.Rate(.5)
    while not rospy.is_shutdown():
        color = []
        for i in range(3):
            color.append(random.randint(0,255))
        test.twsit_pub(color)
        rate.sleep()
        continue