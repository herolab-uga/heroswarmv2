#! /usr/bin/python3

import math
import struct

import geometry_msgs
import rospy
import numpy as np
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import quaternion


class PrintOdom():
    def __init__(self) -> None:
        rospy.init_node("Read Odom", anonymous=True)
        self.odom_node = rospy.Subscriber(
            "/swarmbilly1/position", Odometry, self.read_pos)
        self.positions = {}

    def rpy_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def read_pos(self, msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.z
            orientation = msg.pose.pose.orientation
            theta = self.rpy_from_quaternion(orientation)[2]
            # print("X: {x} \n Y: {y} \n Theta: {theta}".format(x=x,y=y,theta=theta))
            print("Theta: ", theta)
       
if __name__ == '__main__':
    controller = PrintOdom()
    while not rospy.is_shutdown():
        continue
        