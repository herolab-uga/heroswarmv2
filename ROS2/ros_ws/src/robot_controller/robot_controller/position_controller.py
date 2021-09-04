#! /usr/bin/python3

import math
import struct

import geometry_msgs
import rclpy
import numpy as np
from geometry_msgs.msg import Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


class PositionController(Node):
    def __init__(self):
        super().__init__("Position Controller")
        self.odom_sub = self.create_subscription(Odometry,"odom",self.read_pos)
        self.twist_pub = self.create_publisher(Twist, "cmd_vel",5)
        self.position = {}
        self.v_max = .25
        self.omega_max = .5
        self.target_pos = [None,None]

    def read_pos(self,msg):
        self.position["x"] = msg.pose.pose.position.x
        self.position["y"] = msg.pose.pose.position.y
        self.position["orientation"] = msg.pose.pose.orientation

        if not self.target_pos[0] == None and not self.target_pos[0] == None:
            if np.abs(self.position["x"] - self.target_pos[0]) and np.abs(self.position["y"]-self.target_pos[1]):
                self.target_pos[0] = None
                self.target_pos[1] = None
                twist_msg = Twist()
                twist_msg.linear.x = 0
                twist_msg.angular.z = 0
                self.twist_pub.publish(twist_msg)

    
    def rpy_from_quaternion(quaternion):
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

    def move_to_point(self,x,y):
        self.target_pos[0] = x
        self.target_pos[1] = y
        delta_x = x - self.position["x"]
        delta_y = y - self.position["y"]
        theta = self.rpy_from_quaternion(self.position["orientation"][2])
        v = self.v_max (delta_x*np.cos(theta) + delta_y*np.sin(theta))
        omega = (2*self.omega_max)/math.pi(np.arctan2(-delta_x*np.sin(theta) + delta_y*np.cos(theta), v))
        twist_msg = Twist()
        twist_msg.linear.x = v
        twist_msg.angular.z=omega
        self.twist_pub.publish(twist_msg) 