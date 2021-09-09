#! /usr/bin/python3

import math
import struct
import numpy as np

import rclpy
from geometry_msgs.msg import Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_msgs.msg import Enviornment, Light
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16

class PrintOdom(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.odom_sub = self.create_subscription(Odometry, "/swarmsteve/odom",self.read_odom,5)

    def rpy_from_quaternion(self,quaternion):
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

    def read_odom(self,odom_msg):
        odom_msg.twist.twist.linear.x 
        odom_msg.twist.twist.linear.y
        
        odom_msg.twist.twist.angular.z

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        theta = self.rpy_from_quaternion(odom_msg.pose.pose.orientation)[0]

        self.get_logger().info("X: {x} Y: {y} Theta: {theta}".format(x=x,y=y,theta=theta))

def main(args=None):
    rclpy.init(args=args)
    controller = PrintOdom()
    rclpy.spin(controller)
