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


class PositionController():
    def __init__(self):
        rospy.init_node("Posistion_Controller", anonymous=True)

        self.odom_node = rospy.Subscriber(
            "/swarmbilly1/position", Odometry, self.read_pos)
        self.twist_pub = rospy.Publisher(
            "/swarmbilly1/cmd_vel", Twist, queue_size=0)
        self.point_sub = rospy.Subscriber(
            "to_point", Point, self.move_to_point_topic)
        self.position = {}
        self.v_max = 0.1
        self.omega_max = 1.0
        self.target_pos = [None, None]

    def read_pos(self, msg):
        self.position["x"] = msg.pose.pose.position.x
        self.position["y"] = msg.pose.pose.position.z
        self.position["orientation"] = msg.pose.pose.orientation

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

    def move_to_point(self, x, y):

        self.target_pos[0] = x
        self.target_pos[1] = y
        current_x = self.position["x"]
        current_y = self.position["y"]
        orientation = self.position["orientation"]
        twist_pub = self.twist_pub
        # rate = rospy.Rate(25)

        if not self.target_pos[0] == None and not self.target_pos[0] == None:
            # rospy.loginfo("X: {x} Y: {y}".format(x=current_x, y=current_y))
            print("Error: {error}".format(error=math.sqrt((x - current_x)**2 + (y - current_y)**2)))
            if math.sqrt(math.pow((x - current_x),2) + math.pow((y - current_y),2)) < .1:
                print("Done")
                self.target_pos[0] = None
                self.target_pos[1] = None
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                twist_pub.publish(twist_msg)
                self.done = True
            else:
                # print("X: {x}".format(x=current_x))
                # print("Y: {y}".format(y=current_y))
                delta_x = x - current_x
                delta_y = y - current_y
                time = math.sqrt((math.pow(delta_x,2) + math.pow(delta_y,2)) / math.pow(self.v_max,2))
                x_velo = delta_x / time
                y_velo = delta_y / time
                quat = np.quaternion(orientation.x,orientation.y,orientation.z,orientation.w)
                theta = quaternion.as_rotation_vector(quat)[2]

                a = np.cos(theta)
                b = np.sin(theta)
                # print(self.rpy_from_quaternion(orientation))
                
                # print("Theta:theta)
                v = 1*(x_velo*a + y_velo*b)
                omega = self.omega_max * np.arctan2(-b*x_velo + a*y_velo,v) / (np.pi/2)
                twist_msg = Twist()
                twist_msg.linear.x = v
                twist_msg.angular.z = omega
                twist_pub.publish(twist_msg)
            # rate.sleep()

    def move_to_point_topic(self, msg):
        # print(msg)
        x = msg.x
        y = msg.y
        self.move_to_point(x, y)


if __name__ == '__main__':
    controller = PositionController()
    while not rospy.is_shutdown():
        continue
