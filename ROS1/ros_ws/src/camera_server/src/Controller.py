from multiprocessing import managers
import queue
import multiprocessing as mp
from multiprocessing import Manager 
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16
import numpy as np
import rospy
from robot_msgs.msg import Robot_Pos
import math
import threading


class Controller():

    def get_pos(self,msg):
        for robot in msg.robot_pos:
            if robot.child_frame_id == str(self.robot_id):
                self.position_pub.publish(robot)

    def halt_pos_pub(self):
        self.control_thread_event.set()

    def set_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.velocity_pub(twist)

    def __init__(self, robot_id, robot_name):
        self.robot_id = robot_id
        self.robot_name = robot_name
        self.velocity_pub = rospy.Publisher("/"+robot_name+"/cmd_vel",Twist,queue_size=1)
        self.position_pub = rospy.Publisher("/"+robot_name+"/position",Odometry,queue_size=1)
        self.pos_list_sub = rospy.Subscriber("/positions",Robot_Pos,self.get_pos)
