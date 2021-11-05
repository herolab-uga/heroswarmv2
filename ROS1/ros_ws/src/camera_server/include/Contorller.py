import threading
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point
from nav_msgs.msg import Odometry
from robot_msgs.msg import Environment, Light, Robot_Pos
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16
import numpy as np
import rospy


class Controller():
    def __init__(self, robot_id, positions):
        self.robot_id = robot_id
        self.positions = positions
