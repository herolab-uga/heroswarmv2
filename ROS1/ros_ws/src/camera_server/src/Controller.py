import threading
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point
from nav_msgs.msg import Odometry
from robot_msgs.msg import Environment, Light, Robot_Pos
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16
import numpy as np
import rospy


class Controller():

    def get_pos(self):
        for robot in self.positions.robot_pos:
            if robot.child_frame_id == str(self.robot_id):
                return robot

    def update_position(self):
        while not self.stop_event.is_set():
            with self.positions_lock:
                msg = self.get_pos()
                self.position_pub.publish(msg)

    def halt_pos_pub(self):
        self.control_thread_event.set()

    def move_to_point(self, x, y):
        point = Point()
        point.x = x
        point.y = y
        self.move_to.publish(point)

    def set_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.velocity_pub(twist)

    def __init__(self, robot_id, robot_name, positions, positions_lock):
        self.robot_id = robot_id
        self.robot_name = robot_name
        self.positions = positions
        self.positions_lock = positions_lock
        self.velocity_pub = rospy.Publisher("/"+robot_name+"/cmd_vel")
        self.position_pub = rospy.Publisher("/"+robot_name+"/position")
        self.move_to = rospy.Publisher("/"+robot_name+"/move_to_point")
        self.control_thread_event = threading.Event()
        self.control_thread = threading.Thread(
            self.update_position, args=(), daemon=True)
        self.control_thread.start()
