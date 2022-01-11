import queue
import multiprocessing as mp
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
                return robot

    def update_position(self,global_msg):
            msg = self.get_pos(global_msg)
            self.position["x"] = -msg.pose.pose.position.x
            self.position["y"] = msg.pose.pose.position.z
            print("X: ", self.position["x"])
            self.position["orientation"] = msg.pose.pose.orientation
            self.position_pub.publish(msg)

            
    def halt_pos_pub(self):
        self.control_thread_event.set()

    def set_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.velocity_pub(twist)

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

    def move_to_point(self,x,y):

        # rospy.loginfo("X: {x} Y: {y}".format(x=current_x, y=current_y))
        while not np.sqrt((x - self.position["x"])**2 + (y - self.position["y"])**2) < .05:
            
            # Gets the difference between the current position and desired position
                delta_x = x - self.position["x"]
                delta_y = y - self.position["y"]
                theta = -self.rpy_from_quaternion(self.position["orientation"])[2]
                # Gets the time such that the robot would move to the point at v_max
                t = math.sqrt(
                    (math.pow(delta_x, 2) + math.pow(delta_y, 2)) / math.pow(self.v_max, 2))
                # Gets the velocities
                x_velo = delta_x / t
                y_velo = delta_y / t

                # Calculates the sine and cosine of the current theta
                a = np.cos(theta)
                b = np.sin(theta)

                # Finds the linear velocity
                v = 1*(x_velo*a + y_velo*b)

                # Finds the angular velocity
                omega = self.omega_max * np.arctan2(-b*x_velo + a*y_velo, v) / (np.pi/2)

                twist_msg = Twist()
                twist_msg.linear.x = v
                twist_msg.angular.z = omega
                self.twist_pub.publish(twist_msg)
                
        print("Done")
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.twist_pub.publish(twist_msg)

    def move_to_point_topic(self,msg):
        # print(msg)
        x = msg.x
        y = msg.y
        temp = mp.Process(target=self.move_to_point,args=(x,y))
        # mp.set_start_method('fork')
        temp.start()

    def __init__(self, robot_id, robot_name, positions, positions_lock):
        self.robot_id = robot_id
        self.robot_name = robot_name
        self.positions = positions
        self.positions_lock = positions_lock
        # self.velocity_pub = rospy.Publisher("/"+robot_name+"/cmd_vel",Twist,queue_size=5)
        self.position_pub = rospy.Publisher("/"+robot_name+"/position",Odometry,queue_size=3)
        self.move_to = rospy.Publisher("/"+robot_name+"/to_point",Point,queue_size=3)
        self.control_thread_event = threading.Event()
        self.control_thread = threading.Thread(
            target=self.update_position, args=(), daemon=True)
        self.control_thread.start()
