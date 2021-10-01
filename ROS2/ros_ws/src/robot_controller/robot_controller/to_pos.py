import rclpy
from geometry_msgs.msg import Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
import math
import numpy as np





class to_pos(Node):

    def __init__(self):
        super().__init__("to_pos_controller")
        self.twist = self.create_publisher(Twist,'cmd_vel',5)
        self.robot = {}


    def quaternion_from_rpy(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy
        return q

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

    def move_to(self,x,y):
        dif_x = x - self.robot["x"]
        dif_y = y - self.robot["y"]
        dist = math.sqrt((dif_x**2)+(dif_y**2))
        th = math.arctan2(dif_y,dif_x)
        cmd_vel = Twist()
        cmd_vel.linear.x    =  .5 * (np.cos(th) * x - np.sin(th) * y)
        cmd_vel.linear.y    =  .5 * (np.sin(th) * x + np.cos(th) * y)
        cmd_vel.angular.z   =  1 * y
    
    
    def callback(self, msg):
        self.robot["x"] = msg.pose.pose.position.x
        self.robot["y"] = msg.pose.pose.position.y
        self.robot["pos"] = msg.pose.pose.orientation

        