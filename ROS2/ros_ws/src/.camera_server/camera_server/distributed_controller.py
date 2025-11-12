import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_msgs.msg import Robot_Pos


class DistributedController():

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

    def read_position(self,msg):
        for robot in msg.robot_pos:
            if robot.child_frame_id == str(self.id):
                self.position = robot
                self.position_pub.publish(robot)

    def odom_callback(self,msg):
        x_pos = msg.pose.pose.position.x
        y_pos = msg.pose.pose.position.y
        theta = self.rpy_from_quaternion(msg.pose.pose.orientation)[2]

        x_velo = msg.twist.twist.linear.x
        y_velo = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z

        self.odom_pos = [x_pos,y_pos,theta]
        self.odom_velo = [x_velo,y_velo,omega]

    def get_ground_pos(self):
        x = self.position.pose.pose.position.x
        y = self.position.pose.pose.position.y
        theta = -self.rpy_from_quaternion(self.position.pose.pose.orientation)[2]
        return [x,y,theta]

    def get_odom_pos(self):
        return self.odom_pos

    def get_velocity(self):
        return self.odom_velo

    def set_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.velocity_pub(twist)

    def __init__(self, robot_id, robot_name):
        self.id = robot_id
        self.name = robot_name
        self.position = None
        self.odom_pos = None
        self.odom_velo = None
        self.velocity_pub = rospy.Publisher("/{robot_name}/cmd_vel".format(robot_name=self.name),Twist,queue_size=1)
        self.position_pub = rospy.Publisher("/{robot_name}/position".format(robot_name=self.name),Odometry,queue_size=1)
        self.pos_list_sub = rospy.Subscriber("/positions",Robot_Pos,self.read_position)
        self.odom_sub = rospy.Subscriber("/{robot_name}/odom".format(robot_name=self.name),Odometry,self.odom_callback)
