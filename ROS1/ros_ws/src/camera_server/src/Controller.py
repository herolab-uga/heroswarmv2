import queue
import threading
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point
from nav_msgs.msg import Odometry
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
        while not self.control_thread_event.is_set():
            with self.positions_lock:
                msg = self.get_pos()
                self.position_pub.publish(msg)
                self.position["x"] = msg.pose.pose.position.x
                self.position["y"] = msg.pose.pose.position.y
                self.position["orientation"] = msg.pose.pose.orientation

            
    def halt_pos_pub(self):
        self.control_thread_event.set()

    def move_to_point(self, x, y):
        print("Point")
        point = Point()
        point.x = x
        point.y = y
        self.move_to.publish(point)

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
        
        self.target_pos[0] = x
        self.target_pos[1] = y
        current_x = self.position["x"]
        current_y = self.position["y"]
        orientation = self.position["orientation"]
        twist_pub = self.twist_pub
        rate = rospy.Rate(25)

        while not self.target_pos[0] == None and not self.target_pos[0] == None:
            # rospy.loginfo("X: {x} Y: {y}".format(x=current_x, y=current_y))
            if np.sqrt((x - current_x)**2 + (y - current_y)**2) < .05:
                print("Done")
                self.target_pos[0] = None
                self.target_pos[1] = None
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                twist_pub.publish(twist_msg)
                self.done = True
            else:
                delta_x = x - current_x
                delta_y = y - current_y
                theta = self.rpy_from_quaternion(orientation)[2]
                v = -(delta_x*np.cos(theta) + delta_y*np.sin(theta))
                omega = .1*(2*np.arctan2(-np.sin(theta)*delta_x + np.cos(theta)*delta_y,v))/np.pi
                twist_msg = Twist()
                twist_msg.linear.x = v
                twist_msg.angular.z=omega
                twist_pub.publish(twist_msg)
            rate.sleep()

    def move_to_point_topic(self,msg):
        print(msg)
        x = msg.x
        y = msg.y
        self.move_to_point(x,y)

    def __init__(self, robot_id, robot_name, positions, positions_lock):
        self.robot_id = robot_id
        self.robot_name = robot_name
        self.positions = positions
        self.positions_lock = positions_lock
        self.position_pub = rospy.Publisher("position",Odometry,queue_size=3)
        self.move_to = rospy.Subscriber("to_point",Point,self.move_to_point_topic)
        self.twist_pub = rospy.Publisher("cmd_vel",Twist, queue_size=5)
        self.control_thread_event = threading.Event()
        self.control_thread = threading.Thread(
            target=self.update_position, args=(), daemon=True)
        self.control_thread.start()
        self.position = {"x":None,"y":None,"orientation":None}
        self.v_max = 0.1
        self.omega_max = 0.25
        self.target_pos = [None,None]
