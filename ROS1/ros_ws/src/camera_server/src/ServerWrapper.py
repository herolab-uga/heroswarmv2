from __future__ import division

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from robot_msgs.msg import StringList, Robot_Pos

class ServerWrapper():
    
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

    def name_callback(self,msg):
        for i in range(self.num_active_bots,self.selected_bots):
            self.num_active_bots += 1
            name = msg.names[i].data
            
            self.active_bots[i] = {
                "name":name,
                "vel_control":None,
                "global_pos":[0,0,0],
                "vel":[0,0,0],
                "odom_pos":[0,0,0],
                "odom_sub":rospy.Subscriber("/{robot_name}/odom".format(robot_name=str(name)),Twist,self.odom_callback,(i)),
                "cmd_vel":[0,0,0],
                "cmd_vel_pub": rospy.Publisher("/{robot_name}/cmd_vel".format(robot_name=name),Twist,queue_size=1),
                "point":[0,0],
                "to_point_pub": rospy.Publisher("/{robot_name}/to_point".format(robot_name=name),Point,queue_size=1)
            }
    
    def odom_callback(self,msg,id):
        x_vel = msg.twist.twist.linear.x
        y_vel = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z

        x_pos = msg.pose.pose.position.x
        y_pos = msg.pose.pose.position.y
        theta = -self.rpy_from_quaternion(msg.position.pose.pose.orientation)[2]

        self.active_bots[id]["vel"] = [x_vel,y_vel,omega]
        self.active_bots[id]["odom_pos"] = [x_pos,y_pos,theta]

    def position_callback(self,msg,active_bots):
        for i in range(0,self.num_active_bots):
            x = msg.robot_pos[i].pose.pose.position.x
            y = msg.robot_pos[i].pose.pose.position.y
            theta = -self.rpy_from_quaternion(msg.robot_pos[i].pose.pose.orientation)[2]
            active_bots[int(msg.robot_pos[i].child_frame_id)-3]["global_pos"] = [x,y,theta]

    def step(self,rate=10,time=1000):
        pub_rate = rospy.Rate(rate)
        for i in range(0,int(rate*(time/1000))):
            for robot in self.active_bots:
                if self.active_bots[robot]["vel_control"]:
                    self.active_bots[robot]["cmd_vel_pub"].publish(self.active_bots[robot]["cmd_vel"])
                else:
                   self.active_bots[robot]["to_point_pub"].publish(self.active_bots[robot]["to_point"])
            pub_rate.sleep()
            
    def set_velocities(self,vel_list):
        for (index,vel) in enumerate(vel_list):
            if not vel == None:
                self.active_bots[index]["vel_control"] = True
                self.active_bots[index]["cmd_vel"] = Twist(vel[0],vel[1],vel[2])

    def set_points(self,points):
        for (index,point) in enumerate(points):

            if not point == None:
                self.active_bots[index]["vel_control"] = False
                self.active_bots[index]["to_point"] = Point(point[0],point[1],point[2])

    def get_odom_pos(self):
        odom_pos = []
        for active_bot in self.active_bots:
            odom_pos.append(self.active_bots[active_bot]["odom_pos"])
        return odom_pos
    
    def get_position_global(self):
        positions = []
        for active_bot in self.active_bots:
            positions.append(self.active_bots[active_bot]["global_pos"])
        return positions

    def get_velocity(self):
        velocities = []
        for active_bot in self.active_bots:
            velocities.append(self.active_bots[active_bot]["vel"])
        return velocities

    def __init__(self,selected_bots=0) -> None:
        rospy.init_node("server_wrapper",anonymous=True)
        self.selected_bots = selected_bots
        self.active_bots = {}
        self.velocity_subs = []
        self.obom_subs = []
        self.num_active_bots = 0
        self.active_bots_sub = rospy.Subscriber("active_robots",StringList,self.name_callback)
        self.global_position = rospy.Subscriber("positions",Robot_Pos,self.position_callback,(self.active_bots))