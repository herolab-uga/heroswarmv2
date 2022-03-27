from __future__ import division

import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from robot_msgs.msg import StringList, Robot_Pos,Light
from std_msgs.msg import Int16

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

            dict_entry =  {
                "name":name,
                "vel_control":None,
                "global_pos":[0,0,0],
                "vel":[0,0,0],
                "odom_pos":[0,0,0],
                "odom_sub":rospy.Subscriber("/{robot_name}/odom".format(robot_name=str(name)),Odometry,self.odom_callback,(i)),
                "cmd_vel":[0,0,0],
                "cmd_vel_pub": rospy.Publisher("/{robot_name}/cmd_vel".format(robot_name=name),Twist,queue_size=1),
                "point":[0,0],
                "to_point_pub": rospy.Publisher("/{robot_name}/to_point".format(robot_name=name),Point,queue_size=1),
                "light_sub":rospy.Subscriber("/{robot_name}/light".format(robot_name=name),Light,self.light_callback,(i)),
                "prox_sub":rospy.Subscriber("/{robot_name}/proximity".format(robot_name=name),Int16,self.prox_callback,(i)),
                "light_sensor": {"rgbw":None,
                                 "proximity":None}
            }
            
            self.active_bots[i] = dict_entry
            self.active_bots[name] = dict_entry

    def prox_callback(self,msg,id):
        try:
            self.active_bots[id]["light_sensor"]["proximity"] = msg.data
        except KeyError:
            print("Id {id} not found".format(id=id))

    def light_callback(self,msg,id):
        try:
            self.active_bots[id]["light_sensor"]["rgbw"] = msg.light.rgbw
        except KeyError:
            print("Id {id} not found".format(id=id))
        
    
    def odom_callback(self,msg,id):
        x_vel = msg.twist.twist.linear.x
        y_vel = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z

        x_pos = msg.pose.pose.position.x
        y_pos = msg.pose.pose.position.y
        theta = -self.rpy_from_quaternion(msg.pose.pose.orientation)[2]
        
        try:
            self.active_bots[id]["vel"] = [x_vel,y_vel,omega]
            self.active_bots[id]["odom_pos"] = [x_pos,y_pos,theta]
        except KeyError:
            print("Id {id} not found".format(id=id))

    def position_callback(self,msg,active_bots):
        for i in range(0,self.num_active_bots):
            try:
                name = self.active_bots[i]["name"]
                x = msg.robot_pos[i].pose.pose.position.x
                y = msg.robot_pos[i].pose.pose.position.y
                theta = -self.rpy_from_quaternion(msg.robot_pos[i].pose.pose.orientation)[2]
            except KeyError:
                print("Key {key} not found".format(key=i))
                continue
            except IndexError:
                print("Index {index} out of bounds".format(index=i))
                continue
            try:
                active_bots[name]["global_pos"] = [x,y,theta]
            except KeyError:
                print("Key {key} not found".format(key=name))
                continue
                
    def step(self,rate=10,time=100):
        pub_rate = rospy.Rate(rate)
        for i in range(0,int(rate*(time/1000))):
            for robot in self.active_bots:
                if type(robot) == str:
                    if self.active_bots[robot]["vel_control"]:
                        self.active_bots[robot]["cmd_vel_pub"].publish(self.active_bots[robot]["cmd_vel"])
                    else:
                        self.active_bots[robot]["to_point_pub"].publish(self.active_bots[robot]["point"])
            pub_rate.sleep()
    
    def stop(self):
        self.set_velocities([[0.0,0.0,]]*self.num_active_bots)
        self.step()
            
    def set_velocities(self,vel_list):
        for (index,vel) in enumerate(vel_list):
            self.active_bots[index]["vel_control"] = True
            msg = Twist()
            msg.linear.x = vel[0]
            msg.angular.z = vel[1]
            self.active_bots[index]["cmd_vel"] = msg

    def set_points(self,points):
        for (index,point) in enumerate(points):
            if not point == None:
                self.active_bots[index]["vel_control"] = False
                self.active_bots[index]["to_point"] = Point(point[0],point[1],0.0)

    def get_odom_pos(self):
        odom_pos = []
        for active_bot in self.active_bots:
            odom_pos.append(self.active_bots[active_bot]["odom_pos"])
        return odom_pos
    
    def get_position_global(self):
        positions = []
        for active_bot in self.active_bots:
            if type(active_bot) == str:
                positions.append(self.active_bots[active_bot]["global_pos"])
        return positions

    def get_velocity(self):
        velocities = []
        for active_bot in self.active_bots:
            if type(active_bot) == str:
                velocities.append(self.active_bots[active_bot]["vel"])
        return velocities

    def get_proximity(self):
        prox = []
        for active_bot in self.active_bots:
            if type(active_bot) == str:
                prox.append(self.active_bots[active_bot]["proximity "])
        return prox

    def __init__(self,selected_bots=0) -> None:
        rospy.init_node("server_wrapper",anonymous=True)
        self.selected_bots = selected_bots
        self.active_bots = {}
        self.velocity_subs = []
        self.obom_subs = []
        self.num_active_bots = 0
        self.active_bots_sub = rospy.Subscriber("active_robots",StringList,self.name_callback)
        time.sleep(.5)
        self.global_position = rospy.Subscriber("positions",Robot_Pos,self.position_callback,(self.active_bots))
        time.sleep(.5)