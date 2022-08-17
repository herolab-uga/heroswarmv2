from __future__ import division

import time
import rospy
import threading
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from robot_msgs.msg import StringList, Robot_Pos,Light
from std_msgs.msg import Int16, Int16MultiArray

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
            name = msg.data[i].data

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
                                 "proximity":None},
                "neopixel_color":None,
                "neopixel_pub":rospy.Publisher("/{robot_name}/neopixel".format(robot_name=name),Int16MultiArray,queue_size=1)
            }
            
            self.active_bots[i] = dict_entry
            self.active_bots[name] = dict_entry

    def prox_callback(self,msg,id):
        try:
            self.active_bots[id]["light_sensor"]["proximity"] = msg.data
        except KeyError:
            self.missing_bots[id] = time.time() if id not in self.missing_bots.keys() else None
            # print("Id {id} not found".format(id=id))

    def light_callback(self,msg,id):
        try:
            self.active_bots[id]["light_sensor"]["rgbw"] = msg.rgbw
        except KeyError:
            self.missing_bots[id] = time.time() if id not in self.missing_bots.keys() else None
            # print("Id {id} not found".format(id=id))
        
    
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
            self.missing_bots[id] = time.time() if id not in self.missing_bots.keys() else None
            # print("Id {id} not found".format(id=id))

    def position_callback(self,msg,active_bots):
        for id in range(0,self.num_active_bots):
            try:
                name = self.active_bots[id]["name"]
                x = msg.robot_pos[id].pose.pose.position.x
                y = msg.robot_pos[id].pose.pose.position.y
                theta = -self.rpy_from_quaternion(msg.robot_pos[id].pose.pose.orientation)[2]
            except KeyError:
                self.missing_bots[id] = time.time() if id not in self.missing_bots.keys() else None
                # print("Key {key} not found".format(key=id))
                continue
            except IndexError:
                self.missing_bots[id] = time.time() if id not in self.missing_bots.keys() else None
                # print("Index {index} out of bounds".format(index=id))
                continue
            try:
                active_bots[id]["global_pos"] = [x,y,theta]
            except KeyError:
                self.missing_bots[id] = time.time() if id not in self.missing_bots.keys() else None
                # print("Key {key} not found".format(key=name))
                continue
                
    def step(self,rate=10,time=1000):
        pub_rate = rospy.Rate(rate)
        for i in range(0,int(rate*(time/1000))):
            for robot in self.active_bots:
                # print("Stepping")
                if type(robot) == str:
                    # self.active_bots[robot]["neopixel_pub"].publish(self.active_bots[robot]["neopixel_color"])
                    if self.active_bots[robot]["vel_control"]:
                        self.active_bots[robot]["cmd_vel_pub"].publish(self.active_bots[robot]["cmd_vel"])
                    else:
                        self.active_bots[robot]["to_point_pub"].publish(self.active_bots[robot]["point"])
            pub_rate.sleep()
    
    def stop(self):
        self.set_velocities([[0.0,0.0]]*self.num_active_bots)
        self.step()
            
    def set_velocities(self,vel_list):
        for (index,vel) in enumerate(vel_list):
            self.active_bots[index]["vel_control"] = True
            msg = Twist()
            msg.linear.x = vel[0]
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = vel[1]
            self.active_bots[index]["cmd_vel"] = msg

    def set_points(self,points):
        for (index,point) in enumerate(points):
            if not point == None:
                self.active_bots[index]["vel_control"] = False
                self.active_bots[index]["to_point"] = Point(point[0],point[1],0.0)

    def set_neopixel(self,values):
        for (index,value) in enumerate(values):
            if not value == None:
                self.active_bots[index]["neopixel"] = Int16MultiArray(data=value)

    def get_odom_pos(self):
        odom_pos = []
        for active_bot in self.active_bots:
            if type(active_bot) == str:
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

    def get_light(self):
        light = []
        for active_bot in self.active_bots:
            if type(active_bot) == str:
                light.append(self.active_bots[active_bot]["light_sensor"]["rgbw"])
        return light

    def get_proximity(self):
        prox = []
        for active_bot in self.active_bots:
            if type(active_bot) == str:
                prox.append(self.active_bots[active_bot]["proximity"])
        return prox

    def get_active(self):
        return self.active_bots

    def get_num_active(self):
        return self.num_active_bots

    def remove_bots(self):
        while True:
            #print(self.missing_bots)
            for index,id in enumerate(self.missing_bots):
                try:
                    if self.missing_bots[id] < time.time() - self.timeout and self.missing_bots[id] != None:
                        #print(id)
                        name = self.active_bots[id]["name"]
                        self.active_bots.pop(id,None)
                        self.active_bots.pop(name,None)
                        self.missing_bots.pop(index,None)
                        self.num_active_bots = self.num_active_bots - 1
                        print("Bot {id} removed".format(id=id))
                except TypeError:
                    print("Read write error")
            time.sleep(1)

    def __init__(self,selected_bots=0) -> None:
        rospy.init_node("server_wrapper",anonymous=True)
        self.selected_bots = selected_bots
        self.active_bots = {}
        self.num_active_bots = 0
        self.missing_bots = {

        }

        self.timeout = 0.1

        self.missing_bots_thread = threading.Thread(target=self.remove_bots,args=(),daemon=True)
        self.missing_bots_thread.start()
    
        self.active_bots_sub = rospy.Subscriber("active_robots",StringList,self.name_callback)
        time.sleep(.5)

        self.global_position = rospy.Subscriber("positions",Robot_Pos,self.position_callback,(self.active_bots))
        time.sleep(.5)