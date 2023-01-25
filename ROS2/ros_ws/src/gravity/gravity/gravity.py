#! /usr/bin/python3

import rclpy
import random
import threading
import numpy as np
from rclpy.node import Node
from utilities.graph import *
from std_msgs.msg import Float64
from utilities.transformations import *
from geometry_msgs.msg import Twist,Point
from gravity.srv import GoalPoint,MAdvertise
from robot_msgs.msg import RobotPos,StringList

class Gravity(Node):

    def anchor(self,goal_pos):
        self.target_postion_lock.release()
        #we want to direct the robot to the desired anchor position
        dxi = self.neighbors[self.robot_name].position.x - goal_pos.x
        dyi = self.neighbors[self.robot_name].position.y - goal_pos.y
        vels = self.si_to_uni_dyn(np.array([[dxi],[dyi]]),np.array([[self.neighbors[self.robot_name].position.x],[self.neighbors[self.robot_name].position.y],[self.neighbors[self.robot_name].position.theta]]))
        self.vel_pub.publish(Twist(vels[0], 0, vels[1]))
    
    def solver(self):
        pass

    def run(self):
        while True:
            self.target_position_lock.acquire()
            if self.robot_index == 0:
                self.anchor(Gravity.anchor_position)
            elif self.target_position is not None:
                self.anchor(self.target_position)
            else:
                self.target_postion_lock.release()
                self.solver()

    def position_callback(self, msg):
        self.neighbors = {robot.name: robot for robot in msg if robot.name in self.neighbor_names or robot.name == self.robot_name}

    def name_callback(self, msg,L):
        self.robot_index = msg.data.index(self.robot_name)
        neighbors = topological_neighbors(L,self.robot_index)
        self.neighbor_names = {msg.data[index]: None for index in neighbors}
        
    def environment_consensus_callback(self, msg):
        with self.environment_consensus_lock:
            Gravity.g = msg.data
    
    def goal_pos_callback(self, request, response):
        with self.target_position_lock:
            self.target_position = request
        response.confirm = True
        return response
    
    def m_advertise_callback(self, request, response):
        response.m = self.m
        return response

    def __init__(self) -> None:
        super().__init__("gravity")

        # Declare launch parameters
        self.declare_parameter("F")
        self.declare_parameter("g")

        self.robot_name = self.get_namespace() # this is the name of the robot
        self.m = random.randint(1, 100) # this is an intrinsic property of the robot NOTE: this needs to be set in the launch file

        self.active_robots = {}
        # we need to know the names of the robots neighbors as described with the graph laplacian matrix
        # we all need to know the names of the start robot
        self.neighbor_names = {} #dict of neighbors and their m values
        self.neighbors = {} #list of neighbor names

        Gravity.anchor_position = self.get_parameter("anchor_pos").value # this is the desired position of the start robot NOTE: i think i want to make this a service
        Gravity.g = self.get_parameter("g").value # this is a global property of the environment
        self.si_to_uni_dyn,_ = create_si_to_uni_dynamics_mapping()

        # Create a subscriber to the global position topic
        self.pos_subscription = self.create_subscription(RobotPos,"/position",self.position_callback,10)
        # Create subscriber to the active_robots topic
        self.active_subscription = self.create_subscription(StringList,"/active_robots",self.name_callback,10)
        # Create a publisher to the velocity topic
        self.vel_publisher = self.create_publisher(Twist,"/{robot_name}/cmd_vel".format(robot_name=self.robot_name),10)
        
        self.environment_consensus_lock = threading.Lock()
        self.environment_consensus_pub = self.create_publisher(Float64,"/environment_consensus",10)
        self.environment_consensus_sub = self.create_subscription(Float64,"/environment_consensus",self.environment_consensus_callback,10)

        self.target_position = None
        self.target_postion_lock = threading.Lock()
        self.goal_pos_service = self.create_service(GoalPoint,"/{robot_name}/goal_pos".format(robot_name=self.robot_name),self.goal_pos_callback)
        
        self.m_advertise_service = self.create_service(MAdvertise,"/{robot_name}/m".format(robot_name=self.robot_name),self.m_advertise_callback)


        #--------------------------------- Experiment Setup ---------------------------------#
        # Additional set up
        # Fill assign neighbor names with their m values in neighbor_names dict

        # We need to be able to setup different scenarios
        # First we calculate F based on the initial positions of the robots
        Gravity.F = self.get_parameter("F").value # this is a dict of link cost with keys being the names of the neighbors
        if Gravity.F == -1:
            # Calculate F using its neighbors
            pass

def main():
    rclpy.init()
    # Spin in a separate thread
    gravity = Gravity()
    rclpy.spin(gravity)
    gravity.destroy_node()

