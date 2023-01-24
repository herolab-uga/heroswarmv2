#! /usr/bin/python3

import rclpy
import random
from utilities.graph import *
from utilities.
from rclpy.node import Node
from robot_msgs.msg import RobotPos



class Gravity(Node):

    def solver(self):
        # start with checking if this robot is the start 
        if self.robot_name == Gravity.anchor_robot:
            #we want to direct the robot to the desired anchor position
            dxi = self.neighbors[self.robot_name].position.x - Gravity.anchor_position.x

        else:

    def position_callback(self, msg):
        self.neighbors = {robot.name: robot for robot in msg if robot.name in self.neighbor_names or robot.name == self.robot_name}


    def __init__(self) -> None:
        super().__init__("gravity")

        # Declare launch parameters
        self.declare_parameter("F")
        self.declare_parameter("g")
        self.declare_parameter("anchor_pos")

        self.robot_name = self.get_namespace() # this is the name of the robot
        self.m = random.randint(1, 100) # this is an intrinsic property of the robot

        # we need to know the names of the robots neighbors as described with the graph laplacian matrix
        # we all need to know the names of the start robot
        self.neighbor_names = [] #list of neighbor names
        self.neighbors = {} #list of neighbor names
        Gravity.anchor_robot = "" # this is the name of the start robot
        Gravity.anchor_position = self.get_parameter("anchor_pos").value # this is the desired position of the start robot
        Gravity.F = self.get_parameter("F").value # this is required link cost
        Gravity.g = self.get_parameter("g").value # this is a global property of the environment

        # Create a subscriber to the global position topic
        self.subscription = self.create_subscription(RobotPos,"/position",self.position_callback,10)

