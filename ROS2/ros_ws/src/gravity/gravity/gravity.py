#! /usr/bin/python3

import time
import rclpy
import random
import threading
import numpy as np
from rclpy.node import Node
from utilities.graph import *
from std_msgs.msg import Float64,UInt8MultiArray
from utilities.transformations import *
from geometry_msgs.msg import Twist,Point
from gravity.srv import Graph,GoalPoint,VariableAdvertise
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
        dxi = [0,0]
        for index,robot in enumerate(self.neighbors):
            distance = np.asarray([self.neighbors[robot]["position"].position.x - self.neighbors[self.robot_name]["position"].position.x,
                                   self.neighbors[robot]["position"].position.y - self.neighbors[self.robot_name]["position"].position.y])
            expected_r = (Gravity.g * self.neighbors[robot]["m"] * self.m) / self.neighbors[robot]["force"]
            dxi += 1 * (np.power(np.linalg.norm(distance),2) - expected_r) * distance
            dxi =  self.si_to_uni_dyn(dxi)


        # set dxi to go to goal pos if needed
        msg = Twist()
        msg.linear.x = dxi[0]
        msg.linear.y = 0
        msg.angular.z = dxi[1]
        self.vel_publisher(msg)

    # rate group instead of thread?
    def run(self):
        # self.target_position_lock.acquire()
        # self.target_postion_lock.release()
        # get neighbor m values
        # get neighbor f values
        # run g consensus
        self.solver()

    def position_callback(self, msg):
        self.neighbors = {robot.name: {"position":robot} for robot in msg if robot.name in self.neighbor_names or robot.name == self.robot_name}

    def name_callback(self, msg):
        # if you want to catch a mismatch in the number of available robots and the number of robots in the graph laplacian matrix
        # do it in this method
        self.robot_index = msg.data.index(self.robot_name)
        self.active_robots = msg.data
    
    def goal_pos_callback(self, request, response):
        with self.target_position_lock:
            self.target_position = request
        response.confirm = True
        return response
    
    def m_advertise_callback(self, request, response):
        response.value = self.m
        return response
    
    def g_advertise_callback(self,request, response):
        response.value = Gravity.g
        return response
    
    def f_advertise_callback(self,request, response):
        response.value = self.neighbors[request.neighbor_name]["f"]
        return response
    
    def parse_laplacian(self):
        neighbor_index = topological_neighbors(Gravity.L,self.robot_index)

        # This is a dict comprehension
        self.neighbor = {
            self.active_robots[neighbor]:{
                "position":None,
                "force":None,
                "force_client":None,
                "m":None,
                "m_client":None,
                "g_client":None
            }

            for neighbor in neighbor_index
        }

    def graph_laplacian_callback(self,msg):
        Gravity.L = np.asarray(msg.data)
        self.parse_laplacian()

    def calculate_F(self):
        for index,robot in enumerate(self.neighbors):
            distance = np.asarray([self.neighbors[robot]["position"].position.x - self.neighbors[self.robot_name]["position"].position.x,
                                   self.neighbors[robot]["position"].position.y - self.neighbors[self.robot_name]["position"].position.y])
            self.neighbors[robot]["force"] = (Gravity.g * self.m * self.neighbors[robot]["m"])/np.power(distance,2)

    def __init__(self) -> None:
        super().__init__("gravity")

        # Declare launch parameters
        self.declare_parameter("F")
        self.declare_parameter("g")

        self.robot_name = self.get_namespace() # this is the name of the robot
        self.robot_index = None
        self.m = random.randint(1, 100) # this is an intrinsic property of the robot NOTE: this needs to be set in the launch file

        self.active_robots = []
        self.neighbors = {} # Dict of neighborn

        # NOTE: i think i want to make this a service
        Gravity.g = self.get_parameter("g").value # this is a global property of the environment
        Gravity.L = None
        self.si_to_uni_dyn,_ = create_si_to_uni_dynamics_mapping() #- replace this with a function that only takes one pos and one dxi

        # Create a subscriber to the global position topic
        self.pos_subscription = self.create_subscription(RobotPos,"/position",self.position_callback,10)
        # Create subscriber to the active_robots topic
        self.active_subscription = self.create_subscription(StringList,"/active_robots",self.name_callback,10)
        # Create a publisher to the velocity topic
        self.vel_publisher = self.create_publisher(Twist,"/{robot_name}/cmd_vel".format(robot_name=self.robot_name),10)

        self.target_position = None
        self.target_postion_lock = threading.Lock()
        self.goal_pos_service = self.create_service(GoalPoint,"/{robot_name}/goal_pos".format(robot_name=self.robot_name),self.goal_pos_callback)
        
        self.graph_laplacian_sub = self.create_subscription(UInt8MultiArray,"/graph_laplacian",self.graph_laplacian_callback)
        # self.graph_service = self.create_service(Graph, "/{robot_name}/graph_laplacian".format(robot_name=self.robot_name),self.graph_laplacian_callback())
        self.m_advertise_service = self.create_service(VariableAdvertise,"/{robot_name}/m".format(robot_name=self.robot_name),self.m_advertise_callback)
        self.g_advertise_service = self.create_service(VariableAdvertise,"/{robot_name}/g".format(robot_name=self.robot_name),self.g_advertise_callback)
        self.f_advertise_service = self.create_service(VariableAdvertise,"/{robot_name}/f".format(robot_name=self.robot_name),self.g_advertise_callback)

        #--------------------------------- Experiment Setup ---------------------------------#
        # Additional set up
        # Fill assign neighbor names with their m values in neighbor_names dict you can get using their service
        while Gravity.L is None:
            time.sleep(.1)
            continue
        
        # This will set up all the clients that the robot needs to communicate with its neighbors and get the initial values
        for neighbor in self.neighbors:
            
            # Sets up the client
            self.neighbors[neighbor]["f_client"] = self.create_client(VariableAdvertise,"/{robot_name}/m".format(robot_name=neighbor))
            
            # Waits for the client to become available
            while not self.neighbors[neighbor]["f_client"].wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')

            self.neighbors[neighbor]["m_client"] = self.create_client(VariableAdvertise,"/{robot_name}/g".format(robot_name=neighbor))

            while not self.neighbors[neighbor]["m_client"].wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')

            m_msg = VariableAdvertise.Request()
            m_msg.robot_name = self.robot_name
            m_msg.neighbor_name = neighbor

            m_future = self.neighbors[neighbor]["m_client"].call_async(m_msg)
            rclpy.spin_until_future_complete(self,m_future)

            self.neighbors[neighbor]["g_client"] = self.create_client(VariableAdvertise,"/{robot_name}/f".format(robot_name=neighbor))

            while not self.neighbors[neighbor]["g_client"].wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')

        # First we calculate F based on the initial positions of the robots
        self.calculate_F()

        self.run_rate = self.create_rate(120)

        while True:
            self.run()
            self.run_rate.sleep()

        # use a rate for the run method 

def main():
    rclpy.init()
    # Spin in a separate thread
    gravity = Gravity()
    thread = threading.Thread(target=rclpy.spin, args=(gravity, ), daemon=True)
    thread.start()
    gravity.destroy_node()

