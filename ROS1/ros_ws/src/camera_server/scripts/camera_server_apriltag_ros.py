#! /usr/bin/python3
from __future__ import division, print_function

import json
import math
import queue
import threading
from argparse import ArgumentParser
from multiprocessing import Process, Queue
from time import time

import apriltag
import cv2
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from robot_msgs.msg import Robot_Pos, StringList
from robot_msgs.srv import GetCharger, GetChargerResponse, ReleaseCharger, ReleaseChargerResponse 
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
from apriltag_ros.msg import *


class CameraServer():

    def get_positions(self,detections):  
        self.positions = Robot_Pos()
        
        if self.transform_matrix == None:
            try:
                for detection in detections.detections:
                    if detection.id[0] == self.reference_tags[1]:
                        self.ref_x = detection.pose.pose.pose.position
                    elif detection.id[0] == self.reference_tags[2]:
                        self.ref_y = detection.pose.pose.pose.position
                    elif detection.id[0] == self.reference_tags[0]:
                        self.orig = detection.pose.pose.pose.position
                        
                # Creates the rotation matrix
                self.rotation_matrix = np.array([[0, 1], [-1, 0]])
            except IndexError:
                return
            
        robot_names = StringList()
        active_dict = {}
        # print(detections)
        for detection in detections.detections:

            center = [detection.pose.pose.pose.position.x,detection.pose.pose.pose.position.y]
            
            # Gets the center of the tag in inches and rotated accordingly

            center_transform = self.transform(center)

            if detection.id[0] in self.charger_tags and not detection.id[0] in self.close_chargers:
                temp = Pose()
                
                temp.position.x = center_transform[0]
                temp.position.y = center_transform[1]
                temp.position.z = 0.0

                temp.orientation.x = detection.pose.pose.pose.orientation.x
                temp.orientation.y = detection.pose.pose.pose.orientation.y
                temp.orientation.z = detection.pose.pose.pose.orientation.z
                temp.orientation.w = detection.pose.pose.pose.orientation.w

                
                self.charger_tags[detection.id[0]] = temp

            elif not detection.id[0] in self.reference_tags:
                # Gets the forward direction
                # forward_dir = self.heading_dir(detection.pose.pose.pose.orientation, center)
                # dimg1=self.draw1(dimg1,forward_dir,center)

                self.positions.robot_pos.append(Odometry())
                robot_names.data.append(String())
                self.positions.robot_pos[-1].child_frame_id = str(detection.id[0])

                active_dict[str(detection.id[0])] = self.robot_dictionary[str(detection.id[0])]
                robot_names.data[-1].data = self.robot_dictionary[str(detection.id[0])]

                self.positions.robot_pos[-1].pose.pose.position.x = center_transform[0]
                self.positions.robot_pos[-1].pose.pose.position.y = center_transform[1] 
                self.positions.robot_pos[-1].pose.pose.position.z = 0.0

                self.positions.robot_pos[-1].pose.pose.orientation.x = detection.pose.pose.pose.orientation.x
                self.positions.robot_pos[-1].pose.pose.orientation.y = detection.pose.pose.pose.orientation.y
                self.positions.robot_pos[-1].pose.pose.orientation.z = detection.pose.pose.pose.orientation.z
                self.positions.robot_pos[-1].pose.pose.orientation.w = detection.pose.pose.pose.orientation.w
    
        self.active_dict = active_dict

        self.pos_pub.publish(self.positions)
        self.active_pub.publish(robot_names)

    # Uses the transform matrix above to transform points
    def transform(self, matrix):
        matrix = self.rotate(matrix)
        output = np.asarray(
            (
            #     self.transform_matrix[0] * (float(matrix[0])- self.orig.x), 
            #     self.transform_matrix[1] * (float(matrix[1]) - self.orig.y)

                (float(matrix[0])- self.orig.x), 
                (float(matrix[1]) - self.orig.y)
            )
            )
        # return 
        return output

    # Uses the rotation matrix above to rotate points
    def rotate(self, matrix):
        self.ref_x
        self.ref_y
        self.orig
        self.rotation_matrix
        return np.flip(np.matmul(matrix, self.rotation_matrix))

    # Get a charger
    def handle_get_charger(self,req):
        open_charger = list(self.open_chargers)[0]
        self.closed_chargers.append(open_charger)
        return GetChargerResponse(id = open_charger, position=open_charger)

    # Release the charger when done charging
    def handle_release_charger(self,req):

        self.closed_chargers.remove(req.id)
        return ReleaseChargerResponse(True)

    def __init__(self):

        rospy.init_node("camera_server",anonymous=True)

        self.ref_x = None
        self.ref_y = None
        self.orig = None
        self.rotation_matrix = None
        self.transform_matrix = None

        self.reference_tags = [0, 1, 2] # List that holds the ids of the reference tags
        self.charger_tags = [500]

        self.open_chargers = {}
        self.closed_chargers = []

        self.x_distance = 2.413
        self.y_distance = 1.74625 #67.5 #1.7145

        self.pos_pub = rospy.Publisher("/positions",Robot_Pos,queue_size=1)
        self.bridge = CvBridge()

        self.robot_dictionary = None
        with open("/home/michael/Documents/heroswarmv2/ROS1/ros_ws/src/camera_server/scripts/robots.json") as file:
            self.robot_dictionary = json.load(file)

        self.get_charger = rospy.Service("get_charger",GetCharger,self.handle_get_charger)
        self.release_charger = rospy.Service("release_charger",ReleaseCharger,self.handle_release_charger)

        self.active_pub = rospy.Publisher("active_robots",StringList,queue_size=1)

        self.positions = None
        self.active_dict = {}
        self.thread_dict = {}
        
        self.image_queue = Queue(maxsize=1)

        self.tag_detect = rospy.Subscriber("/tag_detections",AprilTagDetectionArray,self.get_positions)

if __name__ == '__main__':
    server = CameraServer()
    rospy.spin()