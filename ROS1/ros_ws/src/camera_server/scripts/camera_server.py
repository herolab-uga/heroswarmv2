#! /usr/bin/python3
from __future__ import division, print_function

import json
import math
import queue
import threading
from time import time

import cv2
import numpy as np
import distributed_controller as distributed_controller 
import rospy
from robot_msgs.msg import StringList
from robot_msgs.srv import GetCharger, GetChargerResponse, ReleaseCharger, ReleaseChargerResponse 
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray,AprilTagDetection
import time


class CameraServer():
                
    def connection_manager(self):
        prev_active = []
        count = 0
        while True:
            active_dict = list(self.active_dict)
            add = [add_bot for add_bot in active_dict if add_bot not in prev_active]
            for new_robot in add:
                self.thread_dict[new_robot] = threading.Thread(target=distributed_controller.DistributedController, args=(new_robot, 
                                                            self.robot_dictionary[new_robot]),daemon=True)
                self.thread_dict[new_robot].start()
                count = count + 1
            prev_active = active_dict

    def get_positions(self,detections):
        if len(self.transform_matrix) == 0:
            # try:
            for detection in detections.detections:
                if detection.id[0] == 0:
                    self.orig = detection.pose.pose.pose.position
                elif detection.id[0] == 1:
                    self.ref_x = detection.pose.pose.pose.position
                elif detection.id[0] == 2:
                    self.ref_y = detection.pose.pose.pose.position
                else:
                    continue

            self.transform_matrix = [
                                        [self.x_distance/(self.ref_x.x - self.orig.x),0],
                                        [0,self.y_distance/(self.ref_y.y - self.orig.y)]
                                    ]

            # Creates the rotation matrix
        # except Exception as e:
        #         print(e)
        #         return
            
        robot_names = StringList()
        active_dict = {}
        self.positions = AprilTagDetectionArray()
        for detection in detections.detections:

            position = self.transform([detection.pose.pose.pose.position.x,detection.pose.pose.pose.position.y])
            # Gets the center of the tag in inches and rotated accordingly
            # print(len(position))?
            if detection.id[0] in self.charger_tags and not detection.id[0] in self.close_chargers:
                self.charger_tags[detection.id[0]] = position

            elif not detection.id[0] in self.reference_tags:
                # Gets the forward direction
                msg = AprilTagDetection()
                
                msg.id = detection.id
                msg.size = detection.size
                # print(position[0])
                msg.pose.pose.pose.position.x = position[0]
                msg.pose.pose.pose.position.y = position[1]
                msg.pose.pose.pose.position.z = 0
                msg.pose.pose.pose.orientation = detection.pose.pose.pose.orientation

                self.positions.detections.append(msg)
                robot_names.data.append(String())

                active_dict[str(detection.id[0])] = self.robot_dictionary[str(detection.id[0])]
                robot_names.data[-1].data = self.robot_dictionary[str(detection.id[0])]
                
        self.active_dict = active_dict
        self.pos_pub.publish(self.positions)
        self.active_pub.publish(robot_names)

    def quaternion_from_rpy(self,roll, pitch, yaw):
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

    def getTheta(self, pt11,pt12,pt21,pt22,):
        vec1 = pt11 - pt12
        vec2 = pt22 - pt21
        vec12dt = math.degrees(math.atan2(np.cross(vec1, vec2),
                            np.dot(vec1, vec2))) + 180
        return vec12dt

    def draw1ine(self, img,point1,point2,clr,):
        corner1 = tuple(point1.ravel().astype(int))
        corner2 = tuple(point2.ravel().astype(int))
        img = cv2.line(img, corner2, corner1, clr, 1)
        return img

    def draw1(self, img,point1,point2,clr,):
        corner1 = tuple(point1.ravel().astype(int))
        corner2 = tuple(point2.ravel().astype(int))
        img = cv2.arrowedLine(img, corner2, corner1, clr, 2)
        return img

    def draw(self, img, corners):
        corner1 = tuple(corners[0].ravel().astype(int))
        corner2 = tuple(corners[1].ravel().astype(int))
        corner3 = tuple(corners[3].ravel().astype(int))

        img = cv2.line(img, corner1, corner2, (255, 0, 0), 2)
        img = cv2.line(img, corner1, corner3, (0, 255, 0), 2)

        return img
    
    # Uses the transform matrix above to transform points
    def transform(self, matrix):
        matrix = np.matmul(self.transform_matrix,np.transpose([matrix[0] - self.orig.x,matrix[1] - self.orig.y]))
        return matrix

    def heading_dir(self, corners, center):
        corner1 = corners[0].ravel()
        corner2 = corners[1].ravel()
        midPt = (corner1 + corner2) / 2
        cMidPt = center - midPt
        theta = math.atan2(cMidPt[1], cMidPt[0])
        cMidPt[0] = cMidPt[0] + 50 * math.cos(theta)
        cMidPt[1] = cMidPt[1] + 50 * math.sin(theta)
        newmidPt = cMidPt + center
        return (newmidPt, theta)

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
        self.transform_matrix = []

        self.display_detections = True

        self.reference_tags = [0, 1, 2] # List that holds the ids of the reference tags
        self.charger_tags = [500]

        self.open_chargers = {}
        self.closed_chargers = []

        self.x_distance = 2.413
        self.y_distance = 1.74625 #67.5 #1.7145

        self.pos_pub = rospy.Publisher("/positions",AprilTagDetectionArray,queue_size=1)
        self.robot_dictionary = None
        with open("/home/michael/Documents/heroswarmv2/ROS1/ros_ws/src/camera_server/scripts/robots.json") as file:
            self.robot_dictionary = json.load(file)

        self.get_charger = rospy.Service("get_charger",GetCharger,self.handle_get_charger)
        self.release_charger = rospy.Service("release_charger",ReleaseCharger,self.handle_release_charger)

        self.active_pub = rospy.Publisher("active_robots",StringList,queue_size=1)

        self.positions = None
        self.active_dict = {}
        self.thread_dict = {}

        self.image_queue = queue.Queue(10)

        self.image_sub = rospy.Subscriber("/tag_detections",AprilTagDetectionArray,self.get_positions)

if __name__ == '__main__':
    server = CameraServer()
    rospy.spin()