#! /usr/bin/python3
from __future__ import division, print_function

import json
import math
import threading
from argparse import ArgumentParser
from multiprocessing import Process, Queue
from time import time

import apriltag
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from robot_msgs.msg import RobotPos, StringList
# from robot_msgs.srv import GetCharger, GetChargerResponse, ReleaseCharger, ReleaseChargerResponse 
# from isaac_ros_apriltag.msg import AprilTagDetectionArray
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time


class CameraServer(Node):
                
    def connection_manager(self,active_dict,robot_dictionary):
        prev_active = []
        count = 0
        thread_dict = None
        while True:
            active_dict = list(active_dict)
            add = [add_bot for add_bot in active_dict if add_bot not in prev_active]
            for new_robot in add:
                thread_dict[new_robot] = threading.Thread(target=distributed_controller.DistributedController, args=(new_robot, 
                                                            robot_dictionary[new_robot]),daemon=True)
                thread_dict[new_robot].start()
                count = count + 1
            prev_active = active_dict

    def get_positions(self,msg):

        frame = self.bridge.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        detections = self.detector.detect(gray)

        dimg1 = frame
    
        self.positions = RobotPos()
        
        if self.transform_matrix == None:
            try:
                # Gets the x and y positions.robot_pos of the reference tags
                self.ref_x = detections[self.reference_tags[1]]["center"]
                self.ref_y = detections[self.reference_tags[2]]["center"]
                self.orig = detections[self.reference_tags[0]]["center"]

                self.transform_matrix = [(np.abs(self.x_distance) / np.abs(self.ref_x[0] - self.orig[0])),
                                            (np.abs(self.y_distance) / np.abs(self.orig[1] - self.ref_y[1]))]

                # Creates the rotation matrix
                self.rotation_matrix = np.array([[0, 1], [-1, 0]])
            except IndexError:
                return
            
        robot_names = StringList()
        active_dict = {}
        # print(detections)
        for detection in detections:
             #making sure to ignore the tags on charging table 
            if(detection["center"][0] <= self.orig[0] - 100):
                continue
            dimg1 = self.draw(frame,detection["lb-rb-rt-lt"])

            center = detection["center"]
            
            # Gets the center of the tag in inches and rotated accordingly

            center_transform = self.transform(center)

            cv2.putText(dimg1,'Id:'+str(detection["id"]), tuple((center.ravel()).astype(int)),self.font,0.8,(150,150,0),2)

            posString = '({x:.2f},{y:.2f})'.format(x=center_transform[0],y=center_transform[1])
           
            if detection["id"] in self.charger_tags and not detection["id"]in self.close_chargers:
                # posString = "({x:.2f},{y:.2f})".format(x=center[0],y=center[1])
                
                (forward_dir,angle) = self.heading_dir(detection.corners,center)
                dimg1=self.draw1(dimg1,forward_dir,center,(0,0,255))
                cv2.putText(dimg1,posString, tuple((center.ravel()).astype(int)+10),self.font,self.fontScale,(255,0,0),self.lineType)
                temp = Pose()
                
                temp.position.x = center_transform[0]
                temp.position.y = center_transform[1]
                temp.position.z = 0.0

                q = self.quaternion_from_rpy(0,0,angle)

                temp.orientation.x = q[0]
                temp.orientation.y = q[1]
                temp.orientation.z = q[2]
                temp.orientation.w = q[3]

                
                self.charger_tags[detection["id"]] = temp

            elif not detection["id"] in self.reference_tags:

                robot_names.data.append(String())
                
                try:
                    active_dict[str(detection["id"])] = self.robot_dictionary[str(detection["id"])]
                    robot_names.data[-1].data = self.robot_dictionary[str(detection["id"])]
                except KeyError:
                    continue

                # Gets the forward direction
                (forward_dir, angle) = self.heading_dir(detection["lb-rb-rt-lt"], center)
                # posString = "({x:.2f},{y:.2f})".format(x=center[0],y=center[1])
                dimg1=self.draw1(dimg1,forward_dir,center,(0,0,255))
                cv2.putText(dimg1,posString, tuple((center.ravel()).astype(int)+10),self.font,self.fontScale,(255,0,0),self.lineType)

                self.positions.robot_pos.append(Odometry())
                self.positions.robot_pos[-1].child_frame_id = str(detection["id"])
                    
                self.positions.robot_pos[-1].pose.pose.position.x = center_transform[0]
                self.positions.robot_pos[-1].pose.pose.position.y = center_transform[1] 
                self.positions.robot_pos[-1].pose.pose.position.z = 0.0

                q = self.quaternion_from_rpy(0,0,angle)

                self.positions.robot_pos[-1].pose.pose.orientation.x = q[0]
                self.positions.robot_pos[-1].pose.pose.orientation.y = q[1]
                self.positions.robot_pos[-1].pose.pose.orientation.z = q[2]
                self.positions.robot_pos[-1].pose.pose.orientation.w = q[3]

            #if the ID is not in the active_dict
            if detection["id"] not in active_dict:
                #add the newly detected robot to the list
                active_dict[detection["id"]] = True
                #publish the list 
                self.active_pub.publish(robot_names)
        
        #loop through the previous function calls list 
        for rob in self.active_dict:
            #if not in the active_dict list, then it has left the camera's view 
            if rob not in active_dict:
                self.active_dict.remove(rob)
                #publish the active list
                self.active_pub.publish(robot_names)


        self.active_dict = active_dict

        self.pos_pub.publish(self.positions)
        #self.active_pub.publish(robot_names)
        
        if self.display_detections:
            self.detections_pub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(dimg1,(1920,1080)), "bgr8"))

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
        output = np.asarray((self.transform_matrix[0] * (float(matrix[0])
                            - self.orig[0]), self.transform_matrix[1]
                            * (float(matrix[1]) - self.orig[1])))
        return self.rotate(output)

    # Uses the rotation matrix above to rotate points
    def rotate(self, matrix):
        self.ref_x
        self.ref_y
        self.orig
        self.rotation_matrix
        return np.flip(np.matmul(matrix, self.rotation_matrix))

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

    # # Get a charger
    # def handle_get_charger(self,req):
    #     open_charger = list(self.open_chargers)[0]
    #     self.closed_chargers.append(open_charger)
    #     return GetChargerResponse(id = open_charger, position=open_charger)

    # # Release the charger when done charging
    # def handle_release_charger(self,req):

    #     self.closed_chargers.remove(req.id)
    #     return ReleaseChargerResponse(True)

    def __init__(self):
        rclpy.init()
        super().__init__("camera_server")

        self.ref_x = None
        self.ref_y = None
        self.orig = None
        self.rotation_matrix = None
        self.transform_matrix = None

        self.display_detections = True

        self.reference_tags = [0, 1, 2] # List that holds the ids of the reference tags
        self.charger_tags = [500]

        self.open_chargers = {}
        self.closed_chargers = []

        self.x_distance = 2.413
        self.y_distance = 1.74625 #67.5 #1.7145
        
        self.detector = apriltag.apriltag("tagStandard41h12")

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.fontScale              = 0.3
        self.fontColor              = (0,0,255)
        self.lineType               = 1

        self.pos_pub = self.create_publisher(RobotPos,"/positions",1)
        self.bridge = CvBridge()

        # CV subscriber
        self.subscription = self.create_subscription(Image, 'opencv_publisher', self.get_positions, 10)
        self.subscription

        self.robot_dictionary = None
        with open("/home/michael/Documents/heroswarmv2/ROS2/ros_ws/src/camera_server/camera_server/robots.json") as file:
            self.robot_dictionary = json.load(file)

        # self.get_charger = self.Service(GetCharger,"get_charger",self.handle_get_charger)
        # self.release_charger = self.Service(ReleaseCharger,"release_charger",self.handle_release_charger)

        self.active_pub = self.create_publisher(StringList,"/active_robots",1)
        if self.display_detections:
            self.detections_pub = self.create_publisher(Image,"/camera/image_detections",1)

        self.positions = None
        self.active_dict = {}
        self.thread_dict = {}
        
def main():
    server = CameraServer()
    rclpy.spin(server)
    