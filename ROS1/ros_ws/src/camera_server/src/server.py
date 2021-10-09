#! /usr/bin/python3
from __future__ import division, print_function

import math
import struct
import threading
import time
from argparse import ArgumentParser
from multiprocessing import Process,Queue

import apriltag
import cv2
import getch
import numpy as np
import rospy
from geometry_msgs.msg import Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from robot_msgs.msg import Robot_Pos


class CameraServer():

    def __init__(self):

        self.ref_x = None
        self.ref_y = None
        self.orig = None
        self.rotation_matrix = []
        self.transform_matrix = []

        self.reference_tags = [0, 1, 2] # List that holds the ids of the reference tags

        self.x_distance = 95
        self.y_distance = 67.5

        self.image_queue = Queue(2)

        # self.window = "Overlay1"

        # cv2.namedWindow(self.window)

        rospy.init_node("camera_server",anonymous=True)

        self.parser = ArgumentParser(description='test apriltag Python bindings')
        self.parser.add_argument('device_or_movie', metavar='INPUT', nargs='?', default=0, help='Movie to load or integer ID of camera device')
        apriltag.add_arguments(self.parser)
        self.options = self.parser.parse_args()
        
        self.detector = apriltag.Detector(
            self.options,
            searchpath=apriltag._get_demo_searchpath()
        )

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.fontScale              = 0.3
        self.fontColor              = (0,0,255)
        self.lineType               = 1

        self.pos_pub = rospy.Publisher("Positions",Robot_Pos,queue_size=10)

    def read_frame(self):
        try:
            capture = cv2.VideoCapture(-1)
            W, H = 4096, 2160
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, W)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            capture.set(cv2.CAP_PROP_FPS, 30)
        except ValueError:
            self.cap = cv2.VideoCapture(self.options.device_or_movie)

        while True:
            process_start = time.time()
            _, frame = capture.read()
            self.image_queue.put(frame)
            print("Process: ",time.time() - process_start)

    def get_positions(self):
        while True:
            if not self.image_queue.empty():
                print("Running")
                frame = self.image_queue.get()
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                detections, dimg = self.detector.detect(gray, return_image = True)
                # head_dir = np.array([0,0])
                # num_detections = len(detections)
                
                dimg1 = dimg

                positions = Robot_Pos()
                # positions.robot_pos = []
                
                if self.transform_matrix == None:
                    try:
                        # Gets the x and y positions.robot_pos of the reference tags
                        self.ref_x = detections[self.reference_tags[1]].center
                        self.ref_y = detections[self.reference_tags[2]].center
                        self.orig = detections[self.reference_tags[0]].center
                    except IndexError:
                        continue

                    self.transform_matrix.append(np.abs(self.x_distance) / np.abs(self.ref_x[0] - self.orig[0]))
                    self.transform_matrix.append(np.abs(self.y_distance) / np.abs(self.orig[1] - self.ref_y[1]))

                    # Creates the rotation matrix
                    self.rotation_matrix = np.array([[0, 1], [-1, 0]])
                
                for detection in detections:
                    # dimg1 = self.draw(frame, detection.corners)
                    center = detection.center
                    
                    # Gets the center of the tag in inches and rotated accordingly

                    center_transform = self.transform(center)

                    # posString = '({x:.2f},{y:.2f})'.format(x=center_transform[0],y=center_transform[1])

                    if not detection.tag_id in self.reference_tags:

                        # Gets the forward direction
                        (forward_dir, angle) = self.heading_dir(detection.corners, center)

                        # forward_dir_transform = self.transform(forward_dir)

                        # Draws the arrows

                        # dimg1 = self.draw1(dimg1, forward_dir, center, (0, 0,255))

                        # center_txt = center.ravel().astype(int).astype(str)
                        # cv2.putText(dimg1,posString,tuple(center.ravel().astype(int) + 10),self.font,self.fontScale,(255, 0, 0),self.lineType)

                        # cv2.putText(dimg1,'Id:' + str(detection.tag_id),tuple(center.ravel().astype(int)),self.font,0.8,(0, 0, 0),2,)

                        positions.robot_pos.append(Odometry())
                        positions.robot_pos[-1].child_frame_id = str(detection.tag_id)

                        if not detection.tag_id in self.reference_tags:
                            
                            positions.robot_pos[-1].pose.pose.position.x = center_transform[0]
                            positions.robot_pos[-1].pose.pose.position.y = 0 
                            positions.robot_pos[-1].pose.pose.position.z = center_transform[1]

                            q = self.quaternion_from_rpy(0,0,np.radians(angle))

                            positions.robot_pos[-1].pose.pose.orientation.x = q[0]
                            positions.robot_pos[-1].pose.pose.orientation.y = q[1]
                            positions.robot_pos[-1].pose.pose.orientation.z = q[2]
                            positions.robot_pos[-1].pose.pose.orientation.w = q[3]

                        else:

                            positions.robot_pos[-1].pose.pose.position.x = center_transform[0]
                            positions.robot_pos[-1].pose.pose.position.y = 0 
                            positions.robot_pos[-1].pose.pose.position.z = center_transform[1]
                        
                self.pos_pub.publish(positions)
                
                
                # cv2.imshow(self.window, overlay)
                # cv2.waitKey(1)



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
        theta = math.degrees(math.atan2(cMidPt[1], cMidPt[0]))
        cMidPt[0] = cMidPt[0] + 50 * math.cos(math.radians(theta))
        cMidPt[1] = cMidPt[1] + 50 * math.sin(math.radians(theta))
        newmidPt = cMidPt + center
        return (newmidPt, theta)

if __name__ == '__main__':
        try:
            server = CameraServer()
            camera_process = Process(target=server.read_frame,args=())
            camera_process.daemon = True
            camera_process.start()
            time.sleep(3)
            server.get_positions()
        except rospy.ROSInterruptException:
            pass
