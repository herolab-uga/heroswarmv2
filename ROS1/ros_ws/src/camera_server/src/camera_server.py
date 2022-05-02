#! /usr/bin/python3
from __future__ import division, print_function

import json
import math
import threading
from argparse import ArgumentParser
from multiprocessing import Process, Queue

import apriltag
import cv2
import numpy as np
import distributed_controller as distributed_controller 
import rospy
from nav_msgs.msg import Odometry
from robot_msgs.msg import Robot_Pos, StringList
from robot_msgs.srv import GetCharger, GetChargerResponse, ReleaseCharger, ReleaseChargerResponse 
from std_msgs.msg import String
from geometry_msgs import Pose


class CameraServer():

    def read_frame(self,image_queue):
        try:
            capture = cv2.VideoCapture(-1)
            W, H = 4096, 2160
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, W)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            capture.set(cv2.CAP_PROP_FPS, 60)
        except ValueError:
            self.cap = cv2.VideoCapture(self.options.device_or_movie)

        while True:
            _, frame = capture.read()
            if image_queue.empty():
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                
                image_queue.put(self.detector.detect(gray, return_image = True))

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
            # sub = [sub_bot for sub_bot in prev_active if sub_bot not in active_dict]
            prev_active = active_dict
            # for missing_robot in sub:
            #     try:
            #         self.thread_dict[missing_robot].halt_pos_pub()
            #         del missing_robot
            #     except KeyError:
            #         print("Could not find controller for tag: {}".format(missing_robot))

    def get_positions(self,image_queue):
        while True:
            if not image_queue.empty():

                detections, dimg = image_queue.get()
                
                dimg1 = dimg

                self.positions = Robot_Pos()
                
                if self.transform_matrix == None:
                    try:
                        # Gets the x and y positions.robot_pos of the reference tags
                        self.ref_x = detections[self.reference_tags[1]].center
                        self.ref_y = detections[self.reference_tags[2]].center
                        self.orig = detections[self.reference_tags[0]].center

                        self.transform_matrix = [(np.abs(self.x_distance) / np.abs(self.ref_x[0] - self.orig[0])),
                                                    (np.abs(self.y_distance) / np.abs(self.orig[1] - self.ref_y[1]))]

                        # Creates the rotation matrix
                        self.rotation_matrix = np.array([[0, 1], [-1, 0]])
                    except IndexError:
                        continue
                    
                robot_names = StringList()
                active_dict = {}
                # print(detections)
                for detection in detections:

                    center = detection.center
                    
                    # Gets the center of the tag in inches and rotated accordingly

                    center_transform = self.transform(center)

                    # posString = '({x:.2f},{y:.2f})'.format(x=center_transform[0],y=center_transform[1])
                    if detection.tag_id in self.charger_tags and not detection.tag_id in self.close_chargers:
                        
                        (forward_dif,angle) = self.heading_dir(detection.corners,center)

                        temp = Pose()
                        
                        temp.position.x = center_transform[0]
                        temp.position.y = center_transform[1]
                        temp.position.z = 0.0

                        q = self.quaternion_from_rpy(0,0,angle)

                        temp.orientation.x = q[0]
                        temp.orientation.y = q[1]
                        temp.orientation.z = q[2]
                        temp.orientation.w = q[3]

                        
                        self.charger_tags[detection.tag_id] = temp

                    elif not detection.tag_id in self.reference_tags:
                        # Gets the forward direction
                        (forward_dir, angle) = self.heading_dir(detection.corners, center)


                        self.positions.robot_pos.append(Odometry())
                        robot_names.names.append(String())
                        self.positions.robot_pos[-1].child_frame_id = str(detection.tag_id)

                        active_dict[str(detection.tag_id)] = self.robot_dictionary[str(detection.tag_id)]
                        robot_names.names[-1].data = self.robot_dictionary[str(detection.tag_id)]

                        self.positions.robot_pos[-1].pose.pose.position.x = center_transform[0]
                        self.positions.robot_pos[-1].pose.pose.position.y = center_transform[1] 
                        self.positions.robot_pos[-1].pose.pose.position.z = 0.0

                        q = self.quaternion_from_rpy(0,0,angle)

                        self.positions.robot_pos[-1].pose.pose.orientation.x = q[0]
                        self.positions.robot_pos[-1].pose.pose.orientation.y = q[1]
                        self.positions.robot_pos[-1].pose.pose.orientation.z = q[2]
                        self.positions.robot_pos[-1].pose.pose.orientation.w = q[3]
                        
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

    def handle_get_charger(self,req):

        open_charger = list(self.open_chargers)[0]
        self.closed_chargers.append(open_charger)
        return GetChargerResponse(open_charger)

    def handle_release_charger(self,req):

        self.closed_chargers.remove(req.id)
        return ReleaseChargerResponse(True)

    def __init__(self):

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

        rospy.init_node("camera_server",anonymous=True)

        self.parser = ArgumentParser(description='test apriltag Python bindings')

        self.parser.add_argument('device_or_movie', 
                                    metavar='INPUT', 
                                    nargs='?', 
                                    default=0, 
                                    help='Movie to load or integer ID of camera device')

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

        self.pos_pub = rospy.Publisher("/positions",Robot_Pos,queue_size=1)

        self.robot_dictionary = None
        with open("/home/michaelstarks/Documents/heroswarmv2/ROS1/ros_ws/src/camera_server/src/robots.json") as file:
            self.robot_dictionary = json.load(file)

        get_charger = rospy.Service("get_charger",GetCharger,self.handle_get_charger)
        release_charger = rospy.Serive("release_charger",ReleaseCharger,self.handle_release_charger)

        self.active_pub = rospy.Publisher("active_robots",StringList,queue_size=1)
        self.positions = None
        self.active_dict = {}
        self.thread_dict = {}
        
        self.connection_manager_thread = threading.Thread(target=self.connection_manager,args=())
        self.connection_manager_thread.start()
        
        self.image_queue = Queue(maxsize=1)
        
        self.camera_process = Process(target=self.read_frame,args=(self.image_queue,))
        self.camera_process.start()

        self.position_tracking_thread = threading.Thread(target=self.get_positions,args=(self.image_queue,),daemon=True)
        self.position_tracking_thread.start()

if __name__ == '__main__':
        try:
            server = CameraServer()
        except rospy.ROSInterruptException:
            pass
