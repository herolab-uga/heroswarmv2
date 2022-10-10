#! /usr/bin/python3
from __future__ import division, print_function
from asyncore import read

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
import distributed_controller as distributed_controller 
import rospy
from nav_msgs.msg import Odometry
from robot_msgs.msg import Robot_Pos, StringList
from robot_msgs.srv import GetCharger, GetChargerResponse, ReleaseCharger, ReleaseChargerResponse 
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

def read_frame(self,image_queue):
        image_pub
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
            _, frame = capture.read()
            if image_queue.empty():
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

if __name__ == "__main__":
    try:
        read_frame()
    except rospy.ROSInterruptException:
        pass