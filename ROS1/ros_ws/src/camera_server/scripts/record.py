#! /usr/bin/python3
from __future__ import division, print_function
from time import time

import cv2
import numpy as np
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
import random
import time
from cv_bridge import CvBridge

robot_color = {}
image = np.empty((2160,4096,3), np.uint8)
image.fill(255)
pose_image_pub = None
bridge = None

def position_callback(msg):
    for robot in msg.detections:
        if not robot.id[0] in robot_color.keys():
            robot_color[robot.id[0]] = (int(random.uniform(0,255)),int(random.uniform(0,255)),int(random.uniform(0,255)))
        position = [
            int(4096*(robot.pose.pose.pose.position.x+.105)/3.108),
            int(2160*(robot.pose.pose.pose.position.y/1.74625))
        ]
        # position = np.matmul(np.transpose(position),[[1,0],[0,-1]])
        cv2.circle(image,(position[0],2160-abs(position[1])), radius=5, color=robot_color[robot.id[0]], thickness=-1)
    # image_bgra = np.concatenate([image, np.full((2160, 4096, 1), 255, dtype=np.uint8)], axis=-1)
    # white = np.all(image_bgra == [255, 255, 255], axis=-1)
    # image_bgra[white,-1] = 0
    pose_image_pub.publish(bridge.cv2_to_imgmsg(image, encoding="rgb8"))
         

if __name__ == "__main__":
    rospy.init_node("record")
    bridge = CvBridge()
    position_sub = rospy.Subscriber("positions",AprilTagDetectionArray,position_callback)
    pose_image_pub = rospy.Publisher("pose_image",Image,queue_size=1)
    # image_sub = rospy.Subscriber("/tag_dectections_image",Image)
    rospy.spin()