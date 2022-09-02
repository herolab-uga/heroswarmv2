#! /bin/python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image 

class CameraCapture:
    def capture(self):
        while not rospy.is_shutdown():
            _,frame = self.video.read()

            msg = Image()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "camera_frame"
            msg.encoding = "bgr8"
            msg.height = self.H
            msg.width = self.W
            msg.data = self.bridge.cv2_to_imgmsg(frame, "bgr8")

    def __init__(self) -> None:
        rospy.init_node("Camera Caputure", anonymous=True)

        self.image_pub = rospy.Publisher("camera/image_raw", Image, queue_size=1)
        self.video = cv2.VideoCapture(-1)
        self.W, self.H = 4096, 2160
        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, self.W)
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)
        self.video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.video.set(cv2.CAP_PROP_FPS, 60)
        self.bridge = CvBridge()

        self.capture()
            