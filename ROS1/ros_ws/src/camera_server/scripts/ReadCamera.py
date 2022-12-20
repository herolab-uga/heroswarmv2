#! /usr/bin/python3
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import time


 
def read_frame():
    rospy.init_node("read_camera",anonymous=True)
    image_pub = rospy.Publisher("/camera/image_raw",Image,queue_size=1)
    bridge = CvBridge()
    try:
        capture = cv2.VideoCapture(-1)
        W, H = 4096, 2160
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        capture.set(cv2.CAP_PROP_FPS, 60)
    except ValueError:
        print("error")
        return

    while not rospy.is_shutdown():
        _, frame = capture.read()
        msg = bridge.cv2_to_imgmsg(frame,"bgr8")
        msg.header.stamp = rospy.Time.now()
        image_pub.publish(msg)
            


def main():
    read_frame()

if __name__ == "__main__":
    main()
