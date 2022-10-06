import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ReadCamera(Node):
    def read_camera(self):
        bridge = CvBridge()
        capture = cv2.VideoCapture(-1)
        W, H = 4096, 2160
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        capture.set(cv2.CAP_PROP_FPS, 60)
        while True:
            ret, frame = capture.read()
            if ret:
                self.image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))
            else:
                self.get_logger().warn('Cannot read camera')
                break
        capture.release()

    def __init__(self):
        super().__init__('ReadCamera')
        self.get_logger().info('ReadCamera node has been started')
        self.image_pub = self.publisher(Image, '/image', 1)
        while True:
            self.read_camera()

def main():
    ReadCamera()
