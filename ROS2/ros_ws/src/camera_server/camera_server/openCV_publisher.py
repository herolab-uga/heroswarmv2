import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class openCV_publisher(Node):
        
    def __init__(self):
        self.cap = cv2.VideoCapture(-1)
        # self.cap.open(0 + cv2.CAP_DSHOW)
        W, H = 1920, 1080
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        super().__init__('openCV_publisher')
        self.publisher = self.create_publisher(Image, 'opencv_publisher', 10)
        self.conv = CvBridge()
        self.read_frame()
        
    def read_frame(self):
        while True:
            ret, img = self.cap.read()
            # cv2.imshow('Video', img)
            # cv2.waitKey(30)
            self.publisher.publish(self.conv.cv2_to_imgmsg(img, "bgr8"))
            if not self.cap.isOpened():
                print("Can't read frame, exiting...")
                break
            # self.get_logger().info('Publishing Video Frame')
            
        
def main(args=None):
    rclpy.init(args=args)
    opencv_publisher = openCV_publisher()
    # rclpy.spin(opencv_publisher)
    # opencv_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()                         