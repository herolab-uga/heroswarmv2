import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class openCV_publisher(Node):
        
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        super().__init__('openCV_publisher')
        self.publisher = self.create_publisher(Image, 'opencv_publisher', 10)
        self.conv = CvBridge()
        self.read_frame()
        
    def read_frame(self):
        while True:
            ret, img = self.cap.read()
            cv2.imshow('Video', img)
            self.publisher.publish(self.conv.cv2_to_imgmsg(img, "bgr8"))
            if not self.cap.isOpened():
                print("Can't read frame, exiting...")
                break
            # self.get_logger().info('Publishing Video Frame')
            
        
def main(args=None):
    rclpy.init(args=args)
    opencv_publisher = openCV_publisher()
    rclpy.spin(opencv_publisher)
    opencv_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()                         