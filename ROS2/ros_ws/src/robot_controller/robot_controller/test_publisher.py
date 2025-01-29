import rclpy
import time

from rclpy.node import Node
from geometry_msgs.msg import Twist


class TestPublisher(Node):

    def __init__(self):

        super().__init__('test_publisher')
        self.pub0 = self.create_publisher(Twist, 'swarmviper/cmd_vel', 10)
        self.start_time = time.time()
        self.start()

    def start(self):
        twist_msg = Twist()

        while time.time() - self.start_time <= 10:
            # go straight
            twist_msg.linear.y = 1.0
            twist_msg.angular.z = 0.0
            end_time = time.time()

       	    self.pub0.publish(twist_msg)
            time.sleep(0.1)

        while time.time() - self.start_time <= 11:
            twist_msg.linear.y = 0.0
            twist_msg.angular.z = 1.0
            end_time = time.time()

       	    self.pub0.publish(twist_msg)
            time.sleep(0.1)

        while time.time() - self.start_time <= 22:
            # go straight
            twist_msg.linear.y = 1.0
            twist_msg.angular.z = 0.0
            end_time = time.time()

       	    self.pub0.publish(twist_msg)
            time.sleep(0.1)
        """
        while time.time() - self.start_time <= 24:
            twist_msg.linear.y = 0.0
            twist_msg.angular.z = 1.0
            end_time = time.time()

       	    self.pub0.publish(twist_msg)
            time.sleep(0.1)

        while time.time() - self.start_time <= 34:
            # go straight
            twist_msg.linear.y = 1.0
            twist_msg.angular.z = 0.0
            end_time = time.time()

       	    self.pub0.publish(twist_msg)
            time.sleep(0.1)

        while time.time() - self.start_time <= 36:
            twist_msg.linear.y = 0.0
            twist_msg.angular.z = 1.0
            end_time = time.time()

       	    self.pub0.publish(twist_msg)
            time.sleep(0.1)
        while time.time() - self.start_time <= 46:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            end_time = time.time()

       	    self.pub0.publish(twist_msg)
            time.sleep(0.1)
        """
        if time.time() - self.start_time >= 30:
            return

def main(args=None):

    rclpy.init(args=args)
    publisher = TestPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__=='__main__': 

    main()
