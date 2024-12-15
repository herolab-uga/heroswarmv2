import rclpy

from rclpy.node import Node 
from geometry_msgs.msg import Twist 

class TestPublisher(Node):

    def __init__(self): 

        super().__init__('test_publisher') 
        self.pub = self.create_publisher(Twist, 'swarmgoblin/cmd_vel', 10) 
        self.start() 

    def start(self): 

        twist_msg = Twist() 
        twist_msg.linear.x = 1.0

        while True: 

            self.pub.publish(twist_msg) 

def main(args=None): 

    rclpy.init(args=args) 
    publisher = TestPublisher() 
    rclpy.spin(publisher)
    publisher.destroy_node() 
    rclpy.shutdown() 

if __name__=='__main__': 

    main()
