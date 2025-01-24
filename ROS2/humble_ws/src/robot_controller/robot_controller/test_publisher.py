import rclpy
import time

from rclpy.node import Node 
from geometry_msgs.msg import Twist 

class TestPublisher(Node):

    def __init__(self): 

        super().__init__('test_publisher') 
        self.pub = self.create_publisher(Twist, 'swarmphoenix/cmd_vel', 10)
        self.start_time = time.perf_counter() 
        self.start() 

    def start(self):        

        twist_msg = Twist() 
        #twist_msg.linear.x = -1.0        
        
        while True:             
            current_time = time.perf_counter()
            if current_time > self.start_time + 10:
                twist_msg.linear.y = 0.0                
            else:
                twist_msg.linear.y = 2.0
                
            self.pub.publish(twist_msg) 

def main(args=None): 

    rclpy.init(args=args) 
    publisher = TestPublisher() 


    # testing here

    rclpy.spin(publisher)
    publisher.destroy_node() 
    rclpy.shutdown() 

if __name__=='__main__': 

    main()
