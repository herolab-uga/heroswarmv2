import rclpy, time

from rclpy.node import Node 
from geometry_msgs.msg import Twist 

class TestPublisher(Node):

    def __init__(self): 

        super().__init__('test_publisher') 
        self.pub0 = self.create_publisher(Twist, 'swarmstarburst/cmd_vel', 10) 
        self.pub1 = self.create_publisher(Twist, 'swarmharley/cmd_vel', 10)
        self.start() 

    def start(self): 

        twist_msg = Twist() 
        # go straight
        twist_msg.linear.x = 1.0
       	self.pub0.publish(twist_msg)
        self.pub1.publish(twist_msg)
        time.sleep(5)
        # turn
        twist_msg.angular.z = 1.57
        twist_msg.linear.x = 0.0
        self.pub0.publish(twist_msg)
        self.pub1.publish(twist_msg)
        time.sleep(5)
        # go straight
        twist_msg.angular.z = 0.0
        twist_msg.linear.x = 1.0
        self.pub0.publish(twist_msg)
        self.pub1.publish(twist_msg)
        time.sleep(5)
        # turn
        twist_msg.angular.z = 1.57
        twist_msg.linear.x = 0.0
        self.pub0.publish(twist_msg)
        self.pub1.publish(twist_msg)
        time.sleep(5)
        # go straight
        twist_msg.angular.z = 0.0
        twist_msg.linear.x = 1.0
        self.pub0.publish(twist_msg)
        self.pub1.publish(twist_msg)
        time.sleep(5)



def main(args=None): 

    rclpy.init(args=args) 
    publisher = TestPublisher() 
    rclpy.spin(publisher)
    publisher.destroy_node() 
    rclpy.shutdown() 

if __name__=='__main__': 

    main()
