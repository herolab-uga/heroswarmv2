#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import random


class MotorTest(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/swarmbot/cmd_vel', 10)
        timer_period = 1  # seconds

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(10)

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = random.uniform(-.3, .3)
        msg.angular.z = random.uniform(-0.9, 0.9)
        print("Setting X: {} | Ang Z: {}".format(msg.linear.x, msg.angular.z))
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    motor_test = MotorTest()

    rclpy.spin(motor_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
