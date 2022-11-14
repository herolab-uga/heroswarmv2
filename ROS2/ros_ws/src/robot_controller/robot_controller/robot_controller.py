#! /usr/bin/python3

import math
import struct
import threading
import time
import json
import numpy as np
import rclpy
import SensorNode

from geometry_msgs.msg import Twist, Point

from std_msgs.msg import Int16, String, Float32, Int16MultiArray

class Controller:

    def __del__(self):
        self.send_values([0, 0, 0])

    def get_pos_global(self, msg):
        for robot in msg.robot_pos:
            if robot.child_frame_id == str(self.id):
                self.position["x"] = robot.pose.pose.position.x
                self.position["y"] = robot.pose.pose.position.y
                self.position["orientation"] = \
                    -self.rpy_from_quaternion(robot.pose.pose.orientation)[2]
                break
        # rospy.loginfo("Global {X: {x} Z: {z} Theta: {theta}"+"}".format(
            # x=self.position["x"], z=self.position["y"], theta=self.position["orientation"]))

    def get_pos(self, msg):
        self.position["x"] = msg.pose.pose.position.x
        self.position["y"] = msg.pose.pose.position.y
        self.position["orientation"] = - \
            self.rpy_from_quaternion(msg.pose.pose.orientation)[2]

        # rospy.loginfo("X: {x} Z: {z} Theta: {theta}".format(
            # x=self.position["x"], z=self.position["y"], theta=self.position["orientation"]))

    def read_twist(self, msg, event=None) -> None:
        if self.stop_timer != None:
            self.stop_timer.cancel()
        # Reads ths twist message x linear velocity
        if not msg.linear.x == 0:
            direction_lin = msg.linear.x / abs(msg.linear.x)
            x_velo = direction_lin * \
                (abs(msg.linear.x) if abs(msg.linear.x) <= .10 else .10)
        else:
            x_velo = 0

        # Reads the twist message z angular velocity
        if not msg.angular.z == 0:
            direction_ang = msg.angular.z / abs(msg.angular.z)
            z_angular = direction_ang * \
                (abs(msg.angular.z) if abs(msg.angular.z) <= 1.85 else 1.85)
        else:
            z_angular = 0
        
        # self.stop_timer = threading.Timer(0.5,self.stop)
        # self.stop_timer.start()

        if not (x_velo == self.linear_x_velo and z_angular == self.angular_z_velo):
            # Logs the data
            self.get_logger().info("X Linear: {x} Y Linear: {y} Z Angular: {z}".format(
                x=x_velo, y=0, z=z_angular))
            # Sends the velocity information to the feather board
            self.send_values([x_velo, 0, z_angular])

    def stop(self):
        self.send_values([0, 0, 0])
        self.stop_timer = None

    # Sending an float to the arduino
    # Message format [msgid , args]
    def send_values(self, values=None, opcode=0):
        # Work around for demo remove later
        # Converts the values to bytes
        byteList = struct.pack("f", opcode) + \
            struct.pack('f'*len(values), *values) + struct.pack('f',0.0)
        # fails to send last byte over I2C, hence this needs to be added
        try:
            while not self.i2c.try_lock():
                pass
            # Writes the values to the i2c
            self.i2c.writeto(self.arduino, byteList, stop=False)

            if opcode == 0:
                self.linear_x_velo = values[0]

                self.linear = values[1]

                self.angular_z_velo = values[2]
        except OSError as e:
            self.get_looger().info(e)
            self.get_logger().loginfo("Could not send message: {opcode} {data}".format(
                opcode=opcode, data=values))
        finally:
            self.i2c.unlock()

    # This has to use the global position
    def move_to_angle(self, angle):
        rate = self.create_rate(10)
        delta_theta = self.position["theta"] - angle
        while delta_theta > 0.05:
            delta_theta = self.position["theta"] - angle
            self.send_values([0.0, 0.0, delta_theta])
            rate.sleep()
        self.send_values([0.0, 0.0, 0.0])

    # Position controller
    def move_to_point(self, msg):
        self.move_to_point_handler(msg.x, msg.y)

    def move_to_point_handler(self, x, y):
        current_x = self.position["x"]
        current_y = self.position["y"]
        theta = self.position["orientation"]

        self.get_logger().info("X: {x} Y: {y}".format(x=current_x, y=current_y))
        if math.sqrt(math.pow((x - current_x), 2) + math.pow((y - current_y), 2)) < .05:
            self.send_values([0, 0, 0])
        else:

            # Gets the difference between the current position and desired position
            delta_x = x - current_x
            delta_y = y - current_y
            # Gets the time such that the robot would move to the point at v_max
            t = math.sqrt(
                (math.pow(delta_x, 2) + math.pow(delta_y, 2)) / math.pow(self.v_max, 2))
            # Gets the velocities
            x_velo = delta_x / t
            y_velo = delta_y / t

            # Calculates the sine and cosine of the current theta
            a = np.cos(theta)
            b = np.sin(theta)

            # Finds the linear velocity
            v = 1*(x_velo*a + y_velo*b)

            # Finds the angular velocity
            omega = self.omega_max * \
                np.arctan2(-b*x_velo + a*y_velo, v) / (np.pi/2)

            self.send_values([v, 0, omega])

    def neopixel_callback(self, msg):
        self.send_values(msg.data, 1.0)

    def pub_battery(self, timer):
        battery_msg = Float32()
        battery_msg.data = self.sensor_data["battery"]
        self.battery_pub.publish(battery_msg)

    def __init__(self):
        super.__init__("robot_controller", anonymous=True)

        # Arduino Device Address
        self.arduino = 0x08

        self.name = rospy.get_namespace()

        self.id = None

        with open("/home/pi/heroswarmv2/ROS1/ros_ws/src/robot_controller/scripts/robots.json") as file:
            robot_dictionary = json.load(file)
            for key in robot_dictionary:
                if robot_dictionary[key] == self.name:
                    self.id = int(key)

        self.position = {
            "x": 0,
            "y": 0,
            "orientation": 0
        }

        self.sensor_data = {
            "temp": 0.0,
            "pressure": 0.0,
            "humidity": 0.0,
            "altitude": 0.0,
            "rgbw": [],
            "gesture": 0,
            "prox": 0,
            "battery": None,
            "mic":0
        }

        self.linear_x_velo = None
        self.linear_y_velo = None
        self.angular_z_velo = None
        self.v_max = 0.1
        self.omega_max = 1.0

        self.open_chargers = None
        self.IMU = None

        # Creates subscribers for positions topics
        if rospy.get_param(self.name + "controller/global_pos") == True:
            self.pos_sub_global = rospy.Subscriber(
                "/positions", Robot_Pos, self.get_pos_global)
        else:
            self.pos_sub_namespace = rospy.Subscriber(
                "position", Odometry, self.get_pos)

        # Creates the twist publisher
        self.twist_sub = self.Subscriber(Twist,"cmd_vel", self.read_twist,queue_size=1)

        # Creates the auto-stop timer
        self.stop_timer = None

        # Publish the battery level on the battery topic with at 5hz
        self.battery_timer = self.create_timer(.2,self.pub_battery)

        # Creates position control topic
        self.position_sub = self.Subscriber(
            Point,"to_point", self.move_to_point)

        # Creates shutdown hook
        self.shutdown_sub = self.Subscriber(
            String,"shutdown", self.shutdown_callback)

        self.init_sensors()

        # Read sensors
        # self.sensor_read_thread = mp.Process(
        #     target=self.read_sensors, args=(self.sensor_data,self.sensor_queue,self.i2c))
        # self.sensor_read_thread.start()

        self.neopixel_subscriber = self.Subscriber(
            Int16MultiArray,"neopixel", self.neopixel_callback)

        self.get_logger().info("Ready")


if __name__ == '__main__':
    controller = Controller()
    sensorNode = SensorNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(sensorNode)
    executor.spin()