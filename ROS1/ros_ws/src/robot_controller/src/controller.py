#! /usr/bin/python3

from concurrent.futures import thread
import math
import struct
import threading
import time
import json
import os
import threading

import adafruit_bmp280
import adafruit_lis3mdl
import adafruit_sht31d
import board
import numpy as np
import rospy
import smbus
from adafruit_apds9960.apds9960 import APDS9960
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from robot_msgs.msg import Environment, Light, Robot_Pos
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16, String
from subprocess import call


class Controller:

    def __del__(self):
        self.send_velocity([0, 0, 0])

    def rpy_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def quaternion_from_rpy(self,roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy
        return q

    def get_pos_global(self,msg):
            for robot in msg.robot_pos:
                if robot.child_frame_id == str(self.id):
                    self.position["x"] = robot.pose.pose.position.x
                    self.position["y"] = robot.pose.pose.position.y
                    self.position["orientation"] = -self.rpy_from_quaternion(robot.pose.pose.orientation)[2]
                    break
            rospy.loginfo("Global {X: {x} Z: {z} Theta: {theta}"+"}".format(x=self.position["x"],z=self.position["y"],theta=self.position["orientation"]))

    def get_pos(self,msg):
        self.position["x"] = msg.pose.pose.position.x
        self.position["y"] = msg.pose.pose.position.y
        self.position["orientation"] = -self.rpy_from_quaternion(msg.pose.pose.orientation)[2]
        
        # rospy.loginfo("X: {x} Z: {z} Theta: {theta}".format(x=self.position["x"],z=self.position["y"],theta=self.position["orientation"]))

    def pub_odom(self, timer, event=None):
        # Creates the odom message
        odom_msg = Odometry()

        data = self.bus.read_i2c_block_data(self.arduino, 0)

        odom_data = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Get odom data from arduino
        for index in range(5):
            bytes = bytearray()
            for i in range(4):
                bytes.append(data[4*index + i])
            odom_data[index] = struct.unpack('f', bytes)[0]

        # Adds Twist data
        theta = np.deg2rad(odom_data[2]) #+ self.position["orientation"]
        odom_msg.twist.twist.linear.x = odom_data[3]
        odom_msg.twist.twist.linear.y = odom_data[4]
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = odom_data[4]

        odom_msg.pose.pose.position.x = odom_data[0] #+ self.position["x"] 
        odom_msg.pose.pose.position.y = odom_data[1] #+ self.position["y"]
        odom_msg.pose.pose.position.z = 0.0

        quaternion = self.quaternion_from_rpy(0,0,theta)

        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]


        self.odom_pub.publish(odom_msg)

    def read_twist(self, msg, event=None) -> None:
        x_velo = 0
        z_angular = 0
        with self.velo_lock:
                self.last_call["time"] = time.time()
        # Reads ths twist message x linear velocity
        if not msg.linear.x == 0:
            direction_lin = msg.linear.x / abs(msg.linear.x)
            x_velo = direction_lin * (abs(msg.linear.x) if abs(msg.linear.x) <= .10 else .10)
        else:
            x_velo = 0

        # Reads the twist message y linear velocity
        y_velo = msg.linear.y

        # Reads the twist message z angular velocity
        if not msg.angular.z == 0:
            direction_ang = msg.angular.z / abs(msg.angular.z)
            z_angular = direction_ang * (abs(msg.angular.z) if abs(msg.angular.z) <= 1.85 else 1.85)
        else:
            z_angular = 0
        
        if not (x_velo == self.linear_x_velo and y_velo == self.linear_y_velo and z_angular == self.angular_z_velo):
            # Logs the data
            rospy.loginfo("X Linear: {x} Y Linear: {y} Z Angular: {z}".format(x=x_velo, y=y_velo, z=z_angular))
            # Sends the velocity information to the feather board
            with self.velo_lock:
                self.last_call["time"] = time.time()
            self.send_velocity([x_velo, y_velo, z_angular])
            self.linear_x_velo = x_velo
            self.linear_y_velo = y_velo
            self.angular_z_velo = z_angular
    
    def auto_stop(self):
        while True:
            if self.last_call["time"] == None:
                continue
            elif time.time() - self.last_call["time"] > 0.25:
                if not (self.linear_x_velo == 0 and self.linear_y_velo == 0 and self.angular_z_velo == 0):
                    self.send_velocity([0, 0, 0])
                    self.linear_x_velo = 0
                    self.linear_y_velo = 0
                    self.angular_z_velo = 0
            time.sleep(.1)

    def read_imu(self, freq) -> None:

        # Creates the IMU message
        imu_msg = Imu()
        rate = rospy.Rate(int(freq))
        while not rospy.is_shutdown():
            # Read the sensor
            acc_x, acc_y, acc_z = self.IMU.acceleration
            gyro_x, gyro_y, gyro_z = self.IMU.gyro

            # Sets the self.position["orientation"] parameters (This is wrong)
            # imu_msg.self.position["orientation"].x = 0.0
            # imu_msg.self.position["orientation"].y = 0.0
            # imu_msg.self.position["orientation"].z = 0.0

            # Sets the angular velocity parameters
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z

            # Sets the linear acceleration parameters
            imu_msg.linear_acceleration.x = acc_x
            imu_msg.linear_acceleration.y = acc_y
            imu_msg.linear_acceleration.z = acc_z

            # Publishes the message
            self.imu_pub.publish(imu_msg)
            rate.sleep()

    # Remove DC bias before computing RMS.
    def mean(self, values):
        return sum(values) / len(values)

    def normalized_rms(self, values):
        minbuf = int(self.mean(values))
        samples_sum = sum(
            float(sample - minbuf) * (sample - minbuf)
            for sample in values
        )

        return math.sqrt(samples_sum / len(values))

    # def read_mic(self) -> None:
    #     # Creates the mic message
    #     mic_msg = Int16()

    #     # Sets the meassage data value
    #     mic_msg.data = None

    #     # Publishes the message
    #     self.mic_pub.publish(mic_msg)

    def read_sensors(self):
        rate = rospy.Rate(60)
        while not rospy.is_shutdown:
            self.temp = self.bmp.temperature
            self.pressure = self.bmp.pressure
            self.humidity = self.humidity_sensor.read_humidity
            self.altitude = self.bmp.altitude
            self.rgbw = set(self.light.color_data)
            self.gesture = self.light.gesture()
            self.prox = self.light.proximity
            rate.sleep()

    def read_light(self, timer) -> None:
        # Creates the light message
        light_msg = Light()

        # Sets the current rgbw value array
        light_msg.rgbw = self.rgbw

        # Sets the gesture type
        light_msg.gesture = self.gesture

        # Publishes the message
        self.light_pub.publish(light_msg)
        
    def read_environment(self, timer) -> None:
        # Creates the environment message
        environ_msg = Environment()

        # Sets the temperature
        environ_msg.temp = self.temp

        # Sets the pressure
        environ_msg.pressure = self.pressure

        # Sets the humidity
        environ_msg.humidity = self.humidity

        # Sets the altitude
        environ_msg.altitude = self.altitude

        # Publishes the message
        self.environment_pub.publish(environ_msg)

    def read_proximity(self, timer) -> None:
        # Creates the proximity message
        proximity_msg = Int16()

        # Sets the proximity value
        proximity_msg.data = self.prox

        # Publishes the message
        self.prox_pub.publish(proximity_msg)

    # Sending an float to the arduino
    # Message format []
    def send_velocity(self, values):
        byteList = []

        # Converts the values to bytes
        for value in values:
            byteList += list(struct.pack('f', value))
        # fails to send last byte over I2C, hence this needs to be added
        byteList.append(0)

        # Writes the values to the i2c
        self.bus.write_i2c_block_data(
            self.arduino, byteList[0], byteList[1:12])

        self.linear_x_velo = values[0]

        self.linear = values[1]

        self.angular_z_velo = values[2]

    # Position controller
    def move_to_point(self,msg):
        current_x = self.position["x"]
        current_y = self.position["y"]
        theta = self.position["orientation"]

        # rospy.loginfo("X: {x} Y: {y}".format(x=current_x, y=current_y))
        print("Error: {error}".format(error=math.sqrt((msg.x - current_x)**2 + (msg.y - current_y)**2)))
        if math.sqrt(math.pow((msg.x - current_x),2) + math.pow((msg.y - current_y),2)) < .05:
            self.send_velocity([0,0,0])
        else:

            # Gets the difference between the current position and desired position
            delta_x = msg.x - current_x
            delta_y = msg.y - current_y
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
            omega = self.omega_max * np.arctan2(-b*x_velo + a*y_velo, v) / (np.pi/2)
            
            self.send_velocity([v,0,omega])

    def shutdown_callback(self,msg):
        if msg.data == "shutdown":
            call("sudo shutdown 0", shell=True)
        elif msg.data == "restart":
            call("sudo shutdown -r 0", shell=True)
        elif msg.data == "restart_ros":
            call("kill {process_id} & source ~/.bashrc".format(process_id=os.getpid()),shell=True)

    def __init__(self):

        rospy.init_node("robot_controller", anonymous=True)

        # Arduino Device Address
        self.arduino = 0x08

        # Init smbus
        self.bus = smbus.SMBus(1)

        # Init the i2c bus
        self.light_sensor = True
        self.environment_sensor = True
        self.imu_sensor = False
        self.proximity_sensor = True
        self.global_pos = False
        self.i2c = board.I2C()
        self.name = rospy.get_namespace()

        self.temp = 0
        self.pressure = 0
        self.humidity = 0
        self.altitude = 0
        self.rgbw = None
        self.gesture = 0
        self.prox = 0

        with open("/home/pi/heroswarmv2/ROS1/ros_ws/src/robot_controller/src/robots.json") as file:
            robot_dictionary = json.load(file)
            for key in robot_dictionary:
                if robot_dictionary[key] == self.name:
                    self.id = int(key)
                    
        self.position = {
            "x":0,
            "y":0,
            "orientation":0
        }
        self.linear_x_velo = None
        self.linear_y_velo = None
        self.angular_z_velo = None
        self.last_call = {"time":None}
        self.v_max = 0.1
        self.omega_max = 1.0

        # Creates subscribers for positions topics 
        if self.global_pos:
            self.pos_sub_global = rospy.Subscriber("/positions", Robot_Pos, self.get_pos_global)
        else:
            self.pos_sub_namespace = rospy.Subscriber("position", Odometry, self.get_pos)

        # Creates the twist publisher
        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.read_twist)

        # Creates the velocity lock for auto stop and velocity control
        self.velo_lock = threading.Lock()

        # Creates the auto-stop thread
        self.stop_thread = threading.Thread(target=self.auto_stop,args=(),daemon=True)
        self.stop_thread.start()

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)        
        self.odom_pub_timer = rospy.Timer(rospy.Duration(1/10),self.pub_odom)

        # Creates position control topic
        self.position_sub = rospy.Subscriber("to_point",Point,self.move_to_point)

        # Creates shutdown hook
        self.shutdown_sub = rospy.Subscriber("shutdown", String, self.shutdown_callback)

        self.light = APDS9960(self.i2c)
        self.light.enable_proximity = True
        self.light.enable_gesture = True
        self.light.enable_color = True

        
        self.magnetometer = adafruit_lis3mdl.LIS3MDL(self.i2c)
        # Creates the i2c interface for the bmp sensor
        self.bmp = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c)

        # Creates the i2c interface for the humidity sensor
        self.humidity_sensor = adafruit_sht31d.SHT31D(self.i2c)

        self.IMU = LSM6DS33(self.i2c)

        self.sensor_lock = threading.Lock()
        self.sensor_read_thread = threading.Thread(target=self.read_sensors,args=(),daemon=True)
        self.sensor_read_thread.start()

         # Creates a publisher for the magnetometer, bmp and humidity sensor
        if self.environment_sensor:
            self.environment_pub = rospy.Publisher("environment", Environment, queue_size=1)
            self.environment_timer = rospy.Timer(rospy.Duration(5),self.read_environment)

        # Creates a publisher for imu data
        if self.imu_sensor:
            self.imu_pub = rospy.Publisher("imu", Imu, queue_size=1)
            self.imu_timer = rospy.Timer(rospy.Duration(5),self.read_imu)

        # Creates a publisher for the light sensor
        if self.light_sensor:
            self.light_pub = rospy.Publisher('light', Light, queue_size=1)
            self.light_timer = rospy.Timer(rospy.Duration(5),self.read_light)

        # Creates a publisher for a proximity sensor
        if self.proximity_sensor:
            self.prox_pub = rospy.Publisher("proximity",Int16, queue_size=1)
            self.environment_timer = rospy.Timer(rospy.Duration(5),self.read_proximity)

        print("Ready")


if __name__ == '__main__':
    controller = Controller()
    while not rospy.is_shutdown():
        continue
