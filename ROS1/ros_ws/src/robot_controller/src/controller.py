#!/usr/bin/python3

from concurrent.futures import thread
import math
from pickletools import float8
import struct
import threading
import time
import json
import os
import threading
import multiprocessing as mp

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
from robot_msgs.msg import Environment, Light, Robot_Pos, StringList
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16, String, Float32, Int16MultiArray
from subprocess import call


class Controller:

    def __del__(self):
        self.send_values([0, 0, 0])

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

    def quaternion_from_rpy(self, roll, pitch, yaw):
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

    def read_arduino_data(self, timer, event=None):

        num_val = 7

        # Creates the odom message
        odom_msg = Odometry()

        data = bytearray(num_val * 4)

        try:
            while not self.i2c.try_lock():
                pass
            self.i2c.readfrom_into(self.arduino,data)
        # Get odom data from arduino
        except Exception as e:
            print(e) #need to fix this
        finally:
            self.i2c.unlock()

        data = list(struct.unpack("f"*num_val, data[:]))

        self.sensor_data["mic"] = data[6]

        # Updates Battery Level
        self.sensor_data["battery"] = data[5]

        # Adds Twist data
        theta = np.deg2rad(data[2])
        odom_msg.twist.twist.linear.x = data[3]
        odom_msg.twist.twist.linear.y = data[4]
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = data[4]

        odom_msg.pose.pose.position.x = data[0]
        odom_msg.pose.pose.position.y = data[1]
        odom_msg.pose.pose.position.z = 0.0

        quaternion = self.quaternion_from_rpy(0, 0, theta)

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
            x_velo = direction_lin * \
                (abs(msg.linear.x) if abs(msg.linear.x) <= .10 else .10)
        else:
            x_velo = 0

        # Reads the twist message y linear velocity
        y_velo = 0

        # Reads the twist message z angular velocity
        if not msg.angular.z == 0:
            direction_ang = msg.angular.z / abs(msg.angular.z)
            z_angular = direction_ang * \
                (abs(msg.angular.z) if abs(msg.angular.z) <= 1.85 else 1.85)
        else:
            z_angular = 0

        if not (x_velo == self.linear_x_velo and y_velo == self.linear_y_velo and z_angular == self.angular_z_velo):
            # Logs the data
            rospy.loginfo("X Linear: {x} Y Linear: {y} Z Angular: {z}".format(
                x=x_velo, y=y_velo, z=z_angular))
            # Sends the velocity information to the feather board
            with self.velo_lock:
                self.last_call["time"] = time.time()
            self.send_values([x_velo, y_velo, z_angular])
            self.linear_x_velo = x_velo
            self.linear_y_velo = y_velo
            self.angular_z_velo = z_angular

    def auto_stop(self):
        while True:
            if self.last_call["time"] == None:
                continue
            elif time.time() - self.last_call["time"] > 0.12:
                if not (self.linear_x_velo == 0 and self.linear_y_velo == 0 and self.angular_z_velo == 0):
                    self.send_values([0.0, 0.0, 0.0])
                    self.linear_x_velo = 0
                    self.linear_y_velo = 0
                    self.angular_z_velo = 0
            time.sleep(.1)

    def read_imu(self, freq) -> None:
        # Creates the IMU message
        imu_msg = Imu()
        # Read the sensor
        acc_x, acc_y, acc_z = self.IMU.acceleration
        gyro_x, gyro_y, gyro_z = self.IMU.gyro

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

    def read_sensors(self, sensor_data):
        rate = rospy.Rate(5)
        
        # Creates sensor objects
        self.light = APDS9960(self.i2c)
        self.light.enable_proximity = True
        self.light.enable_gesture = False
        self.light.enable_color = True

        self.magnetometer = adafruit_lis3mdl.LIS3MDL(self.i2c)
        # Creates the i2c interface for the bmp sensor
        self.bmp = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c)

        # Creates the i2c interface for the humidity sensor
        self.humidity_sensor = adafruit_sht31d.SHT31D(self.i2c)
        self.humidity_sensor.mode = adafruit_sht31d.MODE_PERIODIC
        self.humidity_sensor.frequency = adafruit_sht31d.FREQUENCY_2

        while not rospy.is_shutdown():
            try:
                sensor_data["temp"] = self.bmp.temperature
                sensor_data["pressure"] = self.bmp.pressure
                sensor_data["humidity"] = self.humidity_sensor.relative_humidity[0]
                sensor_data["altitude"] = self.bmp.altitude
                sensor_data["rgbw"] = self.light.color_data
                # sensor_data["gesture"] = self.light.gesture()
                sensor_data["prox"] = self.light.proximity
            except:
                print("Could not read sensor")
            time.sleep(.25)

    def read_light(self, timer) -> None:
        # Creates the light message
        light_msg = Light()

        # Sets the current rgbw value array
        light_msg.rgbw = self.sensor_data["rgbw"]

        # Sets the gesture type
        light_msg.gesture = self.sensor_data["gesture"]

        # Publishes the message
        self.light_pub.publish(light_msg)

    def read_environment(self, timer) -> None:
        # Creates the environment message
        environ_msg = Environment()

        # Sets the temperature
        environ_msg.temp = self.sensor_data["temp"]

        # Sets the pressure
        environ_msg.pressure = self.sensor_data["pressure"]

        # Sets the humidity
        environ_msg.humidity = self.sensor_data["humidity"]

        # Sets the altitude
        environ_msg.altitude = self.sensor_data["altitude"]

        # Publishes the message
        self.environment_pub.publish(environ_msg)

    def read_proximity(self, timer, ) -> None:
        # Creates the proximity message
        proximity_msg = Int16()

        # Sets the proximity value
        proximity_msg.data = self.sensor_data["prox"]

        # Publishes the message
        self.prox_pub.publish(proximity_msg)

    def read_mic(self,timer):
        # Creates the mic message
        mic_msg = Float32()

        # Sets the mic value
        mic_msg.data = self.sensor_data["mic"]

        # Publishes the message
        self.mic_pub.publish(mic_msg)

     # Sending an float to the arduino
    # Message format [msgid , args]
    def send_values(self, values=None, opcode=0):
        # Work around for demo remove later
        if self.name == "/swarmpaddy1/" or self.name == "/swarmstarburst1/" or self.name == "/swarmstarapril1/" or self.name == "/swarmcoral1/":
            # Converts the values to bytes
            byteList = struct.pack("f", opcode) + \
                struct.pack('fff', *values)
        else:
            byteList = struct.pack('fff', *values)
        # # fails to send last byte over I2C, hence this needs to be added
        # byteList.append(0)
        try:
            while not self.i2c.try_lock():
                pass
            # Writes the values to the i2c
            self.i2c.writeto(self.arduino, byteList[1:16], stop=False)

            if opcode == 0:
                self.linear_x_velo = values[0]

                self.linear = values[1]

                self.angular_z_velo = values[2]
        except OSError as e:
            print(e)
            print("Could not send message: {opcode} {data}".format(
                opcode=opcode, data=values))
        finally:
            self.i2c.unlock()


    def move_to_angle(self, angle):
        rate = rospy.Rate(10)
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

        rospy.loginfo("X: {x} Y: {y}".format(x=current_x, y=current_y))
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

    def shutdown_callback(self, msg):
        rospy.loginfo("Shutting Down")
        rospy.signal_shutdown("Rasperry Pi shutting down")
            

    def neopixel_callback(self, msg):
        self.send_values(msg.data, 1.0)

    def pub_battery(self, timer):
        battery_msg = Float32()
        battery_msg.data = self.sensor_data["battery"]
        self.battery_pub.publish(battery_msg)

    def __init__(self):
        # print("Start")
        rospy.init_node("robot_controller", anonymous=True)

        # Arduino Device Address
        self.arduino = 0x08

        self.i2c = board.I2C()
        self.name = rospy.get_namespace()

        self.IMU = LSM6DS33(self.i2c)
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

        with open("/home/pi/heroswarmv2/ROS1/ros_ws/src/robot_controller/src/robots.json") as file:
            robot_dictionary = json.load(file)
            for key in robot_dictionary:
                if robot_dictionary[key] == self.name:
                    self.id = int(key)

        self.position = {
            "x": 0,
            "y": 0,
            "orientation": 0
        }

        self.linear_x_velo = None
        self.linear_y_velo = None
        self.angular_z_velo = None
        self.last_call = {"time": None}
        self.v_max = 0.1
        self.omega_max = 1.0

        self.open_chargers = None

        # Creates subscribers for positions topics
        if rospy.get_param(self.name + "controller/global_pos") == True:
            self.pos_sub_global = rospy.Subscriber(
                "/positions", Robot_Pos, self.get_pos_global)
        else:
            self.pos_sub_namespace = rospy.Subscriber(
                "position", Odometry, self.get_pos)

        # Creates the twist publisher
        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.read_twist)

        # Creates the velocity lock for auto stop and velocity control
        self.velo_lock = threading.Lock()

        # # Creates the auto-stop thread
        # self.stop_thread = threading.Thread(target=self.auto_stop, args=())
        # self.stop_thread.start()

        # Creates the battery publisher
        self.battery_pub = rospy.Publisher("battery", Float32, queue_size=1)

        # Creates the odom publisher
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)

        # Creates timer to read data from arduino
        self.read_arduino_data_timer = rospy.Timer(
            rospy.Duration(1/10), self.read_arduino_data)

        # Publish the batterry level on the battery topic with at 5hz
        self.battery_timer = rospy.Timer(rospy.Duration(1/5), self.pub_battery)

        # Creates position control topic
        self.position_sub = rospy.Subscriber(
            "to_point", Point, self.move_to_point)

        # Creates shutdown hook
        self.shutdown_sub = rospy.Subscriber(
            "shutdown", String, self.shutdown_callback)

        # Read sensors
        self.sensor_read_thread = threading.Thread(
            target=self.read_sensors, args=(self.sensor_data,),daemon=True)
        self.sensor_read_thread.start()

        ###_________________Enables Sensor Data Publishers________________###

        # Creates a publisher for the magnetometer, bmp and humidity sensor
        if rospy.get_param(self.name + "controller/environment") == True or rospy.get_param(self.name + "controller/all_sensors") == True:
            self.environment_pub = rospy.Publisher(
                "environment", Environment, queue_size=1)
            self.environment_timer = rospy.Timer(rospy.Duration(1/10),self.read_environment)

        # Creates a publisher for imu data
        if rospy.get_param(self.name + "controller/imu") == True or rospy.get_param(self.name + "controller/all_sensors") == True:
            self.imu_pub = rospy.Publisher("imu", Imu, queue_size=1)
            self.imu_timer = rospy.Timer(rospy.Duration(1/60),self.read_imu) # not working

        # Creates a publisher for the light sensor
        if rospy.get_param(self.name + "controller/light") == True or rospy.get_param(self.name + "controller/all_sensors") == True:
            self.light_pub = rospy.Publisher('light', Light, queue_size=1)
            self.light_timer = rospy.Timer(
                rospy.Duration(1/5), self.read_light)

        # Creates a publisher for a proximity sensor
        if rospy.get_param(self.name + "controller/proximity") == True or rospy.get_param(self.name + "controller/all_sensors") == True:
            self.prox_pub = rospy.Publisher("proximity", Int16, queue_size=1)
            self.environment_timer = rospy.Timer(
                rospy.Duration(1/5), self.read_proximity)

        if rospy.get_param(self.name + "controller/mic") == True or rospy.get_param(self.name + "controller/all_sensors") == True:
            self.mic_pub = rospy.Publisher("mic", Float32, queue_size=1)
            self.mic_timer = rospy.Timer(
                rospy.Duration(1/5), self.read_mic)

        self.neopixel_subscriber = rospy.Subscriber(
            "neopixel", Int16MultiArray, self.neopixel_callback)

        rospy.loginfo("Ready")


if __name__ == '__main__':
    controller = Controller()
    while not rospy.is_shutdown():
        continue
    # call("sudo shutdown -h 0", shell=True)

