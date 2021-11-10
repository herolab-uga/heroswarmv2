#! /usr/bin/python3

import math
import struct
import threading
import time

import adafruit_bmp280
import adafruit_lis3mdl
import adafruit_sht31d
import board
import numpy as np
import rospy
import smbus
from adafruit_apds9960.apds9960 import APDS9960
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point
from nav_msgs.msg import Odometry
from robot_msgs.msg import Environment, Light, Robot_Pos
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16


class Controller:

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

    def get_pos_global(self,msg):
            for robot in msg.robot_pos:
                if robot.child_frame_id == str(self.id):
                    self.x = robot.pose.pose.position.x
                    self.z = robot.pose.pose.position.z
                    self.heading = self.rpy_from_quaternion(robot.pose.pose.orientation)
                    break
            rospy.loginfo("X: {x} Z: {z} Theta: {theta}".format(x=self.x,z=self.z,theta=self.heading))

    def get_pos(self,msg):
        self.x = msg.pose.pose.position.x
        self.z = msg.pose.pose.position.z
        self.heading = self.rpy_from_quaternion(msg.pose.pose.orientation)
        
        rospy.loginfo("X: {x} Z: {z} Theta: {theta}".format(x=self.x,z=self.z,theta=self.heading))

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
        odom_msg.twist.twist.linear.x = odom_data[3] * math.cos(odom_data[2])
        odom_msg.twist.twist.linear.y = odom_data[3] * math.sin(odom_data[2])
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = odom_data[4]

        odom_msg.pose.pose.position.x = odom_data[0]
        odom_msg.pose.pose.position.y = odom_data[1]
        odom_msg.pose.pose.position.z = 0.0

        self.odom_pub.publish(odom_msg)

    def read_twist(self, msg, event=None) -> None:
        x_velo = 0
        z_angular = 0            
        # Reads ths twist message x linear velocity
        if not msg.linear.x == 0:
            direction_lin = msg.linear.x / abs(msg.linear.x)
            x_velo = direction_lin * (msg.linear.x if abs(msg.linear.x) <= .10 else .10)
        else:
            x_velo = 0

        # Reads the twist message y linear velocity
        y_velo = msg.linear.y

        # Reads the twist message z angular velocity
        if not msg.angular.z == 0:
            direction_ang = msg.angular.z / abs(msg.angular.z)
            z_angular = direction_ang * (msg.angular.z if abs(msg.angular.z) <= .30 else .30)
        else:
            z_angular = 0

        if not (x_velo == self.linear_x_velo and y_velo == self.linear_y_velo and z_angular == self.angular_z_velo):
            # Logs the data
            rospy.loginfo("X Linear: {x} Y Linear: {y} Z Angular: {z}".format(
                x=x_velo, y=y_velo, z=z_angular))
            # Sends the velocity information to the feather board

            self.last_call["time"] = time.time()
            with self.velo_lock:
                self.send_velocity([x_velo, y_velo, z_angular])
                
            self.linear_x_velo = x_velo
            self.linear_y_velo = y_velo
            self.angular_z_velo = z_angular

    def auto_stop(self):
        while True:
                if self.last_call["time"] == None:
                    continue
                elif time.time() - self.last_call["time"] > .300:
                        with self.velo_lock:
                            self.send_velocity([0, 0, 0])
                        self.linear_x_velo = 0
                        self.linear_y_velo = 0
                        self.angular_z_velo = 0
        
            


    def read_imu(self, event=None) -> None:

        # Creates the IMU message
        imu_msg = Imu()

        # Read the sensor
        acc_x, acc_y, acc_z = self.IMU.acceleration
        gyro_x, gyro_y, gyro_z = self.IMU.gyro

        # Sets the self.heading parameters
        imu_msg.self.heading.x = 0.0
        imu_msg.self.heading.y = 0.0
        imu_msg.self.heading.z = 0.0

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

    def read_light(self, event=None) -> None:

        # Creates the light message
        light_msg = Light()

        # Sets the current rgbw value array
        light_msg.rgbw = set(self.light.color_data)

        # Sets the gesture type
        light_msg.gesture = self.light.gesture()

        # Publishes the message
        self.light_pub.publish(light_msg)

    def read_environment(self, event=None) -> None:
        # Creates the environment message
        environ_msg = Environment()

        # Sets the temperature
        environ_msg.temp = self.bmp.temperature

        # Sets the pressure
        environ_msg.pressure = self.bmp.pressure

        # Sets the humidity
        environ_msg.humidity = self.humidity.relative_humidity

        # Sets the altitude
        environ_msg.altitude = self.bmp.altitude

        # Publishes the message
        self.environment_pub.publish(environ_msg)

    def read_proximity(self, timer) -> None:
        # Creates the proximity message
        proximity_msg = Int16()

        # Sets the proximity value
        proximity_msg.data = self.light.proximity

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

        print(values)

    def move_to_point(self, msg):

        set_linear = 0
        set_angular = 0

        self.get_logger().info("X: {x} Y: {y}".format(x=self.x, y=self.y))
        if not (np.sqrt((msg.x - self.x)**2 + (msg.x - self.y)**2) < .05):

            delta_x = msg.x - self.x
            delta_y = msg.y - self.y
            theta = self.rpy_from_quaternion(self.heading)[0]
            v = self.v_max*(delta_x*np.cos(theta) + delta_y*np.sin(theta))
            omega = self.omega_max * \
                (2*np.arctan2(-np.sin(theta)*delta_x +
                    np.cos(theta)*delta_y, v))/np.pi
            set_linear = v
            set_angular = omega

        self.send_velocity([set_linear, 0, set_angular])

    def __init__(self):

        rospy.init_node("robot_controller", anonymous=True)

        # Arduino Device Address
        self.arduino = 0x08

        # Init smbus
        self.bus = smbus.SMBus(1)

        # Init the i2c bus
        self.light = False
        self.environment = False
        self.imu = False
        self.proximity = False
        self.global_pos = False
        self.i2c = board.I2C()
        self.id = 18
        self.x = None
        self.z = None
        self.heading = None
        self.linear_x_velo = None
        self.linear_y_velo = None
        self.angular_z_velo = None
        self.last_call = {"time":None}

        if self.global_pos:
            self.pos_sub_global = rospy.Subscriber("/positions", Robot_Pos, self.get_pos_global)
        else:
            self.pos_sub_namespace = rospy.Subscriber("position", Odometry, self.get_pos)

        self.bmp = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c)
        self.humidity = adafruit_sht31d.SHT31D(self.i2c)
        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.read_twist)
        self.velo_lock = threading.Lock()
        self.stop_thread = threading.Thread(target=self.auto_stop,args=(),daemon=True)
        self.stop_thread.start()

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=5)
        self.odom_timer = rospy.Timer(rospy.Duration(1/15), self.pub_odom)
        self.point_sub = rospy. Subscriber("to_point", Point, self.move_to_point)

        if self.imu:
            self.IMU = LSM6DS33(self.i2c)
            self.imu_pub = rospy.Publisher("imu", Imu, queue_size=5)
            self.imu_timer = rospy.Timer(rospy.Duration(1/30), self.read_imu)

        if self.light:
            self.light = APDS9960(self.i2c)
            self.light.enable_proximity = True
            self.light.enable_gesture = True
            self.light.enable_color = True

            self.light_pub = rospy.Publisher('light', Light, queue_size=5)
            self.light_timer = rospy.Timer(
                rospy.Duration(1/20), self.read_light)

        if self.environment:
            self.magnetometer = adafruit_lis3mdl.LIS3MDL(self.i2c)
            self.bmp = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c)
            self.humidity = adafruit_sht31d.SHT31D(self.i2c)
            self.environment_pub = rospy.Publisher(
                "environment", Environment, queue_size=5)
            self.environment_timer = rospy.Timer(
                rospy.Duration(1/20), self.read_environment)

        if self.proximity:
            self.prox_pub = rospy.Publisher(Int16, "proximity", queue_size=5)
            self.proximity_timer = rospy.Timer(
                rospy.Duration(1/30), self.read_proximity)

        print("Ready")


if __name__ == '__main__':
    controller = Controller()
    while not rospy.is_shutdown():
        continue
