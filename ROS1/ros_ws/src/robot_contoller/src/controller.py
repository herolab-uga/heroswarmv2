#! /usr/bin/python3

import math
import struct

import adafruit_bmp280
import adafruit_lis3mdl
import adafruit_sht31d
import board
import numpy as np
import rospy
import smbus
from adafruit_apds9960.apds9960 import APDS9960
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
from geometry_msgs.msg import Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from robot_msgs.msg import Environment, Light, Robot_Pos
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16


class Controller:

    def __init__(self):

        rospy.init_node("robot_controller",anonymous=True)

        # Arduino Device Address
        self.arduino = 0x08

        # Init smbus
        self.bus = smbus.SMBus(1)

        # Init the i2c bus
        self.light = False
        self.environment = False
        self.imu = False
        self.proximity = False
        self.i2c = board.I2C()
        self.id = 18
        self.x = None
        self.z = None
        self.heading = None
        self.linear_x_velo = None
        self.linear_y_velo = None
        self.angular_z_velo = None
        
        self.pos_sub = rospy.Subscriber("Positions",Robot_Pos, self.get_pos)
        self.bmp = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c)
        self.humidity = adafruit_sht31d.SHT31D(self.i2c)
        self.twist_sub = rospy.Subscriber("/cmd_vel",Twist, self.read_twist,10)
        self.odom_pub = rospy.Publisher("odom",Odometry,queue_size=5)
        self.odom_timer = rospy.Timer(rospy.Duration(1/15),self.pub_odom)

        if self.imu:
            self.IMU = LSM6DS33(self.i2c)
            self.imu_pub = rospy.Publisher("imu",Imu,queue_size=5)
            self.imu_timer = rospy.Timer(rospy.Duration(1/30),self.read_imu)

        if self.light:
            self.light = APDS9960(self.i2c)
            self.light.enable_proximity = True
            self.light.enable_gesture = True
            self.light.enable_color = True

            self.light_pub = rospy.Publisher('light',Light,queue_size=5)
            self.light_timer = rospy.Timer(rospy.Duration(1/20),self.read_light)
            
        if self.environment:
            self.magnetometer = adafruit_lis3mdl.LIS3MDL(self.i2c)
            self.bmp = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c)
            self.humidity = adafruit_sht31d.SHT31D(self.i2c)
            self.environment_pub = rospy.Publisher("environment",Environment,queue_size=5)
            self.environment_timer = rospy.Timer(rospy.Duration(1/20),self.read_environment)
        
        if self.proximity:
            self.prox_pub = rospy.Publisher(Int16,"proximity",queue_size=5)
            self.proximity_timer = rospy.Timer(rospy.Duration(1/30),self.read_proximity)

        print("Ready")

    def rpy_from_quaternion(self,quaternion):
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

    def get_pos(self,msg):
        for robot in msg.robot_pos:
            if robot.child_frame_id == str(self.id):
                self.x = robot.pose.pose.position.x
                self.z = robot.pose.pose.position.z
                self.heading = self.rpy_from_quaternion(robot.pose.pose.orientation)
                break
        rospy.loginfo("X: {x} Z: {y} Theta: {theta}".format(x=self.x,z=self.z,theta=self.heading))

    def pub_odom(self,timer,event=None):
        # Creates the odom message
        odom_msg = Odometry()

        data = self.bus.read_i2c_block_data(self.arduino, 0)

        odom_data = [0.0,0.0,0.0,0.0,0.0]

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

        odom_msg.pose.pose.orientation

        self.odom_pub.publish(odom_msg)
        
    def read_twist(self,msg, event=None) -> None:
        # Reads ths twist message x linear velocity
        x_velo = msg.linear.x
        
        # Reads the twist message y linear velocity
        y_velo = msg.linear.y

        # Reads the twist message z angular velocity
        z_angular = msg.angular.z

        self.linear_x_velo = x_velo

        self.linear=y_velo = y_velo

        self.angular_z_velo = z_angular

        # Logs the data
        rospy.loginfo("X Linear: {x} Y Linear: {y} Z Angular: {z}".format(x=x_velo,y=y_velo,z=z_angular))
        
        if x_velo == self.linear_x_velo and y_velo == self.linear_y_velo and z_angular == self.angular_z_velo:
            return

        # Sends the velocity information to the feather board
        self.send_velocity([x_velo,y_velo,z_angular])

    def read_imu(self,event=None) -> None:
        
        # Creates the IMU message
        imu_msg = Imu()
        
        # Read the sensor
        acc_x, acc_y, acc_z = self.IMU.acceleration
        gyro_x, gyro_y, gyro_z = self.IMU.gyro


        # Sets the orientation parameters
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0

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
    def mean(self,values):
        return sum(values) / len(values)


    def normalized_rms(self,values):
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

    
    def read_light(self,event=None) -> None:

        # Creates the light message
        light_msg = Light()

        # Sets the current rgbw value array
        light_msg.rgbw = set(self.light.color_data)

        # Sets the gesture type
        light_msg.gesture = self.light.gesture()

        # Publishes the message
        self.light_pub.publish(light_msg)

    def read_environment(self,event=None) -> None:
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

    def read_proximity(self,timer) -> None:
        # Creates the proximity message
        proximity_msg = Int16()
        
        # Sets the proximity value
        proximity_msg.data = self.light.proximity

        # Publishes the message
        self.prox_pub.publish(proximity_msg)

        

    # Sending an float to the arduino
    # Message format []
    def send_velocity(self,values):
        byteList = []

        # Converts the values to bytes 
        for value in values:
            byteList += list(struct.pack('f', value))
        byteList.append(0)  # fails to send last byte over I2C, hence this needs to be added 

        # Writes the values to the i2c
        self.bus.write_i2c_block_data(self.arduino, byteList[0], byteList[1:12])
        print(values)


if __name__ == '__main__':
    controller = Controller()
    print("Running")
    while not rospy.is_shutdown():
        continue

