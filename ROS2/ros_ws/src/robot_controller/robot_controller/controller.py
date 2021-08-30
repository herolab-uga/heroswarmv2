#! /usr/bin/python3

import math
import struct

import adafruit_bmp280
import adafruit_lis3mdl
import adafruit_sht31d
import board
import rclpy
import smbus
from tf.transformations import quaternion_from_euler
import geometry_msgs
from adafruit_apds9960.apds9960 import APDS9960
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
from geometry_msgs.msg import Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_msgs.msg import Enviornment, Light
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16


class Controller(Node):

    def __init__(self):

        # Creates the control node
        super().__init__("robot_controller")

        # Arduino Device Address
        self.arduino = 0x08

        # Init smbus
        self.bus = smbus.SMBus(1)

        # Init the i2c bus
        self.light = True
        self.enviornment = True
        self.imu = True
        self.proximity = True
        self.i2c = board.I2C()
        
        self.bmp = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c)
        self.humidity = adafruit_sht31d.SHT31D(self.i2c)
        self.twist_sub = self.create_subscription(Twist,"cmd_vel", self.read_twist,10)
        
        #self.mic_pub = self.create_publisher(Int16,"mic",2)

        if self.imu:
            self.IMU = LSM6DS33(self.i2c)
            self.odom_pub = self.create_publisher(Odometry, "odom",5)
            self.odom_tmr = self.create_timer(.015, self.pub_odom)
            self.imu_pub = self.create_publisher(Imu,"imu",5)
            self.imu_tmr = self.create_timer(1.0, self.read_imu)

        if self.light:
            self.light = APDS9960(self.i2c)
            self.light.enable_proximity = True
            self.light.enable_gesture = True
            self.light.enable_color = True
            self.prox_pub = self.create_publisher(Int16,"proximity",5)
            self.prox_tmr = self.create_timer(.030, self.read_proximity)
            self.light_pub = self.create_publisher(Light,'light',5)
            self.light_tmr = self.create_timer(1.0, self.read_light)
        
        if self.enviornment:
            self.magnetometer = adafruit_lis3mdl.LIS3MDL(self.i2c)
            self.bmp = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c)
            self.humidity = adafruit_sht31d.SHT31D(self.i2c)
            self.enviornment_pub = self.create_publisher(Enviornment,"enviornment",5)
            self.enviorn_tmr = self.create_timer(1.0, self.read_enviornment)
        
        if self.proximity:
            self.proximity_pub = self.create_publisher(Int16,"proximity",5)
            self.proximity_tmr = self.create_timer(1.0, self.read_proximity)

        self.linear_x_velo = None
        self.linear_y_velo = None
        self.angular_z_velo = None
        print("Ready")


    def pub_odom(self):
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

        odom_msg.pose.pose.orientation = quaternion_from_euler(0, 0, odom_data[2])

        self.odom_pub.publish(odom_msg)
        
    def read_twist(self,msg) -> None:
        # Reads ths twist message x linear velocity
        x_velo = msg.linear.x
        
        # Reads the twist message y linear velocity
        y_velo = msg.linear.y

        # Reads the twist message z angular velocity
        z_angular = msg.angular.z

        self.linearx_velo = x_velo

        self.linear=y_velo = y_velo

        self.angular_z_velo = z_angular

        # Logs the data
        self.get_logger().info("X Linear: {x} Y Linear: {y} Z Angular: {z}".format(x=x_velo,y=y_velo,z=z_angular))
        
        if x_velo is self.linearx_velo and y_velo is self.linear_y_velo is z_angular is self.angular_z_velo:
            return

        # Sends the velocity information to the feather board
        self.send_velocity([x_velo,y_velo,z_angular])

    def read_imu(self) -> None:
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

    
    def read_light(self) -> None:
        # Creates the light message
        light_msg = Light()

        # Sets the current rgbw value array
        light_msg.rgbw = set(self.light.color_data)

        # Sets the gesture type
        light_msg.gesture = self.light.gesture()

        # Publishes the message
        self.light_pub.publish(light_msg)

    def read_enviornment(self) -> None:
        # Creates the enviornment message
        enviorn_msg = Enviornment()

        # Sets the temperature
        enviorn_msg.temp = self.bmp.temperature

        # Sets the pressure 
        enviorn_msg.pressure = self.bmp.pressure

        # Sets the humidity
        enviorn_msg.humidity = self.humidity.relative_humidity

        # Sets the altitude
        enviorn_msg.altitude = self.bmp.altitude

        # Publishes the message
        self.enviornment_pub.publish(enviorn_msg)

    def read_proximity(self) -> None:
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


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)

