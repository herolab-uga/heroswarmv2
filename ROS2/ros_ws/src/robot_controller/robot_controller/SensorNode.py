#! /usr/bin/python3

from adafruit_apds9960.apds9960 import APDS9960
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
import adafruit_bmp280
import adafruit_lis3mdl
import adafruit_sht31d
import board

from robot_msgs.msg import Environment, Light, Robot_Pos
from sensor_msgs.msg import Imu

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32

import struct
import time
import numpy as np

ODOM_COVARIANCE_MATRIX = [1e-2, 0, 0, 0, 0, 0,
                          0, 1e-2, 0, 0, 0, 0,
                          0, 0, 1e-2, 0, 0, 0,
                          0, 0, 0, 1e-2, 0, 0,
                          0, 0, 0, 0, 1e-2, 0,
                          0, 0, 0, 0, 0, 1e-2]

IMU_COVARIANCE_MATRIX = [1e-2, 0, 0, 0, 1e-2, 0, 0, 0, 1e-2]


class SensorNode(Node):

    def pub_imu(self, freq) -> None:
        # Creates the IMU message
        imu_msg = Imu()
        # Read the sensor data
        acc_x, acc_y, acc_z = self.IMU.acceleration
        gyro_x, gyro_y, gyro_z = self.IMU.gyro

        imu_msg.header.stamp = self.Time.now()
        imu_msg.header.frame_id = "base_footprint"

        # Sets the angular velocity parameters
        imu_msg.angular_velocity.x = gyro_x - self.x_gyro_avg
        imu_msg.angular_velocity.y = gyro_y - self.y_gyro_avg
        imu_msg.angular_velocity.z = gyro_z - self.z_gyro_avg

        # Sets the linear acceleration parameters
        imu_msg.linear_acceleration.x = acc_x - self.x_avg
        imu_msg.linear_acceleration.y = acc_y - self.y_avg
        imu_msg.linear_acceleration.z = acc_z - self.z_avg

        imu_msg.orientation_covariance = IMU_COVARIANCE_MATRIX
        imu_msg.angular_velocity_covariance = IMU_COVARIANCE_MATRIX
        imu_msg.linear_acceleration_covariance = IMU_COVARIANCE_MATRIX

        # Publishes the message
        self.imu_pub.publish(imu_msg)

# move sensor data gathering to the feather sense
# change i2c baud to 400 khz
    def init_sensors(self):
        try:
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

            self.IMU = LSM6DS33(self.i2c)
            self.x_avg = 0.0
            self.y_avg = 0.0
            self.z_avg = 0.0

            self.x_gyro_avg = 0.0
            self.y_gyro_avg = 0.0
            self.z_gyro_avg = 0.0

            for i in range(0, 1000):
                if i < 499:
                    self.x_gyro_avg += self.IMU.gyro[0]
                    self.y_gyro_avg += self.IMU.gyro[1]
                    self.z_gyro_avg += self.IMU.gyro[2]

                    self.x_avg += self.IMU.acceleration[0]
                    # self.y_avg += self.IMU.acceleration[1]
                    self.z_avg += self.IMU.acceleration[2]

            self.x_avg = self.x_avg / 500
            self.y_avg = self.y_avg / 500
            self.z_avg = self.z_avg / 500

            self.x_gyro_avg = self.x_gyro_avg / 500
            self.y_gyro_avg = self.y_gyro_avg / 500
            self.z_gyro_avg = self.z_gyro_avg / 500

            self.get_logger.info("Done calibrating")

        except ValueError:
            time.sleep(.1)
            self.init_sensors()

    def read_sensors(self, sensor_data):
        rate = self.create_rate(5)

        while not self.ok():
            try:
                sensor_data["temp"] = self.bmp.temperature
                sensor_data["pressure"] = self.bmp.pressure
                sensor_data["humidity"] = self.humidity_sensor.relative_humidity[0]
                sensor_data["altitude"] = self.bmp.altitude
                sensor_data["rgbw"] = self.light.color_data
                sensor_data["gesture"] = self.light.gesture()
                sensor_data["prox"] = self.light.proximity
                # queue.put(sensor_data)
            except:
                print("Could not read sensor")

    def pub_light(self, timer) -> None:
        # Creates the light message
        light_msg = Light()

        # Sets the current rgbw value array
        light_msg.rgbw = self.sensor_data["rgbw"]

        # Sets the gesture type
        light_msg.gesture = self.sensor_data["gesture"]

        # Publishes the message
        self.light_pub.publish(light_msg)

    def pub_environment(self, timer) -> None:
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

    def pub_proximity(self, timer, ) -> None:
        # Creates the proximity message
        proximity_msg = Int16()

        # Sets the proximity value
        proximity_msg.data = self.sensor_data["prox"]

        # Publishes the message
        self.prox_pub.publish(proximity_msg)

    def pub_mic(self, timer):
        # Creates the mic message
        mic_msg = Float32()

        # Sets the mic value
        mic_msg.data = self.sensor_data["mic"]

        # Publishes the message
        self.mic_pub.publish(mic_msg)

    def read_arduino_data(self, timer, event=None):

        num_val = 7

        # Creates the odom message
        odom_msg = Odometry()

        data = bytearray(num_val * 4)

        try:
            while not self.i2c.try_lock():
                pass
            self.i2c.readfrom_into(self.arduino, data)
        # Get odom data from arduino
        except Exception as e:
            self.get_logger.info(e)  # need to fix this
        finally:
            self.i2c.unlock()

        data = list(struct.unpack("f"*num_val, data[:]))

        self.sensor_data["mic"] = data[6]

        # Updates Battery Level
        self.sensor_data["battery"] = data[5]

        # Adds Twist data
        odom_msg.header.stamp = self.now()
        odom_msg.header.frame_id = "base_footprint"
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

        odom_msg.pose.covariance = ODOM_COVARIANCE_MATRIX
        odom_msg.twist.covariance = ODOM_COVARIANCE_MATRIX

        self.odom_pub.publish(odom_msg)

    def read_sensors(self):
        # Reads the sensors and returns the data
        self.imu = LSM6DS33(self.i2c)
        self.magnetometer = adafruit_lis3mdl.LIS3MDL(self.i2c)
        self.bmp = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c)
        self.humidity = adafruit_sht31d.SHT31D(self.i2c)
        self.light = APDS9960(self.i2c)



    def __init__(self) -> None:
        super().__init__("sensor_node",anonymous=True)
        self.i2c = board.I2C()
        self.name = self.get_namespace()

        # Creates the odom publisher
        self.odom_pub = self.Publisher(Odometry,"odom", queue_size=1)

                # Creates the battery publisher
        self.battery_pub = self.Publisher(Float32,"battery", queue_size=1)

        # Creates timer to read data from arduino
        self.read_arduino_data_timer = self.create_timer(.2, self.read_arduino_data)

        ###_________________Enables Sensor Data Publishers________________###
        # all of this is probably no right. Need to look into getting launch parameter in ros2

        # Creates a publisher for the magnetometer, bmp and humidity sensor
        if self.get_param(self.name + "controller/environment") == True or self.get_param(self.name + "controller/all_sensors") == True:
            self.environment_pub = self.Publisher(
                Environment,"environment", queue_size=1)
            self.environment_timer = self.create_timer(.1,self.pub_environment)

        # Creates a publisher for imu data
        if self.get_param(self.name + "controller/imu") == True or self.get_param(self.name + "controller/all_sensors") == True:
            self.imu_pub = self.Publisher(Imu,"imu/data_raw", queue_size=1)
            self.imu_timer = self.create_timer(.1,self.pub_imu) # not working

        # Creates a publisher for the light sensor
        if self.get_param(self.name + "controller/light") == True or self.get_param(self.name + "controller/all_sensors") == True:
            self.light_pub = self.Publisher(Light,'light', queue_size=1)
            self.light_timer = self.create_timer(
                .2, self.pub_light)

        # Creates a publisher for a proximity sensor
        if self.get_param(self.name + "controller/proximity") == True or self.get_param(self.name + "controller/all_sensors") == True:
            self.prox_pub = self.Publisher(Int16,"proximity", queue_size=1)
            self.environment_timer = self.create_timer(
                .2, self.pub_proximity)

        if self.get_param(self.name + "controller/mic") == True or self.get_param(self.name + "controller/all_sensors") == True:
            self.mic_pub = self.Publisher(Float32,"mic", queue_size=1)
            self.mic_timer = self.create_timer(
                .2, self.pub_mic)