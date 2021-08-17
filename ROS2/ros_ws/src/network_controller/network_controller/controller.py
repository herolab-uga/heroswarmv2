#! /usr/bin/python3
import rclpy
import smbus
import struct
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import IMU
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from robot_msgs.msg import Light
from robot_msgs.msg import Enviornment

class Controller(Node):

    def __init__(self) -> None:
        # Creates the control node
        super().__init__("robot_controller")
        # Arduino Device Address
        self.arduino = 0x8
        # Init the i2c buss
        self.bus = smbus.SMBus(1)
        self.twist_sub = self.create_subscription(Twist,
                                                    "cmd_vel", 
                                                    self.read_twist,
                                                    10)
        self.imu_pub = self.create_publisher(IMU,"imu",2)
        self.mic_pub = self.create_publisher(Float64,"mic",2)
        self.light_pub = self.create_publisher(Light,'light',2)
        self.enviornment_pub = self.create_publisher(Enviornment,"enviornment",2)
        self.prox_pub = self.create_publisher(Float64,"proximity",2)
        
    def read_twist(self,msg) -> None:
        # Reads ths twist message x linear velocity
        x_velo = msg.linear.x
        
        # Reads the twist message y linear velocity
        y_velo = msg.linear.y

        # Reads the twist message z angular velocity
        z_angular = msg.angular.z

        # Logs the data
        self.get_logger().info("X Linear: {x} Y Linear: {y} Z Angular: {z}".format(x=x_velo,y=y_velo,z=z_angular))
        
        # Sends the velocity information to the feather board
        self.send_velocity([x_velo,y_velo,z_angular])

    def read_imu(self) -> None:
        # Creates the IMU message
        imu_msg = IMU()

        # Sets the orientation parameters
        imu_msg.orientation.x = 0
        imu_msg.orientation.y = 0
        imu_msg.orientation.z = None

        # Sets the angular velocity parameters
        imu_msg.angular_velocity.x = 0
        imu_msg.angular_velocity.y = 0
        imu_msg.angular_velocity.x = None

        # Sets the linear acceleration parameters
        imu_msg.linear_acceleration.x = None
        imu_msg.linear_acceleration.y = None
        imu_msg.linear_acceleration.z = 0

        # Publishes the message
        self.imu_pub.publish(imu_msg)
    
    def read_mic(self) -> None:
        # Creates the mic message
        mic_msg = Float64

        # Sets the meassage data value
        mic_msg.data = None

        # Publishes the message
        self.mic_pub.publish(mic_msg)

    
    def read_light(self) -> None:
        # Creates the light message
        light_msg = Light()
        
        # Sets the current rgbw value array
        light_msg.rgbw = None

        # Sets the gesture type
        light_msg.gesture.data = None

        # Publishes the message
        self.light_pub.publish(light_msg)

    def read_enviornment(self) -> None:
        # Creates the enviornment message
        enviorn_msg = Enviornment()

        # Sets the temperature
        enviorn_msg.temp = None

        # Sets the pressure 
        enviorn_msg.pressure = None

        # Sets the humidity
        enviorn_msg.humidity = None

        # Publishes the message
        self.enviornment_pub.publish(enviorn_msg)

    def read_proximity(self) -> None:
        # Creates the proximity message
        proximity_msg = Float64()
        
        # Sets the proximity value
        proximity_msg.data = None

        # Publishes the message
        self.prox_pub.publish(proximity_msg)

        

    # Sending an float to the arduino
    # Message format []
    def send_velocity(self,values) -> None:
        byteList = []

        # Converts the values to bytes 
        for value in values:
            byteList += list(struct.pack('f', value))
        byteList.append(0)  # fails to send last byte over I2C, hence this needs to be added 

        # Writes the values to the bus
        self.bus.write_i2c_block_data(self.arduino, byteList[0], byteList[1:20])
    
    
        


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)

