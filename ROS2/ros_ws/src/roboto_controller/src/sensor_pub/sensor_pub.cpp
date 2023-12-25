/* C Library Headers */
#include <stdio.h>
#include <string.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>

/* Linux headers */
#include <errno.h> // Error integer and strerror() function

/* Ros Headers */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_msgs/msg/environment.hpp"
#include "robot_msgs/msg/light.hpp"
// #include "robot_msgs/sensor_enable.hpp"

/* Communication Headers */
#include "uart.hpp"
extern "C"
{
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}



#define LIS3MDL 0x1C
#define SHT31D 0x44
#define APDS9960 0x39
#define LSM6DS33 0x69

#define DEFAULT_PUB_RATE std::chrono::milliseconds(16) /* The default publishing rate for sensor data is 60 hz*/

/**
 * This class will publish sensor information gathered from the
 * Arduino Feather sense.
 **/
class SensorPublisher : public rclcpp::Node
{

	/**
	 * Private Variables
	 **/
private:
	/**
	 * These variables will hold the sensor publisher objects
	 **/
	rclcpp::Publisher<robot_msgs::msg::Light>::SharedPtr lightPublisher;			 /* Light publisher for sensed RGB and gestures */
	rclcpp::Publisher<robot_msgs::msg::Environment>::SharedPtr environmentPublisher; /* Environment publisher for temperature, pressure, humidity, and altitude */
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;				 /* IMU publisher or IMU information */
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr micPublisher;				 /* Mic publisher for volume picked up by mic */
	rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr proximityPublisher;			 /* Proximity publisher for distance information from front of robot */
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;			 /* Odometry publisher for odom data from feather senese */
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr batteryPublisher;			 /* Battery publisher for battery inforation */

	/**
	 * These variables will hold the sensor timmer objects
	 **/
	rclcpp::TimerBase::SharedPtr lightTimer;
	rclcpp::TimerBase::SharedPtr environmentTimer;
	rclcpp::TimerBase::SharedPtr imuTimer;
	rclcpp::TimerBase::SharedPtr micTimer;
	rclcpp::TimerBase::SharedPtr proximityTimer;
	rclcpp::TimerBase::SharedPtr odomTimer;
	rclcpp::TimerBase::SharedPtr batteryTimer;

	std::thread readUartThread;

	/**
	 * Light message variables
	 **/
	std::mutex lightMutex;
	int32_t rgbw[4] = {-1, -1, -1, -1};
	int32_t gesture = -1;

	/**
	 * Environment message variables
	 **/
	std::mutex environmentMutex;
	float temp = -1;
	float pressure = -1;
	float humidity = -1;
	float altitude = -1;

	/**
	 * IMU message variables
	 **/
	std::mutex imuMutex;

	/**
	 * Mic message variables
	 **/
	std::mutex micMutex;
	float volume = -1;

	/**
	 * Proximity message variables
	 **/
	std::mutex proximityMutex;
	int16_t prox = -1;

	/**
	 * Odom message variables
	 **/
	std::mutex odomMutex;
	float linX = -1;
	float linY = -1;
	float linZ = -1;
	float angX = -1;
	float angY = -1;
	float angZ = -1;

	float linVelX = -1;
	float linVelY = -1;
	float linVelZ = -1;
	float angVelX = -1;
	float angVelY = -1;
	float angVelZ = -1;

	/**
	 * Battery message variables
	 **/
	std::mutex batteryMutex;
	float bat = -1;

	/**
	 * I2C Mutex
	 **/
	std::mutex i2cMutex;
	int i2cFd;
	char i2cFileName[20];

	/**
	 * I2C Thread
	 **/
	std::thread readI2CThread;

	/**
	 * Public Functions
	 **/
public:
	/**
	 * SensorPublisher constructor.
	 * \return Function returns a SensorPublisher object
	 **/
	SensorPublisher()
		: Node("sensorPublisher")
	{

		// Create the publishers
		lightPublisher = this->create_publisher<robot_msgs::msg::Light>("/light", 5);
		environmentPublisher = this->create_publisher<robot_msgs::msg::Environment>("/environment", 5);
		imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 5);
		micPublisher = this->create_publisher<std_msgs::msg::Float32>("/mic", 5);
		proximityPublisher = this->create_publisher<std_msgs::msg::Int16>("/proximity", 5);
		odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 5);
		batteryPublisher = this->create_publisher<std_msgs::msg::Float32>("/battery", 5);

		// Create the timers
		lightTimer = this->create_wall_timer(DEFAULT_PUB_RATE, std::bind(&SensorPublisher::pubLight, this));
		environmentTimer = this->create_wall_timer(DEFAULT_PUB_RATE, std::bind(&SensorPublisher::pubEnvironment, this));
		// imuTimer = this->create_wall_timer(DEFAULT_PUB_RATE,std::bind(&SensorPublisher::pubIMU,this));
		micTimer = this->create_wall_timer(DEFAULT_PUB_RATE, std::bind(&SensorPublisher::pubMic, this));
		proximityTimer = this->create_wall_timer(DEFAULT_PUB_RATE, std::bind(&SensorPublisher::pubProximity, this));
		odomTimer = this->create_wall_timer(DEFAULT_PUB_RATE, std::bind(&SensorPublisher::pubOdom, this));
		batteryTimer = this->create_wall_timer(DEFAULT_PUB_RATE, std::bind(&SensorPublisher::pubOdom, this));

		// I2C Setup
		snprintf(i2cFileName, 19, "/dev/i2c-%d", 1);
		i2cFd = open(i2cFileName, O_RDWR);

		if (i2cFd < 0)
		{
			std::cout << "Error opening i2c file" << std::endl;
			exit(1);
		}

		if (setupSHT31D() != true)
		{
			std::cout << "Failed setting up SHT31D" << std::endl;
			exit(1);
		}

		// Instantiate the thread that will read from uart
		readUartThread = std::thread(&SensorPublisher::readUart, this);

		readI2CThread = std::thread(&SensorPublisher::readI2CSensors,this);
		std::cout << "Ready" << std::endl;
	}

	/**
	 * Private Functions
	 **/
private:
	// import adafruit_lis3mdl
	// from adafruit_apds9960.apds9960 import APDS9960
	// from adafruit_lsm6ds.lsm6ds33 import LSM6DS33

	bool setupSHT31D()
	{
		std::cout << "Starting SHT31D Setup" << std::endl;
		if (ioctl(i2cFd, I2C_SLAVE, SHT31D) < 0)
		{
			std::cout << "Faild to set SHT31D I2C Slave" << std::endl;
			return false;
		}
		// High repeatability with clock stretching
		i2c_smbus_write_word_data(i2cFd,0x2C, 0x06);

		// High repeatability 10 measurements per second
		i2c_smbus_write_word_data(i2cFd,0x27, 0x37);

		return true;
	}

	void readSHT31D()
	{
		// Request data
		i2c_smbus_write_word_data(i2cFd,0xE0, 0x00);

		uint8_t temp_h = i2c_smbus_read_byte(i2cFd);
		uint8_t temp_l = i2c_smbus_read_byte(i2cFd);
		uint8_t humidity_h = i2c_smbus_read_byte(i2cFd);
		uint8_t humidity_l = i2c_smbus_read_byte(i2cFd);

		environmentMutex.lock();
		humidity = 100.0 * (float)((humidity_h << 8) | humidity_l) / 65535.0;
		temp = -45.0 + 175 *(float)((temp_h << 8) | temp_l) / 65535.0;
		environmentMutex.unlock();

		// Request Status Register
		i2c_smbus_write_word_data(i2cFd,0xF3, 0x2D);
		std::cout << "Upper: " << std::hex << (i2c_smbus_read_byte(i2cFd)) << std::endl;
		std::cout << "Lower: " << std::hex << (i2c_smbus_read_byte(i2cFd))  << std::endl;
	}

	void readI2CSensors()
	{
		while(true)
		{
			// readPressure();
			readSHT31D();
			// add a sleep to test timing
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}

	// create function for reading data from uart
	void readUart()
	{
		uint8_t buf[255];
		while (true)
		{
			lockMutex();
			uartRead(buf, sizeof(buf));
			unlockMutex();

			odomMutex.lock();
			memcpy(&linX, buf, sizeof(float));
			memcpy(&linY, buf + 4, sizeof(float));
			memcpy(&angZ, buf + 8, sizeof(float));
			memcpy(&linVelX, buf + 12, sizeof(float));
			memcpy(&angVelZ, buf + 16, sizeof(float));
			odomMutex.unlock();

			batteryMutex.lock();
			memcpy(&bat, buf + 20, sizeof(float));
			batteryMutex.unlock();

			micMutex.lock();
			memcpy(&volume, buf + 24, sizeof(float));
			micMutex.unlock();
		}
	}

	/**
	 * Function to publish light information
	 **/
	void pubLight()
	{
		auto lightMsg = robot_msgs::msg::Light();
		lightMutex.lock();
		// Insert RGBW values into message
		lightMsg.rgbw.push_back(rgbw[0]);
		lightMsg.rgbw.push_back(rgbw[1]);
		lightMsg.rgbw.push_back(rgbw[2]);
		lightMsg.rgbw.push_back(rgbw[3]);

		lightMsg.gesture = gesture;
		lightMutex.unlock();
		lightPublisher->publish(lightMsg);
	}

	void pubEnvironment()
	{
		auto environmentMsg = robot_msgs::msg::Environment();
		environmentMutex.lock();
		environmentMsg.temp = temp;
		environmentMsg.pressure = pressure;
		environmentMsg.humidity = humidity;
		environmentMsg.altitude = altitude;
		environmentMutex.unlock();
		environmentPublisher->publish(environmentMsg);
	}

	void pubIMU()
	{
	}

	void pubMic()
	{
		auto micMsg = std_msgs::msg::Float32();
		micMutex.lock();
		micMsg.data = volume;
		micMutex.unlock();
		micPublisher->publish(micMsg);
	}

	void pubProximity()
	{
		auto proxMsg = std_msgs::msg::Int16();
		proximityMutex.lock();
		proxMsg.data = prox;
		proximityMutex.unlock();
		proximityPublisher->publish(proxMsg);
	}

	void pubOdom()
	{
		auto odomMsg = nav_msgs::msg::Odometry();
		odomMutex.lock();

		odomMsg.pose.pose.position.x = linX;
		odomMsg.pose.pose.position.y = linY;
		odomMsg.pose.pose.position.z = 0;

		odomMsg.pose.pose.orientation.x = 0;
		odomMsg.pose.pose.orientation.y = 0;
		odomMsg.pose.pose.orientation.z = angZ;

		odomMsg.twist.twist.linear.x = linVelX;
		odomMsg.twist.twist.linear.y = linVelY;
		odomMsg.twist.twist.linear.z = 0;

		odomMsg.twist.twist.angular.x = 0;
		odomMsg.twist.twist.angular.y = 0;
		odomMsg.twist.twist.angular.z = angVelZ;
		odomMutex.unlock();

		odomPublisher->publish(odomMsg);
	}

	void pubBattery()
	{
		auto battMsg = std_msgs::msg::Float32();
		batteryMutex.lock();
		battMsg.data = bat;
		batteryMutex.unlock();
		batteryPublisher->publish(battMsg);
	}

	// create function for service
};

int main(int argc, char *argv[])
{
	std::cout << "Starting" << std::endl;
	if (tryUartLock())
	{
		uartInit();
		unlockMutex();
	}

	std::cout << "Spinning ROS Node" << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorPublisher>());
	rclcpp::shutdown();
	return 0;
}
