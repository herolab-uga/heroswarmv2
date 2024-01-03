/* C Library Headers */
#include <stdio.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <mutex>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>

// ROS headers
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_msgs/msg/environment.hpp"
#include "robot_msgs/msg/light.hpp"

/* Linux headers */
#include <errno.h> // Error integer and strerror() function

/* Communication Headers */
#include "uart.hpp"
extern "C"
{
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

// I2C sensor's slave address
#define BMP280 0x77
#define SHT31D 0x44
#define LIS3MDL 0x1C
#define LSM6DS33 0x69
#define APDS9960 0x39

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
	 * Light message variables
	 **/
	std::mutex lightMutex;
	int32_t rgbw[4];
	int32_t gesture;

	/**
	 * Environment message variables
	 **/
	std::mutex environmentMutex;
	float temp;
	float pressure;
	float humidity;
	float altitude;

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
	float linX;
	float linY;
	float linZ;
	float angX;
	float angY;
	float angZ;

	float linVelX;
	float linVelY;
	float linVelZ;
	float angVelX;
	float angVelY;
	float angVelZ;

	/**
	 * Battery message variables
	 **/
	std::mutex batteryMutex;
	float bat;

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
	SensorPublisher();
	/**
	 * Private Functions
	 **/
private:
	// Functions for setting up and reading I2C sensor
	bool setupLIS3MDL();

	bool readMagField();

	bool setupSHT31D();

	bool readSHT31D();

	bool readParamsBMP280();

	bool setupBMP280();

	bool readPressure();

	bool setupLSM6DS33();

	bool readLSM6DS33();

	bool setupAPDS9960();

	bool readColor();

	bool readProx();

	void readI2CSensors();

	// create function for reading data from uart
	void readUart();

	/**
	 * Function to publish light information
	 **/
	void pubLight();

	void pubEnvironment();

	void pubIMU();

	void pubMic();

	void pubProximity();

	void pubOdom();

	void pubBattery();

	// create function for sensor enable service
};
