#include "includes/sensor_pub.hpp"

#define LIS3MDL 0x1C

rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;				 /* IMU publisher or IMU information */

/**
 * These variables will hold the sensor timmer objects
 **/

rclcpp::TimerBase::SharedPtr imuTimer;

/**
 * IMU message variables
 **/
 std::mutex imuMutex;

// need to add parameters to know what sensors to turn on and subscirber so you can toggle them in real time
	// Create the publishers	
	imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 5);

	
	// imuTimer = this->create_wall_timer(DEFAULT_PUB_RATE,std::bind(&SensorPublisher::pubIMU,this));

bool SensorPublisher::setupLIS3MDL()
{
    std::cout << "Starting LIS3MDL Setup" << std::endl;
    if (ioctl(i2cFd, I2C_SLAVE, LIS3MDL) < 0)
    {
        std::cout << "Faild to set LIS3MDL I2C Slave" << std::endl;
        return false;
    }

    if (i2c_smbus_write_byte_data(i2cFd, 0x20, 0xE2) != 0)
    {
        std::cout << "Failed sending BMP280 config" << std::endl;
        return false;
    }

    if (i2c_smbus_write_byte_data(i2cFd, 0x21, 0x00) != 0)
    {
        std::cout << "Failed sending BMP280 config" << std::endl;
        return false;
    }

    if (i2c_smbus_write_byte_data(i2cFd, 0x22, 0x00) != 0)
    {
        std::cout << "Failed sending BMP280 config" << std::endl;
        return false;
    }

    if (i2c_smbus_write_byte_data(i2cFd, 0x23, 0x0C) != 0)
    {
        std::cout << "Failed sending BMP280 config" << std::endl;
        return false;
    }

    if (i2c_smbus_write_byte_data(i2cFd, 0x24, 0x00) != 0)
    {
        std::cout << "Failed sending BMP280 config" << std::endl;
        return false;
    }

    return true;
}

// still need to figure out the transfer function for this
bool SensorPublisher::readMagField()
{
    // uint16_t magX = i2c_smbus_read_block_data(i2cFd, 0x28);
    // uint16_t magY = i2c_smbus_read_block_data(i2cFd, 0x2a);
    // uint16_t magZ = i2c_smbus_read_block_data(i2cFd, 0x2c);
    // uint16_t temp = i2c_smbus_read_block_data(i2cFd, 0x2e);
    return false;
}
