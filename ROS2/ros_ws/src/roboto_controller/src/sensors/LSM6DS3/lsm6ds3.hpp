#ifndef LSM6DS3_HPP
#define LSM6DS3_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <semaphore.h>
#include <mutex>

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#include "LSM6DS3/LSM6DS3Sensor.hpp"

#define DEFAULT_IMU_PUB_RATE std::chrono::milliseconds(16)  // 60 Hz

class LSM6DS3Publisher : public rclcpp::Node
{
public:
    LSM6DS3Publisher();

private:
    void pubIMU();

    // I2C resources
    sem_t* gI2cSemaphore;
    int gI2cFd;

    // Sensor driver
    std::unique_ptr<LSM6DS3Sensor> imu;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperaturePublisher;

    // Timer
    rclcpp::TimerBase::SharedPtr imuTimer;

    // Thread safety for msg variables
    std::mutex imuMutex;

    // Node parameters
    std::string i2cBus;
    int i2cAddress;
};

#endif
