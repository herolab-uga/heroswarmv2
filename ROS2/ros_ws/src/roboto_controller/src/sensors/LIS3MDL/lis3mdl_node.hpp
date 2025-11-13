#ifndef LIS3MDL_HPP
#define LIS3MDL_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <semaphore.h>
#include <mutex>

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#include "LIS3MDL/lis3mdl.hpp"

class LIS3MDLPublisher : public rclcpp::Node
{
public:
    LIS3MDLPublisher();

private:
    void pubMag();

    // I2C resources (pattern to match your other nodes)
    sem_t* gI2cSemaphore;
    int    gI2cFd;

    // Sensor driver
    std::unique_ptr<Adafruit_LIS3MDL> mag;

    // Publishers & timer
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magPublisher;
    rclcpp::TimerBase::SharedPtr magTimer;

    // Thread safety
    std::mutex magMutex;

    // Parameters
    int         rangeGauss;    // 4, 8, 12, 16
    int         dataRateHz;    // 1,2,3,5,10,20,40,80,155,300,560,1000
    double      publishRateHz; // any positive value
    std::string frameId;
};

#endif
