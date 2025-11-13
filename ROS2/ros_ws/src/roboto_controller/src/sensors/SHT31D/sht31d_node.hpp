#ifndef SHT31D_HPP
#define SHT31D_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <semaphore.h>
#include <mutex>

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#include "SHT31D/sht31d.hpp"

class SHT31DPublisher : public rclcpp::Node
{
public:
    SHT31DPublisher();

private:
    void pubSHT31D();

    // I2C resources
    sem_t* gI2cSemaphore;
    int    gI2cFd;

    // Sensor driver
    std::unique_ptr<Adafruit_SHT31D> sht;

    // Publishers & timer
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temperaturePublisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr humidityPublisher;
    rclcpp::TimerBase::SharedPtr shtTimer;

    // Thread safety
    std::mutex shtMutex;

    // Optional: frame / topic names (if you ever want to param them)
};

#endif
