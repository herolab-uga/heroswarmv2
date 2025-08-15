#include "std_msgs/msg/int16_multi_array.hpp"



rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr neopixel;

    // make this part of the configurable features
    neopixel = this->create_subscription<std_msgs::msg::Int16MultiArray>("neopixel", 10, std::bind(&Controller::neopixelCallback, this, _1));


void Controller::neopixelCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
    uint8_t buff[7];
    std::memset(buff, 0, 7);
    buff[0] = 1.0;
    std::memcpy(buff + 1, reinterpret_cast<uint8_t *>(msg->data[0]), sizeof(uint16_t));
    std::memcpy(buff + 2, reinterpret_cast<uint8_t *>(msg->data[1]), sizeof(uint16_t));
    std::memcpy(buff + 5, reinterpret_cast<uint8_t *>(msg->data[2]), sizeof(uint16_t));

    this->sendValues(buff, 7);
}