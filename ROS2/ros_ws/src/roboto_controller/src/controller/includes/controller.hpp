/* C Library Headers */
#include <stdio.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>

// ROS headers
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "robot_msgs/srv/get_charger.hpp"
#include "robot_msgs/srv/release_charger.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "robot_msgs/msg/robot_pos.hpp"
#include "nav_msgs/msg/odometry.hpp"
/* Linux headers */
#include <errno.h> // Error integer and strerror() function

/* Communication Headers */
#include "uart.hpp"

class Controller : public rclcpp::Node
{

private:
    std::string robotId;
    std::string robotName;

    // Position
    float linXPos;
    float linYPos;
    float angZPos;

    // Velocity
    float linXVel;
    float linYVel;
    float angZVel;

    float voltageBatt;

    // Charger
    struct
    {
        uint16_t chargerId;
        float x;
        float y;
        float z;

    } charger;

    rclcpp::Client<robot_msgs::srv::GetCharger>::SharedPtr getChargerService;
    rclcpp::Client<robot_msgs::srv::ReleaseCharger>::SharedPtr releaseChargerService;

public:
    Controller();
    ~Controller();

private:
    void getGlobalPos(const robot_msgs::msg::RobotPos::SharedPtr msg);
    void getPos(const nav_msgs::msg::Odometry::SharedPtr msg);
    int sendValues(uint8_t *buff, size_t length);
    void stop();
    void readTwist(const geometry_msgs::msg::Twist::SharedPtr msg);
    void neopixelCallback(const std_msgs::msg::Int16MultiArray::SharedPtr);
    void shutdownCallback(const std_msgs::msg::String::SharedPtr msg);
    void requestCharger();
    void releaseCharger();
    void batteryCallback(const std_msgs::msg::Float32::SharedPtr);
};
