/* C Library Headers */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <vector>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <linux/reboot.h>
#include "includes/controller.hpp"

/* Linux headers */
#include <errno.h> // Error integer and strerror() function

/* Communication Headers */
#include "uart.hpp"

#define MAX_LINEAR_SPEED 0.1
#define LINEAR_THRESHOLD 0.01

#define ANGULAR_THRESHOLD 0.05
#define MAX_ANGULAR_SPEED 1.85

#define EQUAL 0

bool restart = false;

using std::placeholders::_1;

Controller::Controller():Node("controller")
{
    int ret = uartInit();
    if (ret != uartState::CONFIGURED)
    {
        std::cout << "uart not configured properly" << std::endl;
    }

    linXPos = 0.0;
    linYPos = 0.0;
    angZPos = 0.0;

    linXVel = 0.0;
    linYVel = 0.0;
    angZVel = 0.0;

    voltageBatt = 0.0;	

    /* Get the robot ID */
    robotId = std::getenv("ROBOTID");

    /*Get the robot namespace*/
    robotName = this->namespace();

    this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&Controller::readTwist, this, _1));
    this->create_subscription<std_msgs::msg::Int16MultiArray>("neopixel", 10, std::bind(&Controller::neopixelCallback, this, _1));
    this->create_subscription<std_msgs::msg::Float32>("battery", 10, std::bind(&Controller::batteryCallback, this, _1));
    this->create_subscription<std_msgs::msg::String>("shutdown", 10, std::bind(&Controller::shutdownCallback, this, _1));
    this->create_subscription<robot_msgs::msg::RobotPos>("/position", 10, std::bind(&Controller::getGlobalPos, this, _1));

    // Charger Services
    rclcpp::Client<robot_msgs::srv::GetCharger>::SharedPtr getCharger = this->create_client<robot_msgs::srv::GetCharger>("getCharger");
    rclcpp::Client<robot_msgs::srv::ReleaseCharger>::SharedPtr releaseCharger = this->create_client<robot_msgs::srv::ReleaseCharger>("releaseCharger");
}

// destructor stop send stop need a way to prioritize this call for uart communication priority mutex call?
Controller::~Controller()
{
    this->stop();
}

void Controller::getGlobalPos(const robot_msgs::msg::RobotPos::SharedPtr msg)
{
    // loop through list of robots
    std::vector<nav_msgs::msg::Odometry>::iterator robot;
    for (robot = msg->robot_pos.begin(); robot < msg->robot_pos.end(); robot++)
    {
        // if the robotid == id of msg
        if (this->robotId.compare(robot->.child_frame_id) == EQUAL)
        {
            this->lineXPos = robot->.pose.pose.position.x;
            this->linYPos = robot->.pose.pose.position.y;

            tf2::Quaternion q(
                robot->.pose.pose.orientation.x,
                robot->.pose.pose.orientation.y,
                robot->.pose.pose.orientation.z,
                robot->.pose.pose.orientation.w);

            tf2::Matrix3x3 m(q);

            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            this->angZPos = -yaw;
        }
    }
}

void getPos(const nav_msgs::msg::Odom::SharedPtr msg)
{
    this->lineXPos = msg->pose.pose.position.x;
    this->linYPos = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    this->angZPos = -yaw;
}

int Controller::sendValues(uint8_t *buff, size_t len)
{
    uint8_t buffer[64]

        memset(buffer, 0, 64);
    buffer[0] = 0xBE;
    buffer[1] = 0xEF;
    buffer[2] = opcode;

    // This is going to add the data to the message and the appropiate length
    memcpy(buffer + 2, buff, len);
    buffer[len + 4] = '\n';

    // Log message
    lockUartMutex();
    int ret = uartWrite(buffer, len + 4);
    unlockUartMutex();

    return ret;
}

void Controller::stop()
{
    uint8_t buff[13];
    buff[0] = 0x00;
    memset(buff, 0, 13);
    this->sendValues(buff, 13);
}

void Controller::readTwist(const geometry_msgs::msg::Twist::SharedPtr)
{
    x_velo = abs(msg->linear.x) > LINEAR_THRESHOLD ? min(max(msg.linear.x, -MAX_LINEAR_SPEED), MAX_LINEAR_SPEED) : 0.0;
    ang_z_velo = abs(msg->angular.z) > ANGULAR_THRESHOLD ? min(max(msg.linear.z, -MAX_ANGULAR_SPEED), MAX_ANGULAR_SPEED) : 0.0;

    uint8_t buff[13];
    memset(buff, 0, 13);
    memcpy(buff + 1, reinterpret_cast<uint8_t *>(x_velo), sizeof(float));
    memcpy(buff + 9, reinterpret_cast<uint8_t *>(ang_z_velo), sizeof(float));

    this->sendValues(buff, 13);
}

void Controller::neopixelCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
    uint8_t buff[7];
    memset(buff, 0, 7);
    buff[0] = 1.0;
    memcpy(buff + 1, reinterpret_cast<uint8_t *>(msg->data[0]), sizeof(uint16_t));
    memcpy(buff + 2, reinterpret_cast<uint8_t *>(msg->data[1]), sizeof(uint16_t));
    memcpy(buff + 5, reinterpret_cast<uint8_t *>(msg->data[2]), sizeof(uint16_t));

    this->sendValues(buff, 7);
}

void Controller::shutdownCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data.compare("restart") == EQUAL)
    {
        reboot(LINUX_REBOOT_CMD_RESTART);
    }
    else
    {
        reboot(LINUX_REBOOT_CMD_POWER_OFF);
    }
}

void Controller::batteryCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
     voltageBatt = msg.data;
}


int main(int argc, char *argv[])
{
	std::cout << "Starting" << std::endl;

	std::cout << "Spinning ROS Node" << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Controller>());
	rclcpp::shutdown();
    // if (restart == true)
    // {

    // } 
    // else
    // {

    // }
	return 0;
}
