/* C Library Headers */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <vector>
#include <chrono>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/reboot.h>
#include <sys/syscall.h>
#include <linux/reboot.h>
#include "controller.hpp"

/* Linux headers */
#include <errno.h> // Error integer and strerror() function

/* Communication Headers */
#include "uart.hpp"

#include "nav_msgs/msg/odometry.hpp"

#define EQUAL 0

#define VELOCITY_APID   (0x00)

#define CHARGETHRESH 4.0f
#define DISCHARGETHRESH 3.5f

#define MAX_LINEAR_SPEED 0.33f
#define LINEAR_THRESHOLD 0.03f

#define ANGULAR_THRESHOLD 0.05f
#define MAX_ANGULAR_SPEED 1.85f

#define DEFAULT_PUB_RATE std::chrono::milliseconds(16) /* The default publishing rate for sensor data is 60 hz*/


bool restart = false;

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

using std::placeholders::_1;
using namespace std::chrono_literals;

rclcpp::TimerBase::SharedPtr odomTimer;
rclcpp::TimerBase::SharedPtr batteryTimer;

float ODOM_COVARIANCE_MATRIX[36] = { 1e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
                           			0.0, 1e-2, 0.0, 0.0, 0.0, 0.0,
                           			0.0, 0.0, 1e-2, 0.0, 0.0, 0.0,
                           			0.0, 0.0, 0.0, 1e-2, 0.0, 0.0,
                           			0.0, 0.0, 0.0, 0.0, 1e-2, 0.0,
                           			0.0, 0.0, 0.0, 0.0, 0.0, 1e-2 };

float IMU_COVARIANCE_MATRIX[9] = {1e-2, 0.0, 0.0, 
									0.0, 1e-2, 0.0, 
									0.0, 0.0, 1e-2};



Controller::Controller():Node("controller")
{
    init_uart();

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
    robotName = this->get_namespace();
	
    std::cout << "Creating Subscriptions" << std::endl;    
    
    // Standard Nodes for any robot
    cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&Controller::readTwist, this, _1));
    battery = this->create_subscription<std_msgs::msg::Float32>("battery", 10, std::bind(&Controller::batteryCallback, this, _1));
    shutdown = this->create_subscription<std_msgs::msg::String>("shutdown", 10, std::bind(&Controller::shutdownCallback, this, _1));
    pos = this->create_subscription<robot_msgs::msg::RobotPos>("/position", 10, std::bind(&Controller::getGlobalPos, this, _1));
    
    odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 5);
	batteryPublisher = this->create_publisher<std_msgs::msg::Float32>("/battery", 5);

    odomTimer = this->create_wall_timer(DEFAULT_PUB_RATE, std::bind(&Controller::pubOdom, this));
	batteryTimer = this->create_wall_timer(DEFAULT_PUB_RATE, std::bind(&Controller::pubOdom, this));

    // Charger Services
    getChargerService = this->create_client<robot_msgs::srv::GetCharger>("getCharger");
    releaseChargerService = this->create_client<robot_msgs::srv::ReleaseCharger>("releaseCharger");

    std::cout << "Controller setup finished" << std::endl;
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
        if (this->robotId.compare(robot->child_frame_id) == EQUAL)
        {
            this->linXPos = robot->pose.pose.position.x;
            this->linYPos = robot->pose.pose.position.y;

            tf2::Quaternion q(
                robot->pose.pose.orientation.x,
                robot->pose.pose.orientation.y,
                robot->pose.pose.orientation.z,
                robot->pose.pose.orientation.w);

            tf2::Matrix3x3 m(q);

            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            this->angZPos = -yaw;
        }
    }
}

void Controller::getPos(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    this->linXPos = msg->pose.pose.position.x;
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

void Controller::stop()
{
    uint8_t buff[9];
    memset(buff, 0, 9);
    uart_send_message(VELOCITY_APID, buff, 9);
}

void Controller::readTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::cout << "X: " << static_cast<float>(msg->linear.x) << " | Z: " <<  static_cast<float>(msg->angular.z) << std::endl; 
    float x_velo = abs(msg->linear.x) > LINEAR_THRESHOLD ? std::min(std::max(static_cast<float>(msg->linear.x), -MAX_LINEAR_SPEED), MAX_LINEAR_SPEED) : 0.0;
    float ang_z_velo = abs(msg->angular.z) > ANGULAR_THRESHOLD ? std::min(std::max(static_cast<float>(msg->angular.z), -MAX_ANGULAR_SPEED), MAX_ANGULAR_SPEED) : 0.0;

    uint8_t buff[9];
    std::memset(buff, 0, 9);
    std::memcpy(buff + 1, reinterpret_cast<uint8_t *>(&x_velo), sizeof(float));
    std::memcpy(buff + 5, reinterpret_cast<uint8_t *>(&ang_z_velo), sizeof(float));

    uart_send_message(VELOCITY_APID, buff, 9);
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
    voltageBatt = msg->data;

    // if (voltageBatt <= DISCHARGETHRESH)
    // {
    //     auto request = std::make_shared<robot_msgs::srv::GetCharger::Request>();
    //     request -> name.set__name(this->robotName);

    //     while (!this->getChargerService->wait_for_service(1s))
    //     {
    //         if (!rclcpp::ok())
    //         {
    //             std::cout << "Interrupted while waiting for Get Charger service" << std::endl;
    //         }
    //         std::cout << "Service not available, waiting again..." << std::endl;
    //     }

    //     auto result = this->getChargerService->async_send_request(request);

    //     if (rclcpp::spin_until_future_complete(this,result) == rclcpp::FutureReturnCode::SUCCESS)
    //     {
    //         this->charger.chargerId = result.get()->id;
    //         this->charger.x = result.get()->position.x
    //         this->charger.y = result.get()->position.y
    //         this->charger.z = result.get()->position.z
    //     }
    // }
}

// def quaternion_from_rpy(self, roll, pitch, yaw):
//         cy = math.cos(yaw * 0.5)
//         sy = math.sin(yaw * 0.5)
//         cp = math.cos(pitch * 0.5)
//         sp = math.sin(pitch * 0.5)
//         cr = math.cos(roll * 0.5)
//         sr = math.sin(roll * 0.5)

//         q = [0] * 4
//         q[0] = sr * cp * cy - cr * sp * sy
//         q[1] = cr * sp * cy + sr * cp * sy
//         q[2] = cr * cp * sy - sr * sp * cy
//         q[3] = cr * cp * cy + sr * sp * sy
//         return q

void Controller::pubOdom()
{
	auto odomMsg = nav_msgs::msg::Odometry();
	odomMutex.lock();

	odomMsg.pose.pose.position.x = linX;
	odomMsg.pose.pose.position.y = linY;
	odomMsg.pose.pose.position.z = 0;

	tf2::Quaternion m;
	m.setRPY(0,0,angZ);

	// I need to convert from rpy to quaternion
	odomMsg.pose.pose.orientation.x = m.getX();
	odomMsg.pose.pose.orientation.y = m.getY();
	odomMsg.pose.pose.orientation.z = m.getZ();
	odomMsg.pose.pose.orientation.w = m.getW();

	odomMsg.twist.twist.linear.x = linVelX;
	odomMsg.twist.twist.linear.y = linVelY;
	odomMsg.twist.twist.linear.z = 0;

	odomMsg.twist.twist.angular.x = 0;
	odomMsg.twist.twist.angular.y = 0;
	odomMsg.twist.twist.angular.z = angVelZ;
	odomMutex.unlock();

	odomPublisher->publish(odomMsg);
}

void Controller::pubBattery()
{
	auto battMsg = std_msgs::msg::Float32();
	batteryMutex.lock();
	battMsg.data = bat;
	batteryMutex.unlock();
	batteryPublisher->publish(battMsg);
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
