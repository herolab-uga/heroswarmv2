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
#include "router.h"

#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>


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

#define RED_INDEX           (0)
#define GREEN_INDEX         (1)
#define BLUE_INDEX          (2)
#define BRIGHTNESS_INDEX    (3)
#define NEOPIXEL_MSG_APID   (1)

#define DEFAULT_PUB_RATE std::chrono::milliseconds(16) /* The default publishing rate for sensor data is 60 hz*/


bool restart = false;


using std::placeholders::_1;
using namespace std::chrono_literals;

float ODOM_COVARIANCE_MATRIX[36] = { 1e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
                           			0.0, 1e-2, 0.0, 0.0, 0.0, 0.0,
                           			0.0, 0.0, 1e-2, 0.0, 0.0, 0.0,
                           			0.0, 0.0, 0.0, 1e-2, 0.0, 0.0,
                           			0.0, 0.0, 0.0, 0.0, 1e-2, 0.0,
                           			0.0, 0.0, 0.0, 0.0, 0.0, 1e-2 };

float IMU_COVARIANCE_MATRIX[9] = {1e-2, 0.0, 0.0, 
									0.0, 1e-2, 0.0, 
									0.0, 0.0, 1e-2};


std::shared_ptr<Controller> controller; 


int update_odom(uint16_t length, void* args)
{
    // TODO: Fix this
    if (length > 99)
    {
        return -1;
    }

    float linX = 0;
    float linY = 0;
    float angZ = 0;

    float linVelX = 0;
    float linVelY = 0;
    float angVelZ = 0;

    memcpy(&linVelX, &((float*)args)[0], sizeof(linVelX));
    memcpy(&linVelY, &((float*)args)[1], sizeof(linVelY));
    memcpy(&angVelZ, &((float*)args)[2], sizeof(angVelZ));

    memcpy(&linX, &((float*)args)[3], sizeof(linX));
    memcpy(&linY, &((float*)args)[4], sizeof(linY));
    memcpy(&angZ, &((float*)args)[5], sizeof(angZ));


    controller->setOdom(linX, linY, angZ, linVelX, linVelY, angVelZ);

    return 0;
}


Controller::Controller():Node("controller")
{

    init_router();

    ROUTER_REGISTER(0xFF, update_odom);

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
    // battery = this->create_subscription<std_msgs::msg::Float32>("battery", 10, std::bind(&Controller::batteryCallback, this, _1));
    shutdown = this->create_subscription<std_msgs::msg::String>("shutdown", 10, std::bind(&Controller::shutdownCallback, this, _1));
    pos = this->create_subscription<robot_msgs::msg::RobotPos>("/position", 10, std::bind(&Controller::getGlobalPos, this, _1));
    
    odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 5);
	batteryPublisher = this->create_publisher<std_msgs::msg::Float32>("battery", 5);

    odomTimer = this->create_wall_timer(DEFAULT_PUB_RATE, std::bind(&Controller::pubOdom, this));
	batteryTimer = this->create_wall_timer(DEFAULT_PUB_RATE, std::bind(&Controller::pubBattery, this));

    // Moving this here cause it needs to send a uart message
    neopixel_sub = this->create_subscription<std_msgs::msg::Int16MultiArray>("neopixel", 10, std::bind(&Controller::neopixelCallback, this, _1));

    // Charger Services
    // getChargerService = this->create_client<robot_msgs::srv::GetCharger>("getCharger");
    // releaseChargerService = this->create_client<robot_msgs::srv::ReleaseCharger>("releaseCharger");

    init_uart();

    std::cout << "Controller setup finished" << std::endl;
}

// destructor stop send stop need a way to prioritize this call for uart communication priority mutex call?
Controller::~Controller()
{
    this->stop();
}

// Moving this here cause it needs to send a uart message
void Controller::neopixelCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
    uint8_t buff[4] = {0};
    buff[RED_INDEX] = msg->data[RED_INDEX];
    buff[BLUE_INDEX] = msg->data[BLUE_INDEX];
    buff[GREEN_INDEX] = msg->data[GREEN_INDEX];
    buff[BRIGHTNESS_INDEX] = msg->data[BRIGHTNESS_INDEX];

    uart_send_message(NEOPIXEL_MSG_APID, buff, sizeof(buff));
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
            this->linXPosGlobal = robot->pose.pose.position.x;
            this->linYPosGlobal = robot->pose.pose.position.y;

            tf2::Quaternion q(
                robot->pose.pose.orientation.x,
                robot->pose.pose.orientation.y,
                robot->pose.pose.orientation.z,
                robot->pose.pose.orientation.w);

            tf2::Matrix3x3 m(q);

            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            this->angZPosGlobal = -yaw;
        }
    }
}

void Controller::setOdom(float linX, float linY, float angZ, float linVelX, float linVelY, float angVelZ)
{
    std::lock_guard<std::mutex> lock(this->odomMutex);
    
    this->linXPos = linX;
    this->linYPos = linY;
    this->angZPos = angZ;

    this->linXVel = linVelX;
    this->linYVel = linVelY;
    this->angZVel = angVelZ;

}


void Controller::stop()
{
    uint8_t buff[12];
    memset(buff, 0, sizeof(buff));
    uart_send_message(VELOCITY_APID, buff, 9);
}

void Controller::readTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // std::cout << "X: " << static_cast<float>(msg->linear.x) << " | Z: " <<  static_cast<float>(msg->angular.z) << std::endl; 
    float x_velo = abs(msg->linear.x) > LINEAR_THRESHOLD ? std::min(std::max(static_cast<float>(msg->linear.x), -MAX_LINEAR_SPEED), MAX_LINEAR_SPEED) : 0.0;
    float ang_z_velo = abs(msg->angular.z) > ANGULAR_THRESHOLD ? std::min(std::max(static_cast<float>(msg->angular.z), -MAX_ANGULAR_SPEED), MAX_ANGULAR_SPEED) : 0.0;

    uint8_t buff[12];
    std::memset(buff, 0, sizeof(buff));
    std::memcpy(&((float*)buff)[0], &x_velo, sizeof(float));
    std::memcpy(&((float*)buff)[2], &ang_z_velo, sizeof(float));

    uart_send_message(VELOCITY_APID, buff, sizeof(buff));

    // RCLCPP_INFO(this->get_logger(), "Setting volocity to X: %f | Angular: %f", x_velo, ang_z_velo);
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

void Controller::pubOdom()
{
	auto odomMsg = nav_msgs::msg::Odometry();
	std::lock_guard<std::mutex> lock(this->odomMutex);

	odomMsg.pose.pose.position.x = this->linXPos;
	odomMsg.pose.pose.position.y = this->linYPos;
	odomMsg.pose.pose.position.z = 0;

	tf2::Quaternion m;
	m.setRPY(0,0,this->angZPos);

	// I need to convert from rpy to quaternion
	odomMsg.pose.pose.orientation.x = m.getX();
	odomMsg.pose.pose.orientation.y = m.getY();
	odomMsg.pose.pose.orientation.z = m.getZ();
	odomMsg.pose.pose.orientation.w = m.getW();

	odomMsg.twist.twist.linear.x = this->linXVel;
	odomMsg.twist.twist.linear.y = this->linYVel;
	odomMsg.twist.twist.linear.z = 0;

	odomMsg.twist.twist.angular.x = 0;
	odomMsg.twist.twist.angular.y = 0;
	odomMsg.twist.twist.angular.z = this->angZVel;

	this->odomPublisher->publish(odomMsg);
}

void Controller::pubBattery()
{
	auto battMsg = std_msgs::msg::Float32();
	std::lock_guard<std::mutex> lock(this->batteryMutex);
	battMsg.data = this->voltageBatt;
	this->batteryMutex.unlock();
	this->batteryPublisher->publish(battMsg);
}


int main(int argc, char *argv[])
{
    // Set real-time priority
    struct sched_param param;
    param.sched_priority = 60; // moderate RT priority
    if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        // ROS_WARN("Failed to set thread priority");
    }

	std::cout << "Starting" << std::endl;

	std::cout << "Spinning ROS Node TESTING" << std::endl;
	rclcpp::init(argc, argv);
    controller = std::make_shared<Controller>();    
    rclcpp::experimental::executors::EventsExecutor exec;

    exec.add_node(controller);
    exec.spin();

    rclcpp::shutdown();
	
    return 0;
}
