/* C Library Headers */
#include <stdio.h>
#include <thread>
#include <chrono>
#include <string.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"


#include "BMP280/BMP280.hpp"
#include "APDS9960/APDS9960.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<APDS9960Publisher> apds9960Publisher; 
    std::shared_ptr<BMP280Publisher> bmp280Publisher; 
    

    // Set real-time priority
    struct sched_param param;
    param.sched_priority = 55; // moderate RT priority
    if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        // ROS_WARN("Failed to set thread priority");
    }

	// RCLCPP_INFO(this->get_logger(), "Starting APDS9960 Node");
    apds9960Publisher = std::make_shared<APDS9960Publisher>();

    // RCLCPP_INFO(this->get_logger(), "Starting BMP280 Node");
    bmp280Publisher = std::make_shared<BMP280Publisher>();



    rclcpp::experimental::executors::EventsExecutor exec;
    exec.add_node(apds9960Publisher);
    exec.add_node(bmp280Publisher);
    exec.spin();
	
    rclcpp::shutdown();
	
    return 0;
}