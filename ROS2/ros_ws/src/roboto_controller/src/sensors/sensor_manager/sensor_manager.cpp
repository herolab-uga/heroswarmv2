/* C Library Headers */
#include <stdio.h>
#include <thread>
#include <chrono>
#include <string.h>
#include <iostream>

#include <pthread.h>
#include <sched.h>

#include "rclcpp/rclcpp.hpp"

/* Existing sensors */
#include "BMP280/BMP280.hpp"
#include "APDS9960/APDS9960.hpp"

/* IMU */
#include "LSM6DS3/lsm6ds3.hpp"

/* Magnetometer */
#include "LIS3MDL/lis3mdl_node.hpp"

/* Temperature/Humidity */
#include "SHT31D/sht31d_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<APDS9960Publisher> apds9960Publisher; 
    std::shared_ptr<BMP280Publisher> bmp280Publisher; 
    std::shared_ptr<LSM6DS3Publisher> lsm6ds3Publisher;
    std::shared_ptr<LIS3MDLPublisher> lis3mdlPublisher;
    std::shared_ptr<SHT31DPublisher> sht31dPublisher;

    // =====================================================
    // Set CPU AFFINITY → pin this process to CPU core 1
    // =====================================================
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset);   // core index 1

    pthread_t thread = pthread_self();
    int aff_ret = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (aff_ret != 0) {
        std::cerr << "[WARN] Failed to set CPU affinity to core 1: "
                  << strerror(aff_ret) << std::endl;
    } else {
        std::cout << "[INFO] CPU affinity set to core 1" << std::endl;
    }

    // =====================================================
    // Real-time scheduling (FIFO priority 55)
    // =====================================================
    struct sched_param param;
    param.sched_priority = 55; 

    if (pthread_setschedparam(thread, SCHED_FIFO, &param) != 0) {
        std::cerr << "[WARN] Failed to set realtime scheduler (SCHED_FIFO)" 
                  << std::endl;
    }

    // =====================================================
    // Start sensor nodes
    // =====================================================
    apds9960Publisher  = std::make_shared<APDS9960Publisher>();
    bmp280Publisher    = std::make_shared<BMP280Publisher>();
    lsm6ds3Publisher   = std::make_shared<LSM6DS3Publisher>();
    lis3mdlPublisher   = std::make_shared<LIS3MDLPublisher>();
    sht31dPublisher    = std::make_shared<SHT31DPublisher>();

    // =====================================================
    // Executor
    // =====================================================
    rclcpp::experimental::executors::EventsExecutor exec;

    exec.add_node(apds9960Publisher);
    exec.add_node(bmp280Publisher);
    exec.add_node(lsm6ds3Publisher);
    exec.add_node(lis3mdlPublisher);
    exec.add_node(sht31dPublisher);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
