/* C Library Headers */
#include <stdio.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h> 
#include <mutex>
#include "sensor_manager/sensor_manager.hpp"

/* Linux headers */
#include <errno.h> // Error integer and strerror() function


/**
 * I2C Mutex
 **/
 std::mutex gI2cMutex;
int gI2cFd;
char i2cFileName[20];

init_sensor()
{
    gI2cFd = 
}

int get_i2c_fd()
{
    gI2cMutex.lock();
    return gI2cFd;
}

int release_i2c_fd()
{
    gI2cMutex.unlock();
    return -1;
}

