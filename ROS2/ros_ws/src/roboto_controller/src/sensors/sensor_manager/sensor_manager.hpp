/* C Library Headers */
#include <stdio.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <mutex>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>

/* Linux headers */
#include <errno.h> // Error integer and strerror() function

/* Communication Headers */
extern "C"
{
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#define DEFAULT_PUB_RATE std::chrono::milliseconds(16) /* The default publishing rate for sensor data is 60 hz*/


void init_sensor_manager();

int get_i2c_fd();
void release_i2c_fd();

