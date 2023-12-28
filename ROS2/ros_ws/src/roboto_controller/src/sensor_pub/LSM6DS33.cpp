#include "includes/sensor_pub.hpp"

bool SensorPublisher::setupLSM6DS33()
{
    std::cout << "Starting LSM6DS33 Setup" << std::endl;
    if (ioctl(i2cFd, I2C_SLAVE, LSM6DS33) < 0)
    {
        std::cout << "Faild to set SHT31D I2C Slave" << std::endl;
        return false;
    }
    return true;
}

void readLSM6DS33()
{
    
}