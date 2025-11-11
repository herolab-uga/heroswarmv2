extern "C"
{
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#include "APDS9960/APDS9960.hpp"


#define APDS9960 0x39

#define ENABLEREG 0x80
#define CTRLREGONE 0x8F

#define PROXDATA 0x9C
#define CLEARREG 0x94
#define REDREG 0x96
#define GREENREG 0x98
#define BLUEREG 0x9A

#define DEFAULT_PUB_RATE std::chrono::milliseconds(16) /* The default publishing rate for sensor data is 60 hz*/

bool APDS9960Publisher::setupAPDS9960()
{  
    sem_wait(gI2cSemaphore);
    std::cout << "Starting APDS9960 Setup" << std::endl;
    if (ioctl(this->gI2cFd, I2C_SLAVE, APDS9960) < 0)
    {
        return 0;
    }
    // Enable Proximity and Color
    i2c_smbus_write_byte_data(this->gI2cFd, ENABLEREG, 0x07);

    // Control Register 1 - LED Drive level Prox gain and color gain
    i2c_smbus_write_byte_data(this->gI2cFd, CTRLREGONE, 0x0B);

    sem_post(gI2cSemaphore);

    return true;
}

bool APDS9960Publisher::readColor()
{
    sem_wait(gI2cSemaphore);

    if (ioctl(this->gI2cFd, I2C_SLAVE, APDS9960) < 0)
    {
        std::cout << "Faild to set I2C Slave" << std::endl;
        return false;
    }

    this->rgbw[0] = (uint16_t) i2c_smbus_read_word_data(this->gI2cFd, REDREG);
    this->rgbw[1] = (uint16_t) i2c_smbus_read_word_data(this->gI2cFd, GREENREG);
    this->rgbw[2] = (uint16_t) i2c_smbus_read_word_data(this->gI2cFd, BLUEREG);
    this->rgbw[3] = (uint16_t) i2c_smbus_read_word_data(this->gI2cFd, CLEARREG);

    sem_post(gI2cSemaphore);

    return true;
}

uint8_t APDS9960Publisher::readProx()
{
    sem_wait(gI2cSemaphore);

    if (ioctl(this->gI2cFd, I2C_SLAVE, APDS9960) < 0)
    {
        std::cout << "Faild to set I2C Slave" << std::endl;
        return false;
    }

    uint8_t readProx = i2c_smbus_read_byte_data(this->gI2cFd, PROXDATA);

    sem_post(gI2cSemaphore);
    return readProx;
}

void APDS9960Publisher::pubAPDS9960()
{
    
    readColor();

    auto proxMsg = std_msgs::msg::Int16();

    proxMsg.data = readProx();

    this->proximityPublisher->publish(proxMsg);

    auto lightMsg = robot_msgs::msg::Light();
    // Insert RGBW values into message
    lightMsg.rgbw.push_back(this->rgbw[0]);
    lightMsg.rgbw.push_back(this->rgbw[1]);
    lightMsg.rgbw.push_back(this->rgbw[2]);
    lightMsg.rgbw.push_back(this->rgbw[3]);

    lightMsg.gesture = this->gesture;

    this->lightPublisher->publish(lightMsg);
}

APDS9960Publisher::APDS9960Publisher():Node("apds9960")
{
    // TODO: Change the file premissions
    this->gI2cSemaphore = sem_open("/i2c_semaphore", O_CREAT, 777, 1);

    this->gI2cFd = open("/dev/i2c-1", O_RDWR);

    setupAPDS9960();

    // need to add parameters to know what sensors to turn on and subscirber so you can toggle them in real time
    // Create the publishers
    this->lightPublisher = this->create_publisher<robot_msgs::msg::Light>("light", 5);
    this->proximityPublisher = this->create_publisher<std_msgs::msg::Int16>("proximity", 5);

    // Create the timers
    this->lightTimer = this->create_wall_timer(DEFAULT_PUB_RATE, std::bind(&APDS9960Publisher::pubAPDS9960, this));
    std::cout << "Ready" << std::endl;
}