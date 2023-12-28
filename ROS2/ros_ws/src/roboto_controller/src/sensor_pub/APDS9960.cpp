#include "includes/sensor_pub.hpp"

#define ENABLEREG 0x80
#define CTRLREGONE 0x8F

#define PROXDATA 0x9C
#define CLEARREG 0x94
#define REDREG 0x96
#define GREENREG 0x98
#define BLUEREG 0x9A

bool setupAPDS9960()
{

    std::cout << "Starting APDS9960 Setup" << std::endl;
    if (ioctl(i2cFd, I2C_SLAVE, APDS9960) < 0)
    {
        return 0;
    }
    // Enable Proximity and Color
    i2c_smbus_write_byte_data(i2cFd, ENABLEREG, 0x07);

    // Control Register 1 - LED Drive level Prox gain and color gain
    i2c_smbus_write_byte_data(i2cFd, CTRLREGONE, 0x0B);

    return true;
}

void readColor()
{
    // The color data is 16 bit
    uint16_t clear = i2c_smbus_read_word_data(i2cFd, CLEARREG);
    uint16_t red = i2c_smbus_read_word_data(i2cFd, REDREG);
    uint16_t green = i2c_smbus_read_word_data(i2cFd, GREENREG);
    uint16_t blue = i2c_smbus_read_word_data(i2cFd, BLUEREG);

    lightMutex.lock();
    rgbw[0] = red;
    rgbw[0] = green;
    rgbw[0] = blue;
    rgbw[0] = clear;
    lightMutex.unlock();
}

void readProx()
{
    uint8_t readProx = i2c_smbus_read_byte_data(i2cFd, PROXDATA);
    proximityMutex.lock();
    prox = readProx;
    proximityMutex.unlock();
}
