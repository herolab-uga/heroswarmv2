#include "SHT31D/sht31d.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sys/ioctl.h>
#include <unistd.h>

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

namespace {

// Commands from SHT31D datasheet
constexpr uint16_t CMD_SOFT_RESET           = 0x30A2;
constexpr uint16_t CMD_CLEAR_STATUS         = 0x3041;
constexpr uint16_t CMD_SINGLE_SHOT_HIGHREP  = 0x2400; // high repeatability, no clock stretching

} // namespace

Adafruit_SHT31D::Adafruit_SHT31D(const std::string &i2cBusPath,
                                 uint8_t           i2cAddress,
                                 sem_t            *i2cSemaphore)
    : busPath_(i2cBusPath),
      address_(i2cAddress),
      semaphore_(i2cSemaphore),
      fd_(-1),
      initialized_(false)
{
}

Adafruit_SHT31D::~Adafruit_SHT31D()
{
    closeDevice();
}

bool Adafruit_SHT31D::init()
{
    if (!openDevice()) {
        return false;
    }

    if (!selectSlave()) {
        closeDevice();
        return false;
    }

    // Soft reset
    if (!writeCommand(CMD_SOFT_RESET)) {
        std::cerr << "SHT31D: soft-reset failed" << std::endl;
        closeDevice();
        return false;
    }
    ::usleep(10000); // 10 ms

    // Clear status (optional but nice)
    writeCommand(CMD_CLEAR_STATUS);
    ::usleep(1000);

    initialized_ = true;
    return true;
}

bool Adafruit_SHT31D::readTemperatureHumidity(float &temperatureC, float &humidityRH)
{
    if (!initialized_) {
        std::cerr << "SHT31D: readTemperatureHumidity() called before init()" << std::endl;
        return false;
    }

    uint16_t rawT = 0;
    uint16_t rawH = 0;

    if (!readMeasurementRaw(rawT, rawH)) {
        return false;
    }

    // From datasheet:
    // T = -45 + 175 * rawT / 65535
    // RH = 100 * rawH / 65535
    temperatureC = -45.0f + 175.0f * (static_cast<float>(rawT) / 65535.0f);
    humidityRH   = 100.0f * (static_cast<float>(rawH) / 65535.0f);

    return true;
}

// -----------------------------------------------------------------------------
// Private helpers
// -----------------------------------------------------------------------------

bool Adafruit_SHT31D::openDevice()
{
    if (fd_ >= 0) {
        return true;
    }

    fd_ = ::open(busPath_.c_str(), O_RDWR);
    if (fd_ < 0) {
        std::cerr << "SHT31D: Failed to open " << busPath_
                  << ": " << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}

void Adafruit_SHT31D::closeDevice()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    initialized_ = false;
}

bool Adafruit_SHT31D::selectSlave()
{
    if (fd_ < 0) {
        return false;
    }

    if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
        std::cerr << "SHT31D: Failed to set I2C slave address 0x"
                  << std::hex << static_cast<int>(address_) << std::dec
                  << ": " << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}

bool Adafruit_SHT31D::writeCommand(uint16_t cmd)
{
    if (fd_ < 0) {
        return false;
    }

    uint8_t buf[2];
    buf[0] = static_cast<uint8_t>(cmd >> 8);
    buf[1] = static_cast<uint8_t>(cmd & 0xFF);

    if (semaphore_) {
        sem_wait(semaphore_);
    }

    ssize_t written = ::write(fd_, buf, 2);

    if (semaphore_) {
        sem_post(semaphore_);
    }

    if (written != 2) {
        std::cerr << "SHT31D: writeCommand(0x"
                  << std::hex << cmd << std::dec
                  << ") failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}

uint8_t Adafruit_SHT31D::crc8(const uint8_t *data, int len)
{
    // CRC-8 with polynomial 0x31, initial value 0xFF (Sensirion standard)
    uint8_t crc = 0xFF;

    for (int i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

bool Adafruit_SHT31D::readMeasurementRaw(uint16_t &rawTemp, uint16_t &rawHum)
{
    // Trigger single-shot measurement
    if (!writeCommand(CMD_SINGLE_SHOT_HIGHREP)) {
        return false;
    }

    // Max conversion time ~15 ms → wait 20 ms to be safe
    ::usleep(20000);

    uint8_t data[6] = {0};

    if (semaphore_) {
        sem_wait(semaphore_);
    }

    ssize_t n = ::read(fd_, data, 6);

    if (semaphore_) {
        sem_post(semaphore_);
    }

    if (n != 6) {
        std::cerr << "SHT31D: readMeasurementRaw - short read (" << n << " bytes)" << std::endl;
        return false;
    }

    // CRC checks
    if (crc8(&data[0], 2) != data[2]) {
        std::cerr << "SHT31D: temperature CRC mismatch" << std::endl;
        return false;
    }

    if (crc8(&data[3], 2) != data[5]) {
        std::cerr << "SHT31D: humidity CRC mismatch" << std::endl;
        return false;
    }

    rawTemp = static_cast<uint16_t>((data[0] << 8) | data[1]);
    rawHum  = static_cast<uint16_t>((data[3] << 8) | data[4]);

    return true;
}
