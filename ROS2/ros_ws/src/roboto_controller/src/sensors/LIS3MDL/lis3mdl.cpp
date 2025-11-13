#include "LIS3MDL/lis3mdl.hpp"

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

// WHO_AM_I expected value for LIS3MDL
constexpr uint8_t WHO_AM_I_EXPECTED = 0x3D;

// Scale factors (LSB per gauss) from Adafruit’s LIS3MDL implementation:
//  16 gauss -> 1711 LSB/gauss
//  12 gauss -> 2281 LSB/gauss
//   8 gauss -> 3421 LSB/gauss
//   4 gauss -> 6842 LSB/gauss
inline float scale_lsb_per_gauss(lis3mdl_range_t range) {
  switch (range) {
  case LIS3MDL_RANGE_16_GAUSS: return 1711.0f;
  case LIS3MDL_RANGE_12_GAUSS: return 2281.0f;
  case LIS3MDL_RANGE_8_GAUSS:  return 3421.0f;
  case LIS3MDL_RANGE_4_GAUSS:
  default:                     return 6842.0f;
  }
}

} // namespace

Adafruit_LIS3MDL::Adafruit_LIS3MDL(const std::string &i2cBusPath,
                                   uint8_t           i2cAddress,
                                   sem_t            *i2cSemaphore)
    : busPath_(i2cBusPath),
      address_(i2cAddress),
      semaphore_(i2cSemaphore),
      fd_(-1),
      initialized_(false),
      rangeBuffered_(LIS3MDL_RANGE_4_GAUSS) {}

Adafruit_LIS3MDL::~Adafruit_LIS3MDL() {
  closeDevice();
}

bool Adafruit_LIS3MDL::init(lis3mdl_dataRate_t      dataRate,
                            lis3mdl_range_t         range,
                            lis3mdl_performancemode_t perf,
                            lis3mdl_operationmode_t   opMode)
{
  if (!openDevice()) {
    return false;
  }

  if (!selectSlave()) {
    closeDevice();
    return false;
  }

  uint8_t who = 0;
  if (!readRegister(LIS3MDL_REG_WHO_AM_I, who)) {
    std::cerr << "LIS3MDL: Failed to read WHO_AM_I" << std::endl;
    closeDevice();
    return false;
  }

  if (who != WHO_AM_I_EXPECTED) {
    std::cerr << "LIS3MDL: Unexpected WHO_AM_I 0x"
              << std::hex << static_cast<int>(who)
              << ", expected 0x" << static_cast<int>(WHO_AM_I_EXPECTED)
              << std::dec << std::endl;
    closeDevice();
    return false;
  }

  if (!configure(dataRate, range, perf, opMode)) {
    std::cerr << "LIS3MDL: Failed to configure sensor" << std::endl;
    closeDevice();
    return false;
  }

  initialized_ = true;
  return true;
}

bool Adafruit_LIS3MDL::readRaw(int16_t &x, int16_t &y, int16_t &z) {
  if (!initialized_) {
    std::cerr << "LIS3MDL: readRaw() called before init()" << std::endl;
    return false;
  }

  uint8_t buffer[6] = {0};
  if (!readRegisters(LIS3MDL_REG_OUT_X_L, buffer, sizeof(buffer))) {
    std::cerr << "LIS3MDL: Failed to read XYZ registers" << std::endl;
    return false;
  }

  x = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
  y = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
  z = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);

  return true;
}

bool Adafruit_LIS3MDL::readMag(float &mx, float &my, float &mz) {
  int16_t x, y, z;
  if (!readRaw(x, y, z)) {
    return false;
  }

  float scale = scale_lsb_per_gauss(rangeBuffered_);

  float x_gauss = static_cast<float>(x) / scale;
  float y_gauss = static_cast<float>(y) / scale;
  float z_gauss = static_cast<float>(z) / scale;

  constexpr float GAUSS_TO_UT = 100.0f; // 1 gauss = 100 µT
  mx = x_gauss * GAUSS_TO_UT;
  my = y_gauss * GAUSS_TO_UT;
  mz = z_gauss * GAUSS_TO_UT;

  return true;
}

bool Adafruit_LIS3MDL::setDataRate(lis3mdl_dataRate_t dataRate) {
  uint8_t reg = 0;
  if (!readRegister(LIS3MDL_REG_CTRL_REG1, reg)) {
    return false;
  }

  reg &= ~(0x0Fu << 1); // clear DR bits [4:1]
  reg |= (static_cast<uint8_t>(dataRate) & 0x0Fu) << 1;

  return writeRegister(LIS3MDL_REG_CTRL_REG1, reg);
}

bool Adafruit_LIS3MDL::getDataRate(lis3mdl_dataRate_t &dataRate) {
  uint8_t reg = 0;
  if (!readRegister(LIS3MDL_REG_CTRL_REG1, reg)) {
    return false;
  }

  uint8_t dr_bits = (reg >> 1) & 0x0Fu;
  dataRate = static_cast<lis3mdl_dataRate_t>(dr_bits);
  return true;
}

bool Adafruit_LIS3MDL::setRange(lis3mdl_range_t range) {
  uint8_t reg = 0;
  if (!readRegister(LIS3MDL_REG_CTRL_REG2, reg)) {
    return false;
  }

  reg &= ~(0x03u << 5); // FS bits [6:5]
  reg |= (static_cast<uint8_t>(range) & 0x03u) << 5;

  if (!writeRegister(LIS3MDL_REG_CTRL_REG2, reg)) {
    return false;
  }

  rangeBuffered_ = range;
  return true;
}

bool Adafruit_LIS3MDL::getRange(lis3mdl_range_t &range) {
  uint8_t reg = 0;
  if (!readRegister(LIS3MDL_REG_CTRL_REG2, reg)) {
    return false;
  }

  uint8_t fs_bits = (reg >> 5) & 0x03u;
  range = static_cast<lis3mdl_range_t>(fs_bits);
  rangeBuffered_ = range;
  return true;
}

bool Adafruit_LIS3MDL::setPerformanceMode(lis3mdl_performancemode_t mode) {
  uint8_t reg1 = 0;
  if (!readRegister(LIS3MDL_REG_CTRL_REG1, reg1)) {
    return false;
  }
  reg1 &= ~(0x03u << 5);
  reg1 |= (static_cast<uint8_t>(mode) & 0x03u) << 5;
  if (!writeRegister(LIS3MDL_REG_CTRL_REG1, reg1)) {
    return false;
  }

  uint8_t reg4 = 0;
  if (!readRegister(LIS3MDL_REG_CTRL_REG4, reg4)) {
    return false;
  }
  reg4 &= ~(0x03u << 2);
  reg4 |= (static_cast<uint8_t>(mode) & 0x03u) << 2;
  if (!writeRegister(LIS3MDL_REG_CTRL_REG4, reg4)) {
    return false;
  }

  return true;
}

bool Adafruit_LIS3MDL::setOperationMode(lis3mdl_operationmode_t mode) {
  uint8_t reg3 = 0;
  if (!readRegister(LIS3MDL_REG_CTRL_REG3, reg3)) {
    return false;
  }

  reg3 &= ~0x03u;
  reg3 |= (static_cast<uint8_t>(mode) & 0x03u);

  return writeRegister(LIS3MDL_REG_CTRL_REG3, reg3);
}

// -----------------------------------------------------------------------------
// Private helpers
// -----------------------------------------------------------------------------

bool Adafruit_LIS3MDL::openDevice() {
  if (fd_ >= 0) {
    return true;
  }

  fd_ = ::open(busPath_.c_str(), O_RDWR);
  if (fd_ < 0) {
    std::cerr << "LIS3MDL: Failed to open " << busPath_
              << ": " << std::strerror(errno) << std::endl;
    return false;
  }

  return true;
}

void Adafruit_LIS3MDL::closeDevice() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  initialized_ = false;
}

bool Adafruit_LIS3MDL::selectSlave() {
  if (fd_ < 0) {
    return false;
  }

  if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
    std::cerr << "LIS3MDL: Failed to set I2C slave address 0x"
              << std::hex << static_cast<int>(address_) << std::dec
              << ": " << std::strerror(errno) << std::endl;
    return false;
  }

  return true;
}

bool Adafruit_LIS3MDL::writeRegister(uint8_t reg, uint8_t value) {
  if (fd_ < 0) {
    return false;
  }

  if (semaphore_) {
    sem_wait(semaphore_);
  }

  int rc = i2c_smbus_write_byte_data(fd_, reg, value);

  if (semaphore_) {
    sem_post(semaphore_);
  }

  if (rc < 0) {
    std::cerr << "LIS3MDL: writeRegister(0x"
              << std::hex << static_cast<int>(reg) << std::dec
              << ") failed: " << std::strerror(errno) << std::endl;
    return false;
  }

  return true;
}

bool Adafruit_LIS3MDL::readRegister(uint8_t reg, uint8_t &value) {
  if (fd_ < 0) {
    return false;
  }

  if (semaphore_) {
    sem_wait(semaphore_);
  }

  int rc = i2c_smbus_read_byte_data(fd_, reg);

  if (semaphore_) {
    sem_post(semaphore_);
  }

  if (rc < 0) {
    std::cerr << "LIS3MDL: readRegister(0x"
              << std::hex << static_cast<int>(reg) << std::dec
              << ") failed: " << std::strerror(errno) << std::endl;
    return false;
  }

  value = static_cast<uint8_t>(rc);
  return true;
}

bool Adafruit_LIS3MDL::readRegisters(uint8_t startReg,
                                     uint8_t *buffer,
                                     std::size_t length) {
  if (fd_ < 0 || buffer == nullptr || length == 0) {
    return false;
  }

  if (semaphore_) {
    sem_wait(semaphore_);
  }

  int rc = i2c_smbus_read_i2c_block_data(fd_, startReg,
                                         static_cast<uint8_t>(length), buffer);

  if (semaphore_) {
    sem_post(semaphore_);
  }

  if (rc < 0) {
    std::cerr << "LIS3MDL: readRegisters from 0x"
              << std::hex << static_cast<int>(startReg) << std::dec
              << " failed: " << std::strerror(errno) << std::endl;
    return false;
  }

  if (static_cast<std::size_t>(rc) != length) {
    std::cerr << "LIS3MDL: readRegisters length mismatch, expected "
              << length << ", got " << rc << std::endl;
    return false;
  }

  return true;
}

bool Adafruit_LIS3MDL::configure(lis3mdl_dataRate_t      dataRate,
                                 lis3mdl_range_t         range,
                                 lis3mdl_performancemode_t perf,
                                 lis3mdl_operationmode_t   opMode)
{
  // Soft reset via CTRL_REG2 bit2
  uint8_t reg2 = 0;
  if (!readRegister(LIS3MDL_REG_CTRL_REG2, reg2)) {
    return false;
  }
  reg2 |= (1u << 2);
  if (!writeRegister(LIS3MDL_REG_CTRL_REG2, reg2)) {
    return false;
  }
  ::usleep(10 * 1000);

  if (!setPerformanceMode(perf)) {
    return false;
  }
  if (!setDataRate(dataRate)) {
    return false;
  }
  if (!setRange(range)) {
    return false;
  }
  if (!setOperationMode(opMode)) {
    return false;
  }

  return true;
}
