#include "LSM6DS3/LSM6DS3Sensor.hpp"

extern "C"
{
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sys/ioctl.h>
#include <unistd.h>

// -----------------------------------------------------------------------------
// LSM6DS3 Register Definitions (minimal set)
// -----------------------------------------------------------------------------
namespace {

constexpr uint8_t LSM6DS3_REG_FUNC_CFG_ACCESS = 0x01;
constexpr uint8_t LSM6DS3_REG_WHO_AM_I        = 0x0F;
constexpr uint8_t LSM6DS3_REG_CTRL1_XL        = 0x10;
constexpr uint8_t LSM6DS3_REG_CTRL2_G         = 0x11;
constexpr uint8_t LSM6DS3_REG_CTRL3_C         = 0x12;

constexpr uint8_t LSM6DS3_REG_OUT_TEMP_L      = 0x20;
constexpr uint8_t LSM6DS3_REG_OUT_TEMP_H      = 0x21;

constexpr uint8_t LSM6DS3_REG_OUTX_L_G        = 0x22;
constexpr uint8_t LSM6DS3_REG_OUTX_H_G        = 0x23;
constexpr uint8_t LSM6DS3_REG_OUTY_L_G        = 0x24;
constexpr uint8_t LSM6DS3_REG_OUTY_H_G        = 0x25;
constexpr uint8_t LSM6DS3_REG_OUTZ_L_G        = 0x26;
constexpr uint8_t LSM6DS3_REG_OUTZ_H_G        = 0x27;

constexpr uint8_t LSM6DS3_REG_OUTX_L_XL       = 0x28;
constexpr uint8_t LSM6DS3_REG_OUTX_H_XL       = 0x29;
constexpr uint8_t LSM6DS3_REG_OUTY_L_XL       = 0x2A;
constexpr uint8_t LSM6DS3_REG_OUTY_H_XL       = 0x2B;
constexpr uint8_t LSM6DS3_REG_OUTZ_L_XL       = 0x2C;
constexpr uint8_t LSM6DS3_REG_OUTZ_H_XL       = 0x2D;

// Expected WHO_AM_I value
constexpr uint8_t LSM6DS3_WHO_AM_I_VALUE      = 0x6A;

// Sensitivity constants for FS=±2g and FS=±245dps
// From datasheet:
//  Accel: 0.061 mg/LSB -> 0.000061 g/LSB
//  Gyro:  8.75 mdps/LSB -> 0.00875 dps/LSB
constexpr float LSM6DS3_ACCEL_SENS_2G_G_PER_LSB = 0.000061f;
constexpr float LSM6DS3_GYRO_SENS_245DPS_DPS_PER_LSB = 0.00875f;

} // namespace

// -----------------------------------------------------------------------------
// Constructor / Destructor
// -----------------------------------------------------------------------------

LSM6DS3Sensor::LSM6DS3Sensor(const std::string& i2cBusPath,
                             uint8_t           i2cAddress,
                             sem_t*            i2cSemaphore)
    : busPath_(i2cBusPath),
      address_(i2cAddress),
      semaphore_(i2cSemaphore),
      fd_(-1),
      initialized_(false)
{
}

LSM6DS3Sensor::~LSM6DS3Sensor()
{
    closeDevice();
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

bool LSM6DS3Sensor::init()
{
    if (!openDevice()) {
        return false;
    }

    if (!selectSlave()) {
        closeDevice();
        return false;
    }

    // Check WHO_AM_I
    uint8_t whoAmI = 0;
    if (!readRegister(LSM6DS3_REG_WHO_AM_I, whoAmI)) {
        std::cerr << "LSM6DS3: Failed to read WHO_AM_I" << std::endl;
        closeDevice();
        return false;
    }

    if (whoAmI != LSM6DS3_WHO_AM_I_VALUE) {
        std::cerr << "LSM6DS3: Unexpected WHO_AM_I value 0x"
                  << std::hex << static_cast<int>(whoAmI)
                  << ", expected 0x" << static_cast<int>(LSM6DS3_WHO_AM_I_VALUE)
                  << std::dec << std::endl;
        closeDevice();
        return false;
    }

    if (!configureSensor()) {
        std::cerr << "LSM6DS3: Failed to configure sensor" << std::endl;
        closeDevice();
        return false;
    }

    initialized_ = true;
    return true;
}

bool LSM6DS3Sensor::readAccel(float& ax, float& ay, float& az)
{
    if (!initialized_) {
        std::cerr << "LSM6DS3: readAccel() called before init()" << std::endl;
        return false;
    }

    uint8_t buffer[6] = {0};
    if (!readRegisters(LSM6DS3_REG_OUTX_L_XL, buffer, sizeof(buffer))) {
        std::cerr << "LSM6DS3: Failed to read accel registers" << std::endl;
        return false;
    }

    int16_t rawX = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
    int16_t rawY = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
    int16_t rawZ = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);

    ax = rawX * LSM6DS3_ACCEL_SENS_2G_G_PER_LSB;
    ay = rawY * LSM6DS3_ACCEL_SENS_2G_G_PER_LSB;
    az = rawZ * LSM6DS3_ACCEL_SENS_2G_G_PER_LSB;

    return true;
}

bool LSM6DS3Sensor::readGyro(float& gx, float& gy, float& gz)
{
    if (!initialized_) {
        std::cerr << "LSM6DS3: readGyro() called before init()" << std::endl;
        return false;
    }

    uint8_t buffer[6] = {0};
    if (!readRegisters(LSM6DS3_REG_OUTX_L_G, buffer, sizeof(buffer))) {
        std::cerr << "LSM6DS3: Failed to read gyro registers" << std::endl;
        return false;
    }

    int16_t rawX = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
    int16_t rawY = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
    int16_t rawZ = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);

    gx = rawX * LSM6DS3_GYRO_SENS_245DPS_DPS_PER_LSB;
    gy = rawY * LSM6DS3_GYRO_SENS_245DPS_DPS_PER_LSB;
    gz = rawZ * LSM6DS3_GYRO_SENS_245DPS_DPS_PER_LSB;

    return true;
}

bool LSM6DS3Sensor::readTemp(float& temperature)
{
    if (!initialized_) {
        std::cerr << "LSM6DS3: readTemp() called before init()" << std::endl;
        return false;
    }

    uint8_t buffer[2] = {0};
    if (!readRegisters(LSM6DS3_REG_OUT_TEMP_L, buffer, sizeof(buffer))) {
        std::cerr << "LSM6DS3: Failed to read temperature registers" << std::endl;
        return false;
    }

    int16_t rawTemp = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);

    // From datasheet: TEMP_degC = 25 + (TEMP_OUT / 16)
    temperature = 25.0f + (static_cast<float>(rawTemp) / 16.0f);

    return true;
}

// -----------------------------------------------------------------------------
// Private helpers
// -----------------------------------------------------------------------------

bool LSM6DS3Sensor::openDevice()
{
    if (fd_ >= 0) {
        // Already open
        return true;
    }

    fd_ = ::open(busPath_.c_str(), O_RDWR);
    if (fd_ < 0) {
        std::cerr << "LSM6DS3: Failed to open " << busPath_
                  << ": " << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}

void LSM6DS3Sensor::closeDevice()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    initialized_ = false;
}

bool LSM6DS3Sensor::selectSlave()
{
    if (fd_ < 0) {
        return false;
    }

    if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
        std::cerr << "LSM6DS3: Failed to set I2C slave address 0x"
                  << std::hex << static_cast<int>(address_) << std::dec
                  << ": " << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}

bool LSM6DS3Sensor::writeRegister(uint8_t reg, uint8_t value)
{
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
        std::cerr << "LSM6DS3: i2c_smbus_write_byte_data failed for reg 0x"
                  << std::hex << static_cast<int>(reg) << std::dec
                  << ": " << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}

bool LSM6DS3Sensor::readRegister(uint8_t reg, uint8_t& value)
{
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
        std::cerr << "LSM6DS3: i2c_smbus_read_byte_data failed for reg 0x"
                  << std::hex << static_cast<int>(reg) << std::dec
                  << ": " << std::strerror(errno) << std::endl;
        return false;
    }

    value = static_cast<uint8_t>(rc);
    return true;
}

bool LSM6DS3Sensor::readRegisters(uint8_t startReg, uint8_t* buffer, std::size_t length)
{
    if (fd_ < 0 || buffer == nullptr || length == 0) {
        return false;
    }

    if (semaphore_) {
        sem_wait(semaphore_);
    }

    // Use I2C block read (auto-increment must be enabled in CTRL3_C)
    int rc = i2c_smbus_read_i2c_block_data(fd_, startReg, static_cast<uint8_t>(length), buffer);

    if (semaphore_) {
        sem_post(semaphore_);
    }

    if (rc < 0) {
        std::cerr << "LSM6DS3: i2c_smbus_read_i2c_block_data failed from reg 0x"
                  << std::hex << static_cast<int>(startReg) << std::dec
                  << ": " << std::strerror(errno) << std::endl;
        return false;
    }

    if (static_cast<std::size_t>(rc) != length) {
        std::cerr << "LSM6DS3: readRegisters length mismatch, expected "
                  << length << ", got " << rc << std::endl;
        return false;
    }

    return true;
}

bool LSM6DS3Sensor::configureSensor()
{
    // CTRL3_C:
    //  - BDU (bit 6)   = 1 (Block Data Update)
    //  - IF_INC (bit2) = 1 (Auto-increment register address)
    uint8_t ctrl3_c = 0x44; // 0b01000100
    if (!writeRegister(LSM6DS3_REG_CTRL3_C, ctrl3_c)) {
        return false;
    }

    // CTRL1_XL (Accel):
    //  ODR_XL[3:0] = 0b0100 -> 104 Hz  (bits 7:4)
    //  FS_XL[1:0]  = 0b00   -> ±2 g    (bits 3:2)
    //  BW_XL[1:0]  = 0b11   -> 400 Hz anti-alias (bits 1:0)
    uint8_t ctrl1_xl = 0x40 | 0x03; // 0x43
    if (!writeRegister(LSM6DS3_REG_CTRL1_XL, ctrl1_xl)) {
        return false;
    }

    // CTRL2_G (Gyro):
    //  ODR_G[3:0] = 0b0100 -> 104 Hz  (bits 7:4)
    //  FS_G[1:0]  = 0b00   -> ±245 dps (bits 3:2)
    uint8_t ctrl2_g = 0x40; // 0b01000000
    if (!writeRegister(LSM6DS3_REG_CTRL2_G, ctrl2_g)) {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
// Standalone helper functions (reusable for other sensors)
// -----------------------------------------------------------------------------

/**
 * @brief Write an 8-bit value to an 8-bit register over I2C.
 *
 * @param fd   Open I2C file descriptor.
 * @param reg  Register address.
 * @param value Value to write.
 * @param sem  Pointer to semaphore used to guard bus access (may be nullptr).
 * @return 0 on success, negative errno-style value on failure.
 */
int i2c_write_reg8(int fd, uint8_t reg, uint8_t value, sem_t* sem)
{
    if (fd < 0) {
        return -1;
    }

    if (sem) {
        sem_wait(sem);
    }

    int rc = i2c_smbus_write_byte_data(fd, reg, value);

    if (sem) {
        sem_post(sem);
    }

    if (rc < 0) {
        std::cerr << "i2c_write_reg8: failed for reg 0x"
                  << std::hex << static_cast<int>(reg) << std::dec
                  << ": " << std::strerror(errno) << std::endl;
    }

    return rc;
}

/**
 * @brief Read an 8-bit value from an 8-bit register over I2C.
 *
 * @param fd   Open I2C file descriptor.
 * @param reg  Register address.
 * @param sem  Pointer to semaphore used to guard bus access (may be nullptr).
 * @return Register value (0-255) on success, negative errno-style value on failure.
 */
int i2c_read_reg8(int fd, uint8_t reg, sem_t* sem)
{
    if (fd < 0) {
        return -1;
    }

    if (sem) {
        sem_wait(sem);
    }

    int rc = i2c_smbus_read_byte_data(fd, reg);

    if (sem) {
        sem_post(sem);
    }

    if (rc < 0) {
        std::cerr << "i2c_read_reg8: failed for reg 0x"
                  << std::hex << static_cast<int>(reg) << std::dec
                  << ": " << std::strerror(errno) << std::endl;
    }

    return rc;
}

/**
 * @brief Read a block of data starting at an 8-bit register over I2C.
 *
 * @param fd    Open I2C file descriptor.
 * @param reg   Start register address.
 * @param buf   Destination buffer.
 * @param len   Number of bytes to read.
 * @param sem   Pointer to semaphore used to guard bus access (may be nullptr).
 * @return Number of bytes read on success, negative errno-style value on failure.
 */
int i2c_read_block(int fd, uint8_t reg, uint8_t* buf, uint16_t len, sem_t* sem)
{
    if (fd < 0 || buf == nullptr || len == 0) {
        return -1;
    }

    if (sem) {
        sem_wait(sem);
    }

    int rc = i2c_smbus_read_i2c_block_data(fd, reg, static_cast<uint8_t>(len), buf);

    if (sem) {
        sem_post(sem);
    }

    if (rc < 0) {
        std::cerr << "i2c_read_block: failed from reg 0x"
                  << std::hex << static_cast<int>(reg) << std::dec
                  << ": " << std::strerror(errno) << std::endl;
    }

    return rc;
}
