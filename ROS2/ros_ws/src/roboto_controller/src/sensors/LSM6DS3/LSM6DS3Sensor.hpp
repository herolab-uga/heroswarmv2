#ifndef LSM6DS3_SENSOR_H
#define LSM6DS3_SENSOR_H

/**
 * @file LSM6DS3Sensor.h
 * @brief Minimal Linux I2C driver for the ST LSM6DS3 IMU (accel + gyro + temp).
 *
 * Uses /dev/i2c-* via <linux/i2c-dev.h> and <i2c/smbus.h>,
 * with an external POSIX semaphore to protect multi-threaded access.
 */

#include <cstdint>
#include <string>
#include <semaphore.h>

extern "C"
{
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

/**
 * @class LSM6DS3Sensor
 * @brief Simple C++ driver for the LSM6DS3 IMU using Linux I2C.
 *
 * Example usage:
 * @code
 * sem_t i2cSem;
 * sem_init(&i2cSem, 0, 1);
 *
 * LSM6DS3Sensor imu("/dev/i2c-1", 0x6A, &i2cSem);
 * if (!imu.init()) {
 *     // handle error
 * }
 *
 * float ax, ay, az;
 * if (imu.readAccel(ax, ay, az)) {
 *     // use accel values in g
 * }
 * @endcode
 */
class LSM6DS3Sensor {
public:
    /**
     * @brief Construct a new LSM6DS3Sensor object.
     *
     * @param i2cBusPath Path to the I2C device (e.g. "/dev/i2c-1").
     * @param i2cAddress 7-bit I2C address of the sensor (0x6A or 0x6B, typically 0x6A).
     * @param i2cSemaphore Pointer to a POSIX semaphore used to guard I2C transactions.
     *                     Pass the same semaphore instance to all sensors sharing the bus.
     */
    LSM6DS3Sensor(const std::string& i2cBusPath,
                  uint8_t           i2cAddress,
                  sem_t*            i2cSemaphore);

    /**
     * @brief Destructor. Closes the I2C file descriptor if open.
     */
    ~LSM6DS3Sensor();

    /**
     * @brief Initialize the sensor.
     *
     * This:
     *  - Opens the I2C device
     *  - Sets the slave address
     *  - Verifies WHO_AM_I
     *  - Configures accel and gyro for:
     *      - ±2 g
     *      - ±245 dps
     *      - 104 Hz ODR
     *      - Block data update and auto-increment
     *
     * @return true on success, false on failure.
     */
    bool init();

    /**
     * @brief Read acceleration in g.
     *
     * @param[out] ax Acceleration X in g.
     * @param[out] ay Acceleration Y in g.
     * @param[out] az Acceleration Z in g.
     * @return true on success, false on failure.
     */
    bool readAccel(float& ax, float& ay, float& az);

    /**
     * @brief Read angular rate in degrees per second.
     *
     * @param[out] gx Gyro X in dps.
     * @param[out] gy Gyro Y in dps.
     * @param[out] gz Gyro Z in dps.
     * @return true on success, false on failure.
     */
    bool readGyro(float& gx, float& gy, float& gz);

    /**
     * @brief Read temperature in degrees Celsius.
     *
     * @param[out] temperature Temperature in °C.
     * @return true on success, false on failure.
     */
    bool readTemp(float& temperature);

    /**
     * @brief Check if the driver has successfully initialized the device.
     *
     * @return true if initialized, false otherwise.
     */
    [[nodiscard]] bool isInitialized() const { return initialized_; }

private:
    // Non-copyable
    LSM6DS3Sensor(const LSM6DS3Sensor&) = delete;
    LSM6DS3Sensor& operator=(const LSM6DS3Sensor&) = delete;

    // Private helper methods
    bool openDevice();
    void closeDevice();
    bool selectSlave();

    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);
    bool readRegisters(uint8_t startReg, uint8_t* buffer, std::size_t length);

    bool configureSensor();

private:
    std::string busPath_;
    uint8_t     address_;
    sem_t*      semaphore_;
    int         fd_;
    bool        initialized_;
};

#endif // LSM6DS3_SENSOR_H
