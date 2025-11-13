#ifndef ADAFRUIT_SHT31D_H
#define ADAFRUIT_SHT31D_H

#include <cstdint>
#include <string>
#include <semaphore.h>

/**
 * @brief Linux I2C driver for the Sensirion SHT31D temperature/humidity sensor.
 *
 * - No Arduino / Wire / Adafruit_BusIO.
 * - Uses /dev/i2c-* with <linux/i2c-dev.h> and <i2c/smbus.h>.
 * - All I2C operations may be guarded by an external POSIX semaphore.
 *
 * Usage:
 *   sem_t i2cSem;
 *   sem_init(&i2cSem, 0, 1);
 *   Adafruit_SHT31D sht("/dev/i2c-1", 0x44, &i2cSem);
 *   sht.init();
 *   float t, h;
 *   sht.readTemperatureHumidity(t, h);
 */

#define SHT31D_I2C_ADDR_DEFAULT 0x44

class Adafruit_SHT31D {
public:
    Adafruit_SHT31D(const std::string &i2cBusPath,
                    uint8_t           i2cAddress   = SHT31D_I2C_ADDR_DEFAULT,
                    sem_t            *i2cSemaphore = nullptr);

    ~Adafruit_SHT31D();

    /**
     * @brief Initialize the sensor.
     *
     * Opens the device, sets the slave address,
     * and performs a soft-reset.
     *
     * @return true on success, false otherwise.
     */
    bool init();

    /**
     * @brief Read temperature and humidity.
     *
     * Performs a single-shot, high-repeatability measurement.
     *
     * @param[out] temperatureC Temperature in °C.
     * @param[out] humidityRH   Relative humidity in %RH.
     * @return true on success, false otherwise.
     */
    bool readTemperatureHumidity(float &temperatureC, float &humidityRH);

    [[nodiscard]] bool isInitialized() const { return initialized_; }

private:
    Adafruit_SHT31D(const Adafruit_SHT31D &) = delete;
    Adafruit_SHT31D &operator=(const Adafruit_SHT31D &) = delete;

    bool openDevice();
    void closeDevice();
    bool selectSlave();

    bool writeCommand(uint16_t cmd);
    bool readMeasurementRaw(uint16_t &rawTemp, uint16_t &rawHum);
    static uint8_t crc8(const uint8_t *data, int len);

private:
    std::string busPath_;
    uint8_t     address_;
    sem_t      *semaphore_;
    int         fd_;
    bool        initialized_;
};

#endif // ADAFRUIT_SHT31D_H
