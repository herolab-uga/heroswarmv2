/*!
 * @file Adafruit_LIS3MDL.h
 *
 * Linux I2C driver for the LIS3MDL magnetometer.
 * - No Arduino, no Adafruit_BusIO, no SPI.
 * - Uses /dev/i2c-* with <linux/i2c-dev.h> and <i2c/smbus.h>.
 * - All I2C operations may be guarded by an external POSIX semaphore.
 */

#ifndef ADAFRUIT_LIS3MDL_H
#define ADAFRUIT_LIS3MDL_H

#include <cstdint>
#include <string>
#include <semaphore.h>

/*=========================================================================
    I2C ADDRESS/BITS
  -----------------------------------------------------------------------*/
// Feather Sense wiring: LIS3MDL at 0x1E
#define LIS3MDL_I2CADDR_DEFAULT (0x1C) ///< Default address on Feather Sense
/*=========================================================================*/

#define LIS3MDL_REG_WHO_AM_I  0x0F  ///< Part ID (0x3D)
#define LIS3MDL_REG_CTRL_REG1 0x20  ///< Control 1
#define LIS3MDL_REG_CTRL_REG2 0x21  ///< Control 2
#define LIS3MDL_REG_CTRL_REG3 0x22  ///< Control 3
#define LIS3MDL_REG_CTRL_REG4 0x23  ///< Control 4
#define LIS3MDL_REG_STATUS    0x27  ///< Status
#define LIS3MDL_REG_OUT_X_L   0x28  ///< X axis low byte (start of XYZ data)
#define LIS3MDL_REG_INT_CFG   0x30  ///< Interrupt config
#define LIS3MDL_REG_INT_THS_L 0x32  ///< Interrupt threshold low

/** The magnetometer ranges (FS bits) */
typedef enum {
  LIS3MDL_RANGE_4_GAUSS  = 0b00,  ///< +/- 4 gauss
  LIS3MDL_RANGE_8_GAUSS  = 0b01,  ///< +/- 8 gauss
  LIS3MDL_RANGE_12_GAUSS = 0b10,  ///< +/- 12 gauss
  LIS3MDL_RANGE_16_GAUSS = 0b11,  ///< +/- 16 gauss
} lis3mdl_range_t;

/** The magnetometer data rate, includes FAST_ODR bit */
typedef enum {
  LIS3MDL_DATARATE_0_625_HZ = 0b0000, ///<  0.625 Hz
  LIS3MDL_DATARATE_1_25_HZ  = 0b0010, ///<  1.25 Hz
  LIS3MDL_DATARATE_2_5_HZ   = 0b0100, ///<  2.5 Hz
  LIS3MDL_DATARATE_5_HZ     = 0b0110, ///<  5 Hz
  LIS3MDL_DATARATE_10_HZ    = 0b1000, ///<  10 Hz
  LIS3MDL_DATARATE_20_HZ    = 0b1010, ///<  20 Hz
  LIS3MDL_DATARATE_40_HZ    = 0b1100, ///<  40 Hz
  LIS3MDL_DATARATE_80_HZ    = 0b1110, ///<  80 Hz
  LIS3MDL_DATARATE_155_HZ   = 0b0001, ///<  155 Hz (FAST_ODR + UHP)
  LIS3MDL_DATARATE_300_HZ   = 0b0011, ///<  300 Hz (FAST_ODR + HP)
  LIS3MDL_DATARATE_560_HZ   = 0b0101, ///<  560 Hz (FAST_ODR + MP)
  LIS3MDL_DATARATE_1000_HZ  = 0b0111, ///<  1000 Hz (FAST_ODR + LP)
} lis3mdl_dataRate_t;

/** The magnetometer performance mode */
typedef enum {
  LIS3MDL_LOWPOWERMODE   = 0b00, ///< Low power mode
  LIS3MDL_MEDIUMMODE     = 0b01, ///< Medium performance mode
  LIS3MDL_HIGHMODE       = 0b10, ///< High performance mode
  LIS3MDL_ULTRAHIGHMODE  = 0b11, ///< Ultra-high performance mode
} lis3mdl_performancemode_t;

/** The magnetometer operation mode */
typedef enum {
  LIS3MDL_CONTINUOUSMODE = 0b00, ///< Continuous conversion
  LIS3MDL_SINGLEMODE     = 0b01, ///< Single-shot conversion
  LIS3MDL_POWERDOWNMODE  = 0b11, ///< Power-down mode
} lis3mdl_operationmode_t;

/**
 * @brief Simple Linux I2C LIS3MDL driver.
 *
 * Constructor takes:
 *  - I2C bus path (e.g., "/dev/i2c-1")
 *  - I2C address (usually 0x1E on Feather Sense)
 *  - Pointer to a POSIX semaphore to guard bus access (may be nullptr)
 *
 * Units:
 *  - readRaw() returns raw 16-bit counts
 *  - readMag() returns values in microtesla (µT)
 */
class Adafruit_LIS3MDL {
public:
  Adafruit_LIS3MDL(const std::string &i2cBusPath,
                   uint8_t           i2cAddress   = LIS3MDL_I2CADDR_DEFAULT,
                   sem_t            *i2cSemaphore = nullptr);

  ~Adafruit_LIS3MDL();

  bool init(lis3mdl_dataRate_t      dataRate = LIS3MDL_DATARATE_155_HZ,
            lis3mdl_range_t         range    = LIS3MDL_RANGE_4_GAUSS,
            lis3mdl_performancemode_t perf   = LIS3MDL_ULTRAHIGHMODE,
            lis3mdl_operationmode_t   opMode = LIS3MDL_CONTINUOUSMODE);

  bool readRaw(int16_t &x, int16_t &y, int16_t &z);
  bool readMag(float &mx, float &my, float &mz);

  bool setDataRate(lis3mdl_dataRate_t dataRate);
  bool getDataRate(lis3mdl_dataRate_t &dataRate);

  bool setRange(lis3mdl_range_t range);
  bool getRange(lis3mdl_range_t &range);

  bool setPerformanceMode(lis3mdl_performancemode_t mode);
  bool setOperationMode(lis3mdl_operationmode_t mode);

  [[nodiscard]] bool isInitialized() const { return initialized_; }

private:
  Adafruit_LIS3MDL(const Adafruit_LIS3MDL &) = delete;
  Adafruit_LIS3MDL &operator=(const Adafruit_LIS3MDL &) = delete;

  bool openDevice();
  void closeDevice();
  bool selectSlave();

  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegister(uint8_t reg, uint8_t &value);
  bool readRegisters(uint8_t startReg, uint8_t *buffer, std::size_t length);

  bool configure(lis3mdl_dataRate_t      dataRate,
                 lis3mdl_range_t         range,
                 lis3mdl_performancemode_t perf,
                 lis3mdl_operationmode_t   opMode);

private:
  std::string busPath_;
  uint8_t     address_;
  sem_t      *semaphore_;
  int         fd_;
  bool        initialized_;

  lis3mdl_range_t rangeBuffered_;
};

#endif // ADAFRUIT_LIS3MDL_H
