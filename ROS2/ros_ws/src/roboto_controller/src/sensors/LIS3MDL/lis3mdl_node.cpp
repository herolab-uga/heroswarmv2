#include "LIS3MDL/lis3mdl_node.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <chrono>

using namespace std::chrono_literals;

// Hard-coded I2C bus & address for Feather Sense LIS3MDL
static constexpr const char* LSM_I2C_BUS  = "/dev/i2c-1";
static constexpr uint8_t     LSM_I2C_ADDR = LIS3MDL_I2CADDR_DEFAULT;

// Helper: map integer Hz param to LIS3MDL data rate enum
static lis3mdl_dataRate_t mapDataRate(int hz_param)
{
    switch (hz_param) {
        case 1:    return LIS3MDL_DATARATE_0_625_HZ; // close enough
        case 2:    return LIS3MDL_DATARATE_1_25_HZ;
        case 3:    return LIS3MDL_DATARATE_2_5_HZ;
        case 5:    return LIS3MDL_DATARATE_5_HZ;
        case 10:   return LIS3MDL_DATARATE_10_HZ;
        case 20:   return LIS3MDL_DATARATE_20_HZ;
        case 40:   return LIS3MDL_DATARATE_40_HZ;
        case 80:   return LIS3MDL_DATARATE_80_HZ;
        case 155:  return LIS3MDL_DATARATE_155_HZ;
        case 300:  return LIS3MDL_DATARATE_300_HZ;
        case 560:  return LIS3MDL_DATARATE_560_HZ;
        case 1000: return LIS3MDL_DATARATE_1000_HZ;
        default:   return LIS3MDL_DATARATE_155_HZ;
    }
}

static lis3mdl_range_t mapRangeGauss(int gauss)
{
    switch (gauss) {
        case 8:   return LIS3MDL_RANGE_8_GAUSS;
        case 12:  return LIS3MDL_RANGE_12_GAUSS;
        case 16:  return LIS3MDL_RANGE_16_GAUSS;
        case 4:
        default:  return LIS3MDL_RANGE_4_GAUSS;
    }
}

LIS3MDLPublisher::LIS3MDLPublisher() :
    Node("lis3mdl"),
    gI2cSemaphore(nullptr),
    gI2cFd(-1),
    rangeGauss(4),
    dataRateHz(155),
    publishRateHz(50.0),
    frameId("mag_link")
{
    // -----------------------------
    // Parameters
    // -----------------------------
    this->declare_parameter<int>("range_gauss", 4);      // 4, 8, 12, 16
    this->declare_parameter<int>("data_rate", 155);      // matches table
    this->declare_parameter<double>("publish_rate", 50); // Hz
    this->declare_parameter<std::string>("frame_id", "mag_link");

    this->get_parameter("range_gauss", rangeGauss);
    this->get_parameter("data_rate", dataRateHz);
    this->get_parameter("publish_rate", publishRateHz);
    this->get_parameter("frame_id", frameId);

    if (rangeGauss != 4 && rangeGauss != 8 && rangeGauss != 12 && rangeGauss != 16) {
        RCLCPP_WARN(this->get_logger(),
                    "Invalid range_gauss=%d, defaulting to 4 gauss",
                    rangeGauss);
        rangeGauss = 4;
    }

    if (publishRateHz <= 0.0) {
        RCLCPP_WARN(this->get_logger(),
                    "Invalid publish_rate=%.2f, defaulting to 50 Hz",
                    publishRateHz);
        publishRateHz = 50.0;
    }

    lis3mdl_range_t     rangeEnum = mapRangeGauss(rangeGauss);
    lis3mdl_dataRate_t  rateEnum  = mapDataRate(dataRateHz);

    RCLCPP_INFO(this->get_logger(),
                "LIS3MDL on %s @ 0x%02X, range=%d gauss, data_rate=%d Hz, publish_rate=%.2f Hz",
                LSM_I2C_BUS, LSM_I2C_ADDR, rangeGauss, dataRateHz, publishRateHz);

    // -----------------------------
    // Semaphore (shared across sensors)
    // -----------------------------
    gI2cSemaphore = sem_open("/i2c_semaphore", O_CREAT, 0777, 1);
    if (gI2cSemaphore == SEM_FAILED)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to open I2C semaphore");
        throw std::runtime_error("Semaphore open failed");
    }

    // -----------------------------
    // I2C FD (for consistency with your other nodes)
    // -----------------------------
    gI2cFd = open(LSM_I2C_BUS, O_RDWR);
    if (gI2cFd < 0)
    {
        RCLCPP_FATAL(this->get_logger(),
                     "Failed to open I2C bus: %s",
                     LSM_I2C_BUS);
        throw std::runtime_error("I2C open failed");
    }

    // -----------------------------
    // Sensor driver
    // -----------------------------
    mag = std::make_unique<Adafruit_LIS3MDL>(
        std::string(LSM_I2C_BUS),
        LSM_I2C_ADDR,
        gI2cSemaphore);

    if (!mag->init(rateEnum,
                   rangeEnum,
                   LIS3MDL_ULTRAHIGHMODE,
                   LIS3MDL_CONTINUOUSMODE))
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize LIS3MDL sensor");
        throw std::runtime_error("LIS3MDL init failed");
    }

    // -----------------------------
    // Publisher & timer
    // -----------------------------
    magPublisher =
        this->create_publisher<sensor_msgs::msg::MagneticField>("magnetic_field", 5);

    auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publishRateHz));

    magTimer = this->create_wall_timer(
        period_ns,
        std::bind(&LIS3MDLPublisher::pubMag, this));

    RCLCPP_INFO(this->get_logger(), "LIS3MDL node ready");
}

void LIS3MDLPublisher::pubMag()
{
    std::lock_guard<std::mutex> lock(magMutex);

    float mx_ut, my_ut, mz_ut;
    if (!mag->readMag(mx_ut, my_ut, mz_ut))
    {
        RCLCPP_WARN(this->get_logger(), "Failed to read LIS3MDL magnetometer");
        return;
    }

    constexpr double UT_TO_T = 1e-6;

    sensor_msgs::msg::MagneticField msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frameId;

    msg.magnetic_field.x = mx_ut * UT_TO_T;
    msg.magnetic_field.y = my_ut * UT_TO_T;
    msg.magnetic_field.z = mz_ut * UT_TO_T;

    for (double &c : msg.magnetic_field_covariance) {
        c = 0.0;
    }

    magPublisher->publish(msg);
}
