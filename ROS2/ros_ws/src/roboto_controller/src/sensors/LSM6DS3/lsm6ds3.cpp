#include "LSM6DS3/lsm6ds3.hpp"
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

LSM6DS3Publisher::LSM6DS3Publisher() :
    Node("lsm6ds3")
{
    // =============================
    // Parameters
    // =============================
    this->declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");
    this->declare_parameter<int>("i2c_address", 0x6A);

    this->get_parameter("i2c_bus", i2cBus);
    this->get_parameter("i2c_address", i2cAddress);

    RCLCPP_INFO(this->get_logger(),
                "LSM6DS3 starting on %s @ 0x%02X",
                i2cBus.c_str(), i2cAddress);

    // =============================
    // Semaphore
    // =============================
    gI2cSemaphore = sem_open("/i2c_semaphore", O_CREAT, 0777, 1);
    if (gI2cSemaphore == SEM_FAILED)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to open I2C semaphore");
        throw std::runtime_error("Semaphore open failed");
    }

    // =============================
    // I2C FD
    // =============================
    gI2cFd = open(i2cBus.c_str(), O_RDWR);
    if (gI2cFd < 0)
    {
        RCLCPP_FATAL(this->get_logger(),
                     "Failed to open I2C bus: %s",
                     i2cBus.c_str());
        throw std::runtime_error("I2C open failed");
    }

    // =============================
    // Sensor Driver Object
    // =============================
    imu = std::make_unique<LSM6DS3Sensor>(i2cBus, i2cAddress, gI2cSemaphore);

    if (!imu->init())
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize LSM6DS3 sensor");
        throw std::runtime_error("Sensor init failed");
    }

    // =============================
    // Publishers
    // =============================
    imuPublisher =
        this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 5);

    temperaturePublisher =
        this->create_publisher<sensor_msgs::msg::Temperature>("imu_temperature", 5);

    // =============================
    // Timer
    // =============================
    imuTimer = this->create_wall_timer(
        DEFAULT_IMU_PUB_RATE,
        std::bind(&LSM6DS3Publisher::pubIMU, this));

    RCLCPP_INFO(this->get_logger(), "LSM6DS3 Ready");
}

// ============================================================================
// Publish IMU Data
// ============================================================================

void LSM6DS3Publisher::pubIMU()
{
    std::lock_guard<std::mutex> lock(imuMutex);

    float ax, ay, az;
    float gx, gy, gz;
    float temp;

    bool ok_accel = imu->readAccel(ax, ay, az);
    bool ok_gyro  = imu->readGyro(gx, gy, gz);
    bool ok_temp  = imu->readTemp(temp);

    if (!ok_accel || !ok_gyro)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to read IMU data");
        return;
    }

    // =============================
    // IMU Message
    // =============================
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";

    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;

    msg.angular_velocity.x = gx;
    msg.angular_velocity.y = gy;
    msg.angular_velocity.z = gz;

    msg.orientation_covariance[0] = -1;  // No orientation estimate

    imuPublisher->publish(msg);

    // =============================
    // Temperature Message
    // =============================
    if (ok_temp)
    {
        auto tmsg = sensor_msgs::msg::Temperature();
        tmsg.header = msg.header;
        tmsg.temperature = temp;
        temperaturePublisher->publish(tmsg);
    }
}
