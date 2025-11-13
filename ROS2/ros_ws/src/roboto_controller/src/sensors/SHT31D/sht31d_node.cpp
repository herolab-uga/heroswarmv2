#include "SHT31D/sht31d_node.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h>


#define DEFAULT_PUB_RATE std::chrono::milliseconds(16) /* The default publishing rate for sensor data is 60 hz*/

SHT31DPublisher::SHT31DPublisher() :
    Node("sht31d"),
    gI2cSemaphore(nullptr),
    gI2cFd(-1)
{
    RCLCPP_INFO(this->get_logger(), "Starting SHT31DPublisher node");

    // -----------------------------
    // Semaphore (shared across sensors)
    // -----------------------------
    gI2cSemaphore = sem_open("/i2c_semaphore", O_CREAT, 0777, 1);
    if (gI2cSemaphore == SEM_FAILED)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to open I2C semaphore");
        throw std::runtime_error("SHT31D semaphore open failed");
    }

    // -----------------------------
    // I2C FD (matches pattern from other nodes)
    // -----------------------------
    const char* bus = "/dev/i2c-1";
    gI2cFd = open(bus, O_RDWR);
    if (gI2cFd < 0)
    {
        RCLCPP_FATAL(this->get_logger(),
                     "Failed to open I2C bus: %s",
                     bus);
        throw std::runtime_error("SHT31D I2C open failed");
    }

    // -----------------------------
    // Sensor driver
    // -----------------------------
    sht = std::make_unique<Adafruit_SHT31D>(
        std::string(bus),
        SHT31D_I2C_ADDR_DEFAULT,
        gI2cSemaphore);

    if (!sht->init())
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize SHT31D sensor");
        throw std::runtime_error("SHT31D init failed");
    }

    // -----------------------------
    // Publishers & timer
    // -----------------------------
    temperaturePublisher =
        this->create_publisher<std_msgs::msg::Float64>("/temperature", 5);

    humidityPublisher =
        this->create_publisher<std_msgs::msg::Float64>("/humidity", 5);

    shtTimer = this->create_wall_timer(
        DEFAULT_PUB_RATE,
        std::bind(&SHT31DPublisher::pubSHT31D, this));

    RCLCPP_INFO(this->get_logger(), "SHT31DPublisher ready");
}

void SHT31DPublisher::pubSHT31D()
{
    std::lock_guard<std::mutex> lock(shtMutex);

    float temperatureC = 0.0f;
    float humidityRH   = 0.0f;

    if (!sht->readTemperatureHumidity(temperatureC, humidityRH))
    {
        RCLCPP_WARN(this->get_logger(), "Failed to read SHT31D");
        return;
    }

    std_msgs::msg::Float64 tempMsg;
    std_msgs::msg::Float64 humMsg;

    tempMsg.data = static_cast<double>(temperatureC);
    humMsg.data  = static_cast<double>(humidityRH);

    temperaturePublisher->publish(tempMsg);
    humidityPublisher->publish(humMsg);
}
