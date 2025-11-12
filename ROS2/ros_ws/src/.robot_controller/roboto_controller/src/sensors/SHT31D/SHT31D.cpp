#include "includes/sensor_pub.hpp"
#include "std_msgs/msg/float64.hpp"

#define SHT31D 0x44

class SHT31D : public rclcpp::Node
{

    private:

        float temp;
        float humidity;
        std::mutex SHT31DMutex;

        bool SensorPublisher::setupSHT31D()
        {
            std::cout << "Starting SHT31D Setup" << std::endl;
            if (ioctl(i2cFd, I2C_SLAVE, SHT31D) < 0)
            {
                std::cout << "Faild to set SHT31D I2C Slave" << std::endl;
                return false;
            }
            // High repeatability with clock stretching
            i2c_smbus_write_word_data(i2cFd, 0x2C, 0x06);

            // High repeatability 10 measurements per second
            i2c_smbus_write_word_data(i2cFd, 0x27, 0x37);

            return true;
        }

        bool SensorPublisher::readSHT31D()
        {

            if (ioctl(i2cFd, I2C_SLAVE, SHT31D) < 0)
            {
                std::cout << "Faild to set SHT31D I2C Slave" << std::endl;
                return false;
            }

            // Request data
            i2c_smbus_write_word_data(i2cFd, 0xE0, 0x00);

            uint8_t temp_h = i2c_smbus_read_byte(i2cFd);
            uint8_t temp_l = i2c_smbus_read_byte(i2cFd);
            uint8_t humidity_h = i2c_smbus_read_byte(i2cFd);
            uint8_t humidity_l = i2c_smbus_read_byte(i2cFd);

            SHT31DMutex.lock();
            humidity = 100.0 * (float)((humidity_h << 8) | humidity_l) / 65535.0;
            temp = -45.0 + 175 * (float)((temp_h << 8) | temp_l) / 65535.0;
            SHT31DMutex.unlock();

            // Request Status Register
            i2c_smbus_write_word_data(i2cFd, 0xF3, 0x2D);
            std::cout << "Upper: " << std::hex << (i2c_smbus_read_byte(i2cFd)) << std::endl;
            std::cout << "Lower: " << std::hex << (i2c_smbus_read_byte(i2cFd)) << std::endl;
            return true;
        }

        void SHT31D::pubhumidity()
        {
            readSHT31D();
        }

        SHT31DPublisher::SHT31D():Node("sht31d")
        {
            setupSHT31D();
            humidityPublisher = this->create_publisher<robot_msgs::msg::Float64>("/humidity", 5);
            humidityTimer = this->create_wall_timer(DEFAULT_PUB_RATE, std::bind(&BMP280Publisher::pubhumidity, this));
        }
}

int main(int argc, char *argv[])
{

	std::cout << "Spinning SHT31D ROS Node" << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SHT31DPublisher>());
	rclcpp::shutdown();
	return 0;
}