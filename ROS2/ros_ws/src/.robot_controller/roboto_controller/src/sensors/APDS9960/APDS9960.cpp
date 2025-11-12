#include <fcntl.h>           /* For O_* constants */
#include <sys/stat.h>        /* For mode constants */
#include <semaphore.h>
#include <sys/ioctl.h>

extern "C"
{
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/light.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"


#define APDS9960 0x39

#define ENABLEREG 0x80
#define CTRLREGONE 0x8F

#define PROXDATA 0x9C
#define CLEARREG 0x94
#define REDREG 0x96
#define GREENREG 0x98
#define BLUEREG 0x9A

#define DEFAULT_PUB_RATE std::chrono::milliseconds(16) /* The default publishing rate for sensor data is 60 hz*/


class APDS9960Publisher : public rclcpp::Node
{
    private:
        int gI2cFd = 0;

        /**
         * Light message variables
         **/
        int32_t rgbw[4] = {-1, -1, -1, -1};
        int32_t gesture = -1;

        sem_t* gI2cSemaphore;

        rclcpp::TimerBase::SharedPtr lightTimer;
        rclcpp::TimerBase::SharedPtr proximityTimer;

        rclcpp::Publisher<robot_msgs::msg::Light>::SharedPtr lightPublisher;			 /* Light publisher for sensed RGB and gestures */
        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr proximityPublisher;			 /* Proximity publisher for distance information from front of robot */

        bool setupAPDS9960()
        {  
            sem_wait(gI2cSemaphore);
            std::cout << "Starting APDS9960 Setup" << std::endl;
            if (ioctl(this->gI2cFd, I2C_SLAVE, APDS9960) < 0)
            {
                return 0;
            }
            // Enable Proximity and Color
            i2c_smbus_write_byte_data(this->gI2cFd, ENABLEREG, 0x07);

            // Control Register 1 - LED Drive level Prox gain and color gain
            i2c_smbus_write_byte_data(this->gI2cFd, CTRLREGONE, 0x0B);

            sem_post(gI2cSemaphore);;

            return true;
        }

        bool readColor()
        {
            sem_wait(gI2cSemaphore);

            if (ioctl(this->gI2cFd, I2C_SLAVE, APDS9960) < 0)
            {
                std::cout << "Faild to set I2C Slave" << std::endl;
                return false;
            }

            // The color data is 16 bit
            uint16_t clear = i2c_smbus_read_word_data(this->gI2cFd, CLEARREG);
            uint16_t red = i2c_smbus_read_word_data(this->gI2cFd, REDREG);
            uint16_t green = i2c_smbus_read_word_data(this->gI2cFd, GREENREG);
            uint16_t blue = i2c_smbus_read_word_data(this->gI2cFd, BLUEREG);

            this->rgbw[0] = red;
            this->rgbw[1] = green;
            this->rgbw[2] = blue;
            this->rgbw[3] = clear;

            sem_post(gI2cSemaphore);

            return true;
        }

        uint8_t readProx()
        {
            sem_wait(gI2cSemaphore);

            if (ioctl(this->gI2cFd, I2C_SLAVE, APDS9960) < 0)
            {
                std::cout << "Faild to set I2C Slave" << std::endl;
                return false;
            }

            uint8_t readProx = i2c_smbus_read_byte_data(this->gI2cFd, PROXDATA);

            sem_post(gI2cSemaphore);
            return readProx;
        }

        void pubAPDS9960()
        {
            
            readColor();

            auto proxMsg = std_msgs::msg::Int16();
 
            proxMsg.data = readProx();

            this->proximityPublisher->publish(proxMsg);

            auto lightMsg = robot_msgs::msg::Light();
            // Insert RGBW values into message
            lightMsg.rgbw.push_back(this->rgbw[0]);
            lightMsg.rgbw.push_back(this->rgbw[1]);
            lightMsg.rgbw.push_back(this->rgbw[2]);
            lightMsg.rgbw.push_back(this->rgbw[3]);

            lightMsg.gesture = this->gesture;

            this->lightPublisher->publish(lightMsg);
        }

    public:
        APDS9960Publisher():Node("apds9960")
        {
            // TODO: Change the file premissions
            this->gI2cSemaphore = sem_open("/i2c_semaphore", O_CREAT, 777, 1);

            this->gI2cFd = open("/dev/i2c-1", O_RDWR);

            setupAPDS9960();

            // need to add parameters to know what sensors to turn on and subscirber so you can toggle them in real time
            // Create the publishers
            this->lightPublisher = this->create_publisher<robot_msgs::msg::Light>("/light", 5);
            this->proximityPublisher = this->create_publisher<std_msgs::msg::Int16>("/proximity", 5);

            // Create the timers
            this->lightTimer = this->create_wall_timer(DEFAULT_PUB_RATE, std::bind(&APDS9960Publisher::pubAPDS9960, this));
            std::cout << "Ready" << std::endl;
        }
};

int main(int argc, char *argv[])
{

	std::cout << "Spinning APDS9960 ROS Node" << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<APDS9960Publisher>());
	rclcpp::shutdown();
	return 0;
}