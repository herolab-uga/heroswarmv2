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

        bool setupAPDS9960();
        bool readColor();
        uint8_t readProx();
        void pubAPDS9960();

    public:
        APDS9960Publisher();
};