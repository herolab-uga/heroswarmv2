#include <semaphore.h>
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/environment.hpp"

class BMP280Publisher : public rclcpp::Node
{
    private:

        int gI2cFd = 0;
        sem_t* gI2cSemaphore;

        bool readParamsBMP280();

        bool setupBMP280();

        float bmp280_compensate_T_int32(int32_t adc_T);

        int32_t bmp280_compensate_P_int64(int32_t adc_P);

        bool readPressure();

        void pubEnvironment();

    public:
        BMP280Publisher();
};