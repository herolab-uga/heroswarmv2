#ifndef IMU_H
#define IMU_H
    #include <Arduino.h>
    #include <Adafruit_Sensor_Calibration.h>
    #include <Adafruit_AHRS.h>
    #include <Adafruit_LIS3MDL.h>
    #include <Adafruit_LSM6DS33.h>

    class imu{
        #define FILTER_UPDATE_RATE_HZ 100
        #define PRINT_EVERY_N_UPDATES 10
        uint32_t timestamp;
        int counter = 0; 
        public:
            Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
            Adafruit_LSM6DS33 lsm6ds;
            Adafruit_LIS3MDL lis3mdl;
            Adafruit_Madgwick filter;
            void setupIMU();
            bool init_sensors(void);
            void setup_sensors(void);
            void updateIMU();
            float getHeading(bool isRad = true);
            void printIMU();
    };


#endif