#include <IMU.h>

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

void imu::setupIMU(){
    if (!cal.begin()) {
        Serial.println("Failed to initialize calibration helper");
    } else if (! cal.loadCalibration()) {
        Serial.println("No calibration loaded/found");
    }

    if (!init_sensors()) {
        Serial.println("Failed to find sensors");
        while (1) delay(10);
    }

    accelerometer->printSensorDetails();
    gyroscope->printSensorDetails();
    magnetometer->printSensorDetails();

    setup_sensors();
    filter.begin(FILTER_UPDATE_RATE_HZ);
    timestamp = millis();

    Wire.setClock(400000); // 400KHz
}

bool imu::init_sensors(void){
    if (!lsm6ds.begin_I2C() || !lis3mdl.begin_I2C()) {
        return false;
    }
    accelerometer = lsm6ds.getAccelerometerSensor();
    gyroscope = lsm6ds.getGyroSensor();
    magnetometer = &lis3mdl;

    return true;
}

void imu::setup_sensors(void){
    // set lowest range
    lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

    // set slightly above refresh rate
    lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
}

void imu::updateIMU(){

    float gx, gy, gz;
    float roll, pitch, heading;
    if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
        return;
    }
    timestamp = millis();
    // Read the motion sensors
    sensors_event_t accel, gyro, mag;
    accelerometer->getEvent(&accel);
    gyroscope->getEvent(&gyro);
    magnetometer->getEvent(&mag);

    cal.calibrate(mag);
    cal.calibrate(accel);
    cal.calibrate(gyro);
    // Gyroscope needs to be converted from Rad/s to Degree/s
    // the rest are not unit-important
    gx = (gyro.gyro.x) * SENSORS_RADS_TO_DPS;
    gy = (gyro.gyro.y) * SENSORS_RADS_TO_DPS;
    gz = (gyro.gyro.z) * SENSORS_RADS_TO_DPS;

    filter.update(gx, gy, gz, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

    if (counter++ <= PRINT_EVERY_N_UPDATES) {
        return;
    }
  // reset the counter
    // roll = filter.getRoll();
    // pitch = filter.getPitch();
    // heading = filter.getYaw();
    // Serial.print("Orientation: ");
    // Serial.print(heading);
    // Serial.print(", ");
    // Serial.print(pitch);
    // Serial.print(", ");
    // Serial.println(roll);
    counter = 0;
}

float imu::getHeading(bool isRad){
    float yaw;
    if(isRad){
        yaw = filter.getYawRadians();
    }
    else{
        yaw = filter.getYaw();
    }
    return yaw;
}
