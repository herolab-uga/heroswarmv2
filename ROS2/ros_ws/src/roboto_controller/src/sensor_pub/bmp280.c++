#include <stdio.h>

// I2C sensor's slave address
#define BMP280 0x77
// BMP280 Registers

// Temperature
#define T1 0x88
#define T2 0x8A
#define T3 0x8C

// Pressure
#define P1 0x8E
#define P2 0x90
#define P3 0x92
#define P4 0x94
#define P5 0x96
#define P6 0x98
#define P7 0x9A
#define P8 0x9C
#define P9 0x9E

/**
 * BMP280 Param Struct
 */
struct
{
    // Temperatur Parameters
    uint16_t t1;
    int16_t t2;
    int16_t t3;

    // Pressure Parameters
    uint16_t p1;
    int16_t p2;
    int16_t p3;
    int16_t p4;
    int16_t p5;
    int16_t p6;
    int16_t p7;
    int16_t p8;
    int16_t p9;

    int32_t t_fine;

} BMP280Params;

float bmp280_compensate_T_int32(int32_t adc_T)
{
    std::cout << "Raw Temperature: " << adc_T << std::endl;
    double var1, var2;
    float T;
    var1 = (((double)adc_T) / 16384.0 - ((double)BMP280Params.t1) / 1024.0) * ((double)BMP280Params.t2);                                                                        //((((adc_T >> 3) - ((int32_t)BMP280Params.t1 << 1))) * ((int32_t)BMP280Params.t2)) >> 11;
    var2 = ((((double)adc_T) / 131072.0 - ((double)BMP280Params.t1) / 8192.0) * (((double)adc_T) / 131072.0 - ((double)BMP280Params.t1) / 8192.0)) * ((double)BMP280Params.t3); //(((((adc_T >> 4) - ((int32_t)BMP280Params.t1)) * ((adc_T >> 4) - ((int32_t)BMP280Params.t1))) >> 12) * ((int32_t)BMP280Params.t3)) >> 14;
    BMP280Params.t_fine = (int32_t)(var1 + var2);
    T = BMP280Params.t_fine / 5120.0;
    std::cout << "Var1: " << var1 << std::endl;
    std::cout << "Var2: " << var2 << std::endl;
    std::cout << "Converted Temperature: " << T << std::endl; // Conv temp and pressure are wrong, replace the bmp280params struct values read from the device with the ones in the datasheet and check the math again to see if it matches what is in the datasheet
    return T;
}

int32_t bmp280_compensate_P_int64(int32_t adc_P)
{
    std::cout << "Raw Pressure: " << adc_P << std::endl;
    double var1, var2;
    float p;
    var1 = ((double)BMP280Params.t_fine / 2.0) - 64000.0; // This number came from the datasheet
    var2 = var1 * var1 * ((double)BMP280Params.p6) / 32768.0;
    var2 = var2 + var1 * ((double)BMP280Params.p5) * 2.0;
    var2 = (var2 / 4.0) + (((double)BMP280Params.p4) * 65536.0);
    var1 = (((double)BMP280Params.p3) * var1 * var1 / 524288.0 * ((double)BMP280Params.p2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)BMP280Params.p1);
    p = 1048576.0 - (double)adc_P;             // This number came from the datasheet
    p = (p - (var2 / 4096.0)) * 6250.0 / var1; // This number came from the datasheet
    var1 = ((double)BMP280Params.p9) * p * p / 2147483648.0;
    var2 = p * ((double)BMP280Params.p8) / 32768.0;
    p = p + (var1 + var2 + ((double)BMP280Params.p7)) / 16.0;
    std::cout << "Var1: " << var1 << std::endl;
    std::cout << "Var2: " << var2 << std::endl;
    std::cout << "Converted Pressure: " << p << std::endl;
    return (int32_t)p;
}

bool readParamsBMP280()
{
    // Get the temperature parameters
    BMP280Params.t1 = (uint16_t)(i2c_smbus_read_byte_data(i2cFd, T1 + 1) << 8 | i2c_smbus_read_byte_data(i2cFd, T1));
    std::cout << "T1: " << BMP280Params.t1 << std::endl;
    BMP280Params.t2 = (int16_t)(i2c_smbus_read_byte_data(i2cFd, T2 + 1) << 8 | i2c_smbus_read_byte_data(i2cFd, T2));
    std::cout << "T2: " << BMP280Params.t2 << std::endl;
    BMP280Params.t3 = (int16_t)(i2c_smbus_read_byte_data(i2cFd, T3 + 1) << 8 | i2c_smbus_read_byte_data(i2cFd, T3));
    std::cout << "T3: " << BMP280Params.t3 << std::endl;

    // Get the pressure parameters
    BMP280Params.p1 = (uint16_t)(i2c_smbus_read_byte_data(i2cFd, (P1 + 1)) << 8 | i2c_smbus_read_byte_data(i2cFd, P1));
    std::cout << "P1: " << BMP280Params.p1 << std::endl;
    BMP280Params.p2 = (int16_t)(i2c_smbus_read_byte_data(i2cFd, P2 + 1) << 8 | i2c_smbus_read_byte_data(i2cFd, P2));
    std::cout << "P2: " << BMP280Params.p2 << std::endl;
    BMP280Params.p3 = (int16_t)(i2c_smbus_read_byte_data(i2cFd, P3 + 1) << 8 | i2c_smbus_read_byte_data(i2cFd, P3));
    std::cout << "P3: " << BMP280Params.p3 << std::endl;
    BMP280Params.p4 = (int16_t)(i2c_smbus_read_byte_data(i2cFd, P4 + 1) << 8 | i2c_smbus_read_byte_data(i2cFd, P4));
    std::cout << "P4: " << BMP280Params.p4 << std::endl;
    BMP280Params.p5 = (int16_t)(i2c_smbus_read_byte_data(i2cFd, P5 + 1) << 8 | i2c_smbus_read_byte_data(i2cFd, P5));
    std::cout << "P5: " << BMP280Params.p5 << std::endl;
    BMP280Params.p6 = (int16_t)(i2c_smbus_read_byte_data(i2cFd, P6 + 1) << 8 | i2c_smbus_read_byte_data(i2cFd, P6));
    std::cout << "P6: " << BMP280Params.p6 << std::endl;
    BMP280Params.p7 = (int16_t)(i2c_smbus_read_byte_data(i2cFd, P7 + 1) << 8 | i2c_smbus_read_byte_data(i2cFd, P7));
    std::cout << "P7: " << BMP280Params.p7 << std::endl;
    BMP280Params.p8 = (int16_t)(i2c_smbus_read_byte_data(i2cFd, P8 + 1) << 8 | i2c_smbus_read_byte_data(i2cFd, P8));
    std::cout << "P8: " << BMP280Params.p8 << std::endl;
    BMP280Params.p9 = (int16_t)(i2c_smbus_read_byte_data(i2cFd, P9 + 1) << 8 | i2c_smbus_read_byte_data(i2cFd, P9));
    std::cout << "P9: " << BMP280Params.p9 << std::endl;
    return true;
}

bool setupBMP280()
{
    std::cout << "Starting BMP280 Setup" << std::endl;
    if (ioctl(i2cFd, I2C_SLAVE, BMP280) < 0)
    {
        std::cout << "Faild to set I2C Slave" << std::endl;
        return false;
    }

    if (readParamsBMP280() != true)
    {
        std::cout << "Faild to read parameters" << std::endl;
        return false;
    }

    std::cout << "Writing BMP280 Register F4 and F5" << std::endl;
    buf[0] = 0x5F;
    if (i2c_smbus_write_byte_data(i2cFd, 0xF4, 0x5F) != 0)
    {
        std::cout << "Failed sending BMP280 config" << std::endl;
        return false;
    }

    buf[0] = 0x10;
    if (i2c_smbus_write_byte_data(i2cFd, 0xF5, 0x10) != 0)
    {
        std::cout << "Failed sending BMP280 config" << std::endl;
        return false;
    }

    std::cout << "F5: " << i2c_smbus_read_byte_data(i2cFd, 0xF5) << std::endl;

    std::cout << "F4: " << i2c_smbus_read_byte_data(i2cFd, 0xF4) << std::endl;

    return true;
}

void readPressure()
{
    if (ioctl(i2cFd, I2C_SLAVE, BMP280) < 0)
    {
        return 0;
    }
    uint32_t rawTemp = (uint32_t)(i2c_smbus_read_byte_data(i2cFd, 0xFA) << 12 | i2c_smbus_read_byte_data(i2cFd, 0xFB) << 4 | ((i2c_smbus_read_byte_data(i2cFd, 0xFC) >> 4)));
    float convTemp = bmp280_compensate_T_int32(rawTemp & 0x000FFFF8);

    environmentMutex.lock();
    temp = (float)convTemp;
    environmentMutex.unlock();

    uint32_t rawPressure = (uint32_t)(i2c_smbus_read_byte_data(i2cFd, 0xF7) << 12 | i2c_smbus_read_byte_data(i2cFd, 0xF8) << 4 | ((i2c_smbus_read_byte_data(i2cFd, 0xF9) >> 4)));
    int32_t convPressure = bmp280_compensate_P_int64(rawPressure);

    environmentMutex.lock();
    pressure = (float)convPressure;
    environmentMutex.unlock();
}