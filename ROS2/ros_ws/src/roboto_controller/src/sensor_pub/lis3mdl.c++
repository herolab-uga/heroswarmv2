	bool setupLIS3MDL()
	{
		std::cout << "Starting LIS3MDL Setup" << std::endl;
		if (ioctl(i2cFd, I2C_SLAVE, LIS3MDL) < 0)
		{
			std::cout << "Faild to set LIS3MDL I2C Slave" << std::endl;
			return false;
		}

		if (i2c_smbus_write_byte_data(i2cFd, 0x20, 0xE2) != 0)
		{
			std::cout << "Failed sending BMP280 config" << std::endl;
			return false;
		}

		if (i2c_smbus_write_byte_data(i2cFd, 0x21, 0x00) != 0)
		{
			std::cout << "Failed sending BMP280 config" << std::endl;
			return false;
		}

		if (i2c_smbus_write_byte_data(i2cFd, 0x22, 0x00) != 0)
		{
			std::cout << "Failed sending BMP280 config" << std::endl;
			return false;
		}

		if (i2c_smbus_write_byte_data(i2cFd, 0x23, 0x0C) != 0)
		{
			std::cout << "Failed sending BMP280 config" << std::endl;
			return false;
		}

		if (i2c_smbus_write_byte_data(i2cFd, 0x24, 0x00) != 0)
		{
			std::cout << "Failed sending BMP280 config" << std::endl;
			return false;
		}

		return true;
	}

	void readMagField()
	{
		uint16_t magX = i2c_smbus_read_block_data(i2cFd,0x28);
		uint16_t magY = i2c_smbus_read_block_data(i2cFd,0x2a);
		uint16_t magZ = i2c_smbus_read_block_data(i2cFd,0x2c);
		uint16_t temp = i2c_smbus_read_block_data(i2cFd,0x2e);
	}