#include "crc.hpp"
#include <stdio.h>

uint16_t calculate_crc(uint8_t *data, uint16_t length)
  {
    uint8_t lsb, msb;
    uint16_t checksum = 0;
    for (uint8_t i = 0; i < length; i++)
    {
      lsb = checksum;
      msb = (checksum >> 8) + data[i];
      lsb += msb;
      checksum = ((uint16_t)msb << 8) | (uint16_t)lsb;
    }
    return checksum;
  }

