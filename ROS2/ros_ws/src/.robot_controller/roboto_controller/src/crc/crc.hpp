#include <stdint.h>

#define CRC_SIZE (sizeof(uint16_t))

uint16_t calculate_crc(uint8_t *data, uint16_t length);
