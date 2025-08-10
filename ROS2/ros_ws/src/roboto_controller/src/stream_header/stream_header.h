#include <stdint.h>
#include "defines.hpp"

typedef struct
{
	uint16_t version;
	uint16_t apid;
	uint16_t length;
} stream_header_t;

#define STREAM_HEADER_SIZE (sizeof(stream_header_t))

typedef struct
{
	stream_header_t header;
	uint8_t payload[MAX_MSG_SIZE];
	uint16_t crc;
} stream_pkt_t;

uint16_t wrap_pkt(const uint16_t apid, uint8_t* src_buff, uint8_t* dest_buff, size_t len);

uint16_t read_stream_pkt(uint8_t* buff, size_t len, stream_pkt_t* stream_pkt);

