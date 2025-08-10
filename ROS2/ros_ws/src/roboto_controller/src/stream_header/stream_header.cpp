#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "crc.hpp"
#include "stream_header.h"
#include "defines.hpp"

#define PKT_PROTOCOL_VERSION (0)


uint16_t wrap_pkt(const uint16_t apid, uint8_t* src_buff, uint8_t* dest_buff, size_t len)
{
	stream_header_t header;

	if ((MAX_MSG_SIZE - sizeof(stream_header_t) - CRC_SIZE) < len)
	{
		// Serial.println("Packet too large");
	}

	header.version = PKT_PROTOCOL_VERSION;
	header.apid = apid;

	// The size of the packet should include the crc
	// When the packet is RX'd and the header parsed for the correct length
	// the length has to include the crc since the crc is not apart of the
	// header.
	header.length = len + CRC_SIZE;

	memcpy(dest_buff, &header, sizeof(stream_header_t));
	memcpy(dest_buff + sizeof(stream_header_t), src_buff, len);
	

	return header.length + STREAM_HEADER_SIZE - CRC_SIZE;
}

uint16_t read_stream_pkt(uint8_t* buff, size_t len, stream_pkt_t* stream_pkt)
{
	memcpy(&stream_pkt, buff, len);
	stream_pkt->header.length -= CRC_SIZE;
	return stream_pkt->header.length + STREAM_HEADER_SIZE + CRC_SIZE;
}
