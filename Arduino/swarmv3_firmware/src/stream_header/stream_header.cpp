#include "crc.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "defines.h"
#include "stream_header.h"

#define PKT_PROTOCOL_VERSION (0)


uint16_t wrap_pkt(const uint16_t apid, uint8_t* src_buff, uint8_t* dest_buff, size_t len)
{
	stream_header_t header;

	if ((MAX_MSG_SIZE - sizeof(stream_header_t) - CRC_SIZE) < len)
	{
		printf("Packet too large\rn");
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
	memcpy(&stream_pkt->header, buff, sizeof(stream_header_t));
	stream_pkt->header.length -= CRC_SIZE;
	// Will reevaluate this later if we are running into buffer overflow problems
	// could potentially be tying up the buffer for too long
	stream_pkt->payload = &buff[sizeof(stream_header_t)];
	memcpy(&stream_pkt->crc, &buff[sizeof(stream_header_t) + (stream_pkt->header.length)], CRC_SIZE);
	return stream_pkt->header.length + STREAM_HEADER_SIZE + CRC_SIZE;
}
