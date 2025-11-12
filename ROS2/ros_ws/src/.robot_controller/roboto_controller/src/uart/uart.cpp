#include "crc.hpp"
#include "uart.hpp"
#include "router.h"
#include <string.h>
#include <limits.h>
#include "defines.hpp"
#include <fcntl.h>           /* For O_* constants */
#include <sys/stat.h>        /* For mode constants */
#include <mqueue.h>
#include <unistd.h>
#include <sys/types.h>
#include <thread>


#include <stdio.h>
#include <stdlib.h>
#include <sys/select.h>

#include "stream_header.h"

#define BAUDRATE        (921600)
    
#define UART_FILE       "/dev/ttyACM0"

typedef enum
{
    SYNC,
    HEADER,
    DATA,
} uart_state_t;

typedef enum
{
    NO_ERROR = 0,
    SYNC_ERROR = -1,
    LENGTH_ERROR = -2,
    NO_MORE_BITS = -3,
    MAX_SIZE_EXCEEDED = -4,
    NOT_IMPLEMENTED = -5,

} uart_errors_t;

const uint8_t SYNC_PATTERN[] = {0xDE, 0xAD, 0xBE, 0xEF};

int gUartFd = 0;
mqd_t gUartTxQueue;

std::thread gUartTxThread;
std::thread gUartRxThread;

void uart_send_message(uint16_t apid, uint8_t *data, size_t length)
{
    uint16_t ret = 0;
    uint8_t buffer[MAX_MSG_SIZE] = {0};

    if (MAX_MSG_SIZE - (sizeof(SYNC_PATTERN) + STREAM_HEADER_SIZE + CRC_SIZE) >= length)
    {
        ret = wrap_pkt(apid, data, buffer, length);
        mq_send(gUartTxQueue, (char*) buffer, ret, 0);
    }
    else
    {
        
    }
}

void uart_tx_thread()
{

    int16_t ret = 0;
    uint16_t crc = 0;
    uint8_t buff[MAX_MSG_SIZE];

    while (true)
    {
        memset(buff, 0, sizeof(buff));
        ret = mq_receive(gUartTxQueue, (char*) buff, sizeof(buff), NULL);
        if (ret > 0)
        {
            crc = calculate_crc(buff, ret);
    
            memcpy(&buff[ret], &crc, sizeof(crc));
    
            memmove(&buff[sizeof(SYNC_PATTERN)], buff, ret + CRC_SIZE);
    
            memcpy(buff, SYNC_PATTERN, sizeof(SYNC_PATTERN));
    
            ret = write(gUartFd, (char*) buff, ret);

        }

    }
}

uart_errors_t read_incoming_data(uint8_t *buff, size_t *length)
{
    int ret = 0;
    uart_errors_t error = NO_ERROR;
    uint16_t read_len = 0;
    uart_state_t state = SYNC;

    do
    {
        error = NO_ERROR;

        switch (state)
        {
        case SYNC:
        {
            
            for (; *length < sizeof(SYNC_PATTERN); (*length)++)
            {
                ret = read(gUartFd, &buff[*length], 1);
                
                if (0 != memcmp(&buff[*length], &SYNC_PATTERN[*length], sizeof(buff[*length])))
                {
                    error = SYNC_ERROR;
                    break;
                }
            }
            if (sizeof(SYNC_PATTERN) != *length)
            {
                error = NO_MORE_BITS;
            }
            else
            {
                
                state = HEADER;
                *length = 0;
            }
            break;
        }
        case HEADER:
        {
            
            for (; *length < sizeof(stream_header_t); (*length)++)
            {
                ret = read(gUartFd, &buff[*length], 1);
                
            }

            if (STREAM_HEADER_SIZE != *length)
            {
                error = NO_MORE_BITS;
            }
            else
            {
                state = DATA;
                
                // 4 is the start index of the data length in the header
                memcpy(&read_len, &buff[4], sizeof(uint16_t));
                if ( MAX_MSG_SIZE < read_len)
                {
                    error = MAX_SIZE_EXCEEDED;
                    break;
                }
            }
            break;
        }
        case DATA:
        {
            for (int32_t i = 0; i < read_len; i++)
            {
                ret = read(gUartFd, &buff[*length], 1);
                
                (*length)++;
            }

            if (STREAM_HEADER_SIZE +  read_len != *length)
            {
                error = NO_MORE_BITS;
            }
            else
            {
                
            }
            
            break;
        }
        }

    } while ((ret != 0) && (NO_ERROR == error));

    return error;
}

void uart_rx_thread()
{
    
    int32_t ret = 0;

    uint16_t crc = 0;

    size_t length = 0;

    stream_pkt_t incoming_pkt;

    fd_set select_fds;

    FD_ZERO(&select_fds);
    FD_SET(gUartFd, &select_fds);
    uint8_t buff[MAX_MSG_SIZE];

    while (true)
    {
        if (select(gUartFd + 1, &select_fds, NULL, NULL, NULL))
        {
            
            length = 0;
            memset(buff, 0, sizeof(buff));

            ret = read_incoming_data(buff, &length);

            if (ret >= 0)
            {
                ret = (int32_t)read_stream_pkt(buff, length, &incoming_pkt);
                crc = calculate_crc(buff, ret - CRC_SIZE);

                if (crc != incoming_pkt.crc)
                {
                    
                }
                else
                {
                    router_dispatch(incoming_pkt.header.apid, incoming_pkt.header.length, incoming_pkt.payload);
                }
            }
        }
    }
}

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
                // error_message ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                // error_message ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

/// @brief
void init_uart()
{
    struct mq_attr queue_attr;
    queue_attr.mq_maxmsg = 20;
    queue_attr.mq_msgsize = MAX_MSG_SIZE;

    // This queue will only be used in the 
    gUartTxQueue = mq_open("uart_tx_queue", O_CREAT | O_NONBLOCK, queue_attr);

    gUartFd = open(UART_FILE, O_NONBLOCK);

    // Configure UART
    set_interface_attribs(gUartFd, BAUDRATE, 0);

    // Spawn the RX thread
    gUartRxThread = std::thread(uart_tx_thread);
    gUartTxThread = std::thread(uart_rx_thread);

    
    
}
