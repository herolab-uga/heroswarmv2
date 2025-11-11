#include "crc.hpp"
#include "uart.hpp"
#include "router.h"
#include <string.h>
#include <limits.h>
#include "defines.hpp"
#include <fcntl.h>    /* For O_* constants */
#include <sys/stat.h> /* For mode constants */
#include <mqueue.h>
#include <unistd.h>
#include <sys/types.h>
#include <thread>
#include <termios.h>
#include <errno.h>
#include <iostream>
#include <semaphore.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/select.h>

#include "stream_header.h"

#define BAUDRATE (921600)

#define UART_FILE "/dev/serial0"

#define UART_QUEUE "/uart_tx_queue"

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

static int gUartFd = 0;
static mqd_t gUartTxQueue;

static std::thread gUartTxThread;
static std::thread gUartRxThread;

void print_buffer(const uint8_t *buffer, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        printf("%X ", buffer[i]);
    }

    printf("\n");
}

void uart_send_message(uint16_t apid, uint8_t *data, size_t length)
{
    uint16_t ret = 0;
    uint8_t buffer[MAX_MSG_SIZE] = {0};

    if (MAX_MSG_SIZE - (sizeof(SYNC_PATTERN) + STREAM_HEADER_SIZE + CRC_SIZE) >= length)
    {
        ret = wrap_pkt(apid, data, buffer, length);
        // printf("Sending message: %X | len: %u\n", apid, ret);
        mq_send(gUartTxQueue, (char *)buffer, ret, 0);
    }
    else
    {
    }
}

void uart_tx_thread()
{

    // Set real-time priority
    struct sched_param param;
    param.sched_priority = 90; // moderate RT priority
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0)
    {
        // ROS_WARN("Failed to set thread priority");
    }

    int16_t ret = 0;
    uint16_t crc = 0;
    uint8_t buff[MAX_MSG_SIZE];

    mqd_t tx_mq = mq_open(UART_QUEUE, O_RDONLY);

    if (tx_mq < 0)
    {
        printf("Failed to create msg queue %d | %s\n", errno, strerror(errno));
    }

    while (true)
    {
        memset(buff, 0, sizeof(buff));
        ret = mq_receive(tx_mq, (char *)buff, sizeof(buff), NULL);
        if (ret > 0)
        {
            crc = calculate_crc(buff, ret);

            memcpy(&buff[ret], &crc, sizeof(crc));

            memmove(&buff[sizeof(SYNC_PATTERN)], buff, ret + CRC_SIZE);

            memcpy(buff, SYNC_PATTERN, sizeof(SYNC_PATTERN));

            ret = write(gUartFd, (char *)buff, ret + sizeof(SYNC_PATTERN) + CRC_SIZE);
        }
        else if (ret < 0)
        {
            printf("Failed to read from msg queue %d | %s\n", errno, strerror(errno));
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

            for (; *length < sizeof(SYNC_PATTERN);)
            {
                ret = read(gUartFd, &buff[*length], 1);

                if (0 == memcmp(&buff[*length], &SYNC_PATTERN[*length], sizeof(buff[*length])))
                {
                    (*length)++;
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
                if (MAX_MSG_SIZE < read_len)
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

            if ((STREAM_HEADER_SIZE + read_len) != *length)
            {
                error = NO_MORE_BITS;
            }
            else
            {
                return NO_ERROR;
            }

            break;
        }
        }

    } while ((ret != 0) && (NO_ERROR == error));

    return error;
}

void uart_rx_thread()
{
    // Set real-time priority
    struct sched_param param;
    param.sched_priority = 60; // moderate RT priority
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0)
    {
        // ROS_WARN("Failed to set thread priority");
        printf("Failed to set rt prio\n");
    }

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
        FD_ZERO(&select_fds);
        FD_SET(gUartFd, &select_fds);
        if (select(gUartFd + 1, &select_fds, NULL, NULL, NULL))
        {
            length = 0;
            memset(buff, 0, sizeof(buff));

            ret = read_incoming_data(buff, &length);

            if (uart_errors_t::NO_ERROR == ret)
            {

                ret = (int32_t)read_stream_pkt(buff, length, &incoming_pkt);
                crc = calculate_crc(buff, length - CRC_SIZE);

                if (crc != incoming_pkt.crc)
                {
                    // printf("CRC Failed Incoming: %u | Caclulated: %u\n", incoming_pkt.crc, crc);
                }
                else
                {
                    router_dispatch(incoming_pkt.header.apid, incoming_pkt.header.length, incoming_pkt.payload);
                }
            }
            else
            {
            }
        }
    }
}

int set_interface_attribs(int fd, int speed, int parity)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0)
    {
        // error_message ("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0;        // no signaling chars, no echo,
                            // no canonical processing
    tty.c_oflag = 0;        // no remapping, no delays
    tty.c_cc[VMIN] = 0;     // read doesn't block
    tty.c_cc[VTIME] = 5;    // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem controls,
                                       // enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        // error_message ("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

/// @brief
void init_uart()
{
    std::cout << "Initializing UART" << std::endl;
    struct mq_attr queue_attr;
    queue_attr.mq_flags = 0;
    queue_attr.mq_maxmsg = 20;
    queue_attr.mq_curmsgs = 0;
    queue_attr.mq_msgsize = MAX_MSG_SIZE;

    mq_unlink(UART_QUEUE); // ignore errors

    // This queue will only be used in the
    gUartTxQueue = mq_open(UART_QUEUE, (O_CREAT | O_NONBLOCK | O_WRONLY), 0777, &queue_attr);

    if (gUartTxQueue < 0)
    {
        printf("Failed to create msg queue %d | %s\n", gUartTxQueue, strerror(errno));
    }

    gUartFd = open(UART_FILE, O_NONBLOCK | O_RDWR);

    printf("Opened uart fd: %d\n", gUartFd);

    if (-1 == gUartFd)
    {
        printf("Error opening uart fd %d | %s\n", gUartFd, strerror(errno));
    }

    // Configure UART
    set_interface_attribs(gUartFd, B921600, 0);

    // Spawn the RX thread
    gUartRxThread = std::thread(uart_rx_thread);
    gUartTxThread = std::thread(uart_tx_thread);

    std::cout << "Finished Initializing" << std::endl;
}
