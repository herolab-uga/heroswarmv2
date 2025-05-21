/* C Library Headers */
#include <stdio.h>
#include <string.h>
#include <mutex>
#include <iostream>

/* Linux headers */
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sys/stat.h>
#include <pthread.h>
#include <mqueue.h>

/* ROS Headers */
// add the headers for ros logger
#include "uart.hpp"
#include "crc/crc.h"
#include "utils/defines.h"
#include "stream_header/stream_header.h"

#define BAUDRATE B921600
#define MQBASE "/uart_mutex"

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

} uart_errors_t;

typedef struct
{
    uint16_t apid;
    uint16_t length;
    uint8_t data[MAX_MSG_SIZE];
} queue_data_t;

const uint8_t SYNC_PATTERN[] = {0xDE, 0xAD, 0xBE, 0xEF};

int serialPort;
bool uartConfigured = false;
mqd_t gUartMessageQueue;

int uartRead(uint8_t* buffer, size_t len)
{
    int bytes_read = 0;
    bytes_read = read(serialPort,buffer,len);
    return bytes_read;
}

int32_t dispatch_serial(void* buff, size_t len)
{

    return mq_send(gUartMessageQueue, (char*) buff, len, NULL);

}

void uartTx(void* parameters)
{
    uint16_t ret = 0;
    uint16_t crc = 0;

    uint8_t data = 0;

    uint8_t buff[MAX_MSG_SIZE];

    mqd_t uart_tx_mq = mq_open(MQBASE, O_RDONLY);

    while (true)
    {

        ret = mq_receive(uart_tx_mq, (char*) buff, sizeof(buff), NULL);

        crc = calculate_crc(buff, ret);

        memcpy(&buff[ret], &crc, sizeof(crc));

        memmove(&buff[sizeof(SYNC_PATTERN)], buff, ret + CRC_SIZE);

        memcpy(buff, SYNC_PATTERN, sizeof(SYNC_PATTERN));

        write(serialPort, buff, ret + sizeof(SYNC_PATTERN) + CRC_SIZE); 
    }
}

// will take in a node to log to
int uartInit()
{

    std::cout << "Initializing UART" << std::endl;
    if (uartConfigured == true){
        return uartState::CONFIGURED; // return configured 
    }

    mq_attr attributes;

    attributes.mq_maxmsg = MAX_MSG_SIZE;
    attributes.mq_maxmsg = MAX_QUEUE_DEPTH;

    gUartMessageQueue = mq_open(MQBASE, O_WRONLY | O_CREAT | O_NONBLOCK, &attributes);
        
    struct termios tty;

    // set up the serial port config
    
    serialPort = open("/dev/serial0", O_RDWR);

    if (serialPort < 0)
    {
        // try to fix error
        return uartState::ERRFILO; // if unable to fix error return uart open error code
    }

    /* Get the default setting for the tty port. Applying setting without calling this function is undefined behavior*/
    if (tcgetattr(serialPort, &tty) != 0)
    {
        // try to fix error
        return uartState::ERRGETATTR; // if unable to fix return uart get attr error code
    }

    /**
    * Setup the serial port.
    **/
    tty.c_cflag &= ~PARENB;                                             /* Clear parity bit, disable parity Raspberry Pi does not use it */
    tty.c_cflag &= ~CSTOPB;                                             /* Clear stop field, onle one stop bit used in communication Raspberry Pi only uses on stop bit */
    tty.c_cflag &= ~CSIZE;                                              /* Clear all the size bits, then use one of the statements below */
    tty.c_cflag |= CS8;                                                 /* 8 bits per byte */
    tty.c_cflag &= ~CRTSCTS;                                            /* Disable flow control not used */
    tty.c_cflag |= CREAD | CLOCAL;                                      /* Turn on READ and ignore ctrl lines */

    tty.c_lflag &= ~ICANON;                                             /* Disable Canonical Mode */
    tty.c_lflag &= ~ECHO;                                               /* Disable echo */
    tty.c_lflag &= ~ECHOE;                                              /* Disable erasure */ 
    tty.c_lflag &= ~ECHONL;                                             /* Disable new-line echo */ 
    tty.c_lflag &= ~ISIG;                                               /* Disable interpretation of INTR, QUIT and SUSP */ 

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                             /* Turn off s/w flow ctrl */
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);    /* Disable any special handling of received bytes */ 

    tty.c_oflag &= ~OPOST;                                              /* Prevent special interpretation of output bytes (e.g. newline chars) */
    tty.c_oflag &= ~ONLCR;                                              /* Prevent conversion of newline to carriage return/line feed */

    tty.c_cc[VTIME] = 1;    /* Wait for up to 1s (1 deciseconds), returning as soon as any data is received. */
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty,BAUDRATE);
    cfsetospeed(&tty,BAUDRATE);

    /* Save tty settings, also checking for error */
    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        // try to fix error
        return uartState::ERRGETATTR; // if unable return uart set attr error code
    }

    std::cout << "Finished Initializing UART" << std::endl;
    
    return uartState::CONFIGURED;
}

//int main()
//{
//    
//    int ret = uartInit();
//
//    std::cout << ret << std::endl; 
//
//    std::string test = "Hello World";
//    while (true)
//    {
//	    std::cout << uartWrite(reinterpret_cast<const uint8_t*>(&test[0]),test.size()) << std::endl;
//    }
//    
//}
