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

/* ROS Headers */
// add the headers for ros logger

#include "uart.hpp"

#define MUTEXNAME "/tmp/uart_mutex"

int mutexFd;
int serialPort;
bool uartConfigured = false;

int uartRead(uint8_t* buffer, size_t len)
{
    int bytes_read = 0;
    bytes_read = read(serialPort,buffer,len);
    return bytes_read;
}

int uartWrite(uint8_t* const buffer, size_t len)
{
    return write(serialPort,buffer,len);
}

int lockUartMutex()
{
    // Lock the mutex
    struct flock fl;
    fl.l_type = F_WRLCK;  // Write lock
    fl.l_whence = SEEK_SET;
    fl.l_start = 0;
    fl.l_len = 0;
    return fcntl(mutexFd, F_SETLKW, &fl);
}

int unlockUartMutex() 
{
    // Lock the mutex
    struct flock fl;
    fl.l_type = F_UNLCK;  // Write lock
    fl.l_whence = SEEK_SET;
    fl.l_start = 0;
    fl.l_len = 0;
    return fcntl(mutexFd, F_SETLKW, &fl);
}

// will take in a node to log to
int uartInit()
{
    // Create the mutex
    mutexFd = open(MUTEXNAME,O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);

    if (mutexFd == -1) 
    {
        std::cerr << "Error opening " << MUTEXNAME << ": "  << strerror(errno)  << std::endl;
        return 1;
    }

    std::cout << "Initializing UART" << std::endl;
    if (uartConfigured == true){
        return uartState::CONFIGURED; // return configured 
    }
        
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
