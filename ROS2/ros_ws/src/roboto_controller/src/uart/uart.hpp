#pragma once
#ifndef UARTHEADER
#define UARTHEADER
#define BAUDRATE B115200

// error enumeration?

enum uartState
{
    CONFIGURED,     /* Uart port is configured */
    ERRFILO,        /* Error in opening the uart file */
    ERRGETATTR,     /* Error getting the tty parameters*/
    ERRSETATTR,     /* Error in setting the tty parameters */
    LOCKNOTSET      /* Lock not set */
};

int uartRead(uint8_t* buffer, size_t len);
int uartWrite(uint8_t* buffer, size_t len);
bool tryUartLock();
int uartInit();
void lockMutex();
void unlockMutex();

#endif