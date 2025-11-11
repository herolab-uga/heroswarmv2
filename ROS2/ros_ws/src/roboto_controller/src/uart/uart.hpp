#pragma once
#ifndef UARTHEADER
#define UARTHEADER
#include <cstdint>
#include <sys/types.h>

// error enumeration?

enum uartState
{
    CONFIGURED,     /* Uart port is configured */
    ERRFILO,        /* Error in opening the uart file */
    ERRGETATTR,     /* Error getting the tty parameters*/
    ERRSETATTR,     /* Error in setting the tty parameters */
    LOCKNOTSET      /* Lock not set */
};

void init_uart();
void uart_send_message(uint16_t apid, uint8_t *data, size_t length);

#endif