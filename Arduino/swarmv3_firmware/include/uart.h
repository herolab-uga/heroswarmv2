#pragma once

#include <stdio.h>
#include <stdint.h>

void init_uart();
void uart_rx_task(void*);
void uart_tx_task(void*);
void uart_send_message(uint16_t apid, void* data, size_t length);