#include "FreeRTOS.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "includes/CAN.h"
#include "includes/uart.h"
#include "stdio.h"
#include "pico/stdio.h"
#include <limits.h>
#include <string.h>

uint8_t msgBuff [MAX_MSG_SIZE];
uint8_t msgLen = 0;


void uart_rx_isr(void){
    while (uart_is_readable(UART_ID)){
        msgBuff[msgLen] = uart_getc(UART_ID);
        msgLen++;
    }
    // for (int i = 0; i < msgLen; i++){
    //     printf("%i",msgBuff[i]);
    //     printf(" ");
    // }
    // printf("\n\r");
    if (msgBuff[msgLen-1] == '\n'){
        printf("Full Message");
        printf("\r\n");
        vTaskNotifyGiveIndexedFromISR(CANUartTaskHandle,0,NULL);
    }
    // printf("Inside ISR\r\n");
}

/// @brief 
void CANInit(){
    uint baudrate = uart_init(CAN_UART_ID,115200);
    printf("%d\r\n",baudrate);

    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);

    uart_set_hw_flow(CAN_UART_ID,pdFALSE,pdFALSE);

    uart_set_format(CAN_UART_ID,8,1,UART_PARITY_NONE);

    uart_set_fifo_enabled(CAN_UART_ID,pdTRUE);
    
    // irq_set_exclusive_handler(CAN_UART_IRQ,uart_rx_isr);
    irq_set_enabled(CAN_UART_IRQ,pdTRUE);
    uart_set_irq_enables(CAN_UART_ID,pdTRUE,pdFALSE);
}

void CANTask(){

    CANUartTaskHandle = xTaskGetCurrentTaskHandle();

    uint32_t notifiedValue;
    
    while(pdTRUE){
        // uart_puts(UART_ID, "\nHello, uart interrupts\n");
        // printf("UART Init\r\n");
        ulTaskNotifyTakeIndexed(0,pdTRUE,portMAX_DELAY);
        uart_write_blocking(UART_ID,msgBuff,msgLen);
    }
}