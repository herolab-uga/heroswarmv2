#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

#include <stdio.h>
#include "router.h"
#include "defines.h"
// #include "includes/CAN.h"
#include "tlm.h"
#include "uart.h"
// #include "battery.h"
// #include "charging.h"
#include "motors.h"
// #include "Adafruit_TinyUSB.h"
#include "neopixel.hpp"

#if defined(TIMESTATS)
#if defined(DEBUG)
void debug(void* paramters)
{  
    DEBUG_PRINTF("Starting Run Time Stats Thread ");
    TickType_t last_wake_time = xTaskGetTickCount();

    char task_time_data[256];
    
    while (pdTRUE)
    {
        vTaskDelayUntil(&last_wake_time, 10000/1024);
        memset(task_time_data, 0, sizeof(task_time_data));
        vTaskGetRunTimeStats(task_time_data);
        DEBUG_PRINTF("%s", task_time_data);
    }
}
#endif
#endif

void setup() {
    // USB serial port init for debug
#if defined(DEBUG)
    Serial.begin(115200);
    while(!Serial);
    delay(2500);
    DEBUG_PRINTF("Starting......");
#endif

    init_router();

    init_tlm();
    init_uart();
    // init_battery_adc();
    init_motor_control();
    init_neopixel();

    DEBUG_PRINTF("Finished Setup");


    // Start the UART Task
    xTaskCreate(uart_rx_task, "UART RX Task",1024,NULL,tskIDLE_PRIORITY + 2,NULL);
    xTaskCreate(uart_tx_task, "UART TX Task",1024,NULL,tskIDLE_PRIORITY + 1,NULL);

    xTaskCreate(tlm_task, "TLM Task",1024,NULL,tskIDLE_PRIORITY + 1,NULL);
    
    xTaskCreate(motor_task, "Motor Control Task",1024,NULL,tskIDLE_PRIORITY + 1,NULL);
#if defined(TIMESTATS)
#if defined(DEBUG)
    xTaskCreate(motor_task, "Time Stats",1024,NULL,tskIDLE_PRIORITY + 1,NULL);
#endif
#endif
    
    // vTaskStartScheduler returning is an error state and thus the program should end. I may implement the watchdog to reset the system
    // if the function ever returns.
    // return 0;
}

void loop()
{

}
