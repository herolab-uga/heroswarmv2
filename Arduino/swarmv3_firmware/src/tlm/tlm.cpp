#include "tlm.h"
#include "uart.h"
#include <stdio.h>
#include "router.h"
#include <stdint.h>
#include <string.h>
#include "defines.h"
#include "motors.h"

#include <Arduino.h>
#include "Adafruit_TinyUSB.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

static uint32_t gTlmPeriodMS = 1000;
static SemaphoreHandle_t gTlmPeriodMutex;

typedef struct
{
    // Odom TLM
    float xVel;
    float yVel;
    float omega;
    float x;
    float y;
    float theta;

    // Battery TLM
    uint16_t battery_voltage; // battery voltage is in mV

    // Do we want Dock TLM?

} tlm_t;

int set_tlm_period_ms(uint16_t len, void* args)
{
    if (sizeof(gTlmPeriodMS) != len)
    {
        DEBUG_PRINTF("Incorrect number of arguments");
        return -1;
    }
    LOCK_SEMAPHORE(gTlmPeriodMutex);
    memcpy(&gTlmPeriodMS, args, sizeof(gTlmPeriodMS));
    UNLOCK_SEMAPHORE(gTlmPeriodMutex);
    return 0;
}

uint32_t get_tlm_period_ms()
{
    uint32_t time_ms = 0;
    LOCK_SEMAPHORE(gTlmPeriodMutex);
    time_ms = gTlmPeriodMS;
    UNLOCK_SEMAPHORE(gTlmPeriodMutex);
    return time_ms;
}

void tlm_task(void* params)
{
    DEBUG_PRINTF("Starting Tlm Task");
    vTaskDelay(10/portTICK_PERIOD_MS);
    odom_t tmp_odom;
    uint32_t sleep_time = 0;

    tlm_t tlm_struct;

    uint8_t tlm_array[sizeof(tlm_struct)];

    TickType_t last_wake_time = xTaskGetTickCount();

    while (pdTRUE)
    {
        sleep_time = get_tlm_period_ms();
        memset(&tmp_odom, 0, sizeof(tmp_odom));
        memset(&tlm_array, 0, sizeof(tlm_array));
        memset(&tlm_struct, 0, sizeof(tlm_struct));

        // Get Odom tlm
        // send_odom(&tmp_odom);

        // Only want to copy the contents of tmp_odom struct to tlm_stuct
        memcpy(&tlm_struct, &tmp_odom, sizeof(tmp_odom) - (3 * sizeof(float))); 


        memcpy(tlm_array, &tlm_struct, sizeof(tlm_struct));
        uart_send_message(0xFF,tlm_array, sizeof(tlm_array));

        vTaskDelayUntil(&last_wake_time, sleep_time/portTICK_PERIOD_MS);
    }

}

void init_tlm()
{
    gTlmPeriodMutex = xSemaphoreCreateMutex();

    ROUTER_REGISTER(3, set_tlm_period_ms);
}
