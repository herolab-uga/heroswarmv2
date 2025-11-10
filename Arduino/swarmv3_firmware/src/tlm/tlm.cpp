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
    float x_vel; //0-3
    float y_vel; //4-7
    float omega; //8-11
    float x; //12-15
    float y; //16-19
    float theta; //20-23

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
    vTaskDelay(10000/1024);
    odom_t tmp_odom;
    uint32_t sleep_time = 16;

    tlm_t tlm_struct;

    float tlm_array[sizeof(tlm_struct)];

    TickType_t last_wake_time = xTaskGetTickCount();

    while (pdTRUE)
    {
        sleep_time = get_tlm_period_ms();
        memset(&tmp_odom, 0, sizeof(tmp_odom));
        memset(&tlm_array, 0, sizeof(tlm_array));
        memset(&tlm_struct, 0, sizeof(tlm_struct));

        // Get Odom tlm
        send_odom(&tmp_odom);

        // DEBUG_PRINTF("Sending TLM Data");
        memcpy(&tlm_array[0], &tmp_odom.x_vel, 4);
        memcpy(&tlm_array[1], &tmp_odom.y_vel, 4);
        memcpy(&tlm_array[2], &tmp_odom.omega, 4);
        memcpy(&tlm_array[3], &tmp_odom.x, 4);
        memcpy(&tlm_array[4], &tmp_odom.y, 4);
        memcpy(&tlm_array[5], &tmp_odom.theta, 4);
        // memcpy(&tlm_array[6], &tmp_odom.battery_voltage, 4);
        uart_send_message(0xFF,tlm_array, 26);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(sleep_time));
    }

}

void init_tlm()
{
    gTlmPeriodMutex = xSemaphoreCreateMutex();

    ROUTER_REGISTER(3, set_tlm_period_ms);
}
