#include "string.h"
#include "router.h"
#include "defines.h"
#include "charging.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#define DS0 17
#define DS1 16
#define DS2 18
#define DS3 19

static uint8_t gNumActive = 0;
static uint8_t gChargingPortState[4] = {0, 0, 0, 0};
static SemaphoreHandle_t gChargingPortStateMutex;

static uint32_t gChargingSwitchingPeriod = 0;
static SemaphoreHandle_t gChargingSwitchingPeriodMutex;

uint8_t get_port_state(uint8_t *buff)
{
    uint8_t num_active = 0;
    xSemaphoreTake(gChargingPortStateMutex, MUTEX_WAIT_TIME);
    memcpy(buff, gChargingPortState, sizeof(gChargingPortState));
    num_active = gNumActive;
    xSemaphoreGive(gChargingPortStateMutex);

    return gNumActive;
}

int set_port_state(uint16_t len, void *args)
{
    if (4 != len)
    {
        return -1;
    }

    xSemaphoreTake(gChargingPortStateMutex, MUTEX_WAIT_TIME);
    for (uint8_t i = 0; i < sizeof(gChargingPortState); i++)
    {
        gChargingPortState[i] = ((uint8_t *)args)[i];
        gNumActive += ((uint8_t *)args)[i];
    }
    xSemaphoreGive(gChargingPortStateMutex);

    return 0;
}

int get_switching_period()
{
    uint32_t period = 0;
    xSemaphoreTake(gChargingSwitchingPeriodMutex, MUTEX_WAIT_TIME);
    period = gChargingSwitchingPeriod;
    xSemaphoreGive(gChargingSwitchingPeriodMutex);
    return period;
}

int set_switching_period(uint16_t len, void *args)
{
    if (sizeof(gChargingSwitchingPeriod) != len)
    {
        return -1;
    }
    xSemaphoreTake(gChargingSwitchingPeriodMutex, MUTEX_WAIT_TIME);
    memcpy(&gChargingSwitchingPeriod, (uint8_t *)args, sizeof(gChargingSwitchingPeriod));
    xSemaphoreGive(gChargingSwitchingPeriodMutex);
    return 0;
}

void charging_task()
{
    uint8_t num_active = 0;
    uint8_t docking_port[4] = {0, 0, 0, 0};

    while (pdTRUE)
    {
        num_active = get_port_state(docking_port);

        if (num_active > 0)
        {
            for (uint8_t i = 0; i < num_active; i++)
            {
                if (0 != docking_port[i])
                {
                    switch (i)
                    {
                    case 0:
                        gpio_put(DS0, 1);
                        break;
                    case 1:
                        gpio_put(DS1, 1);
                        break;
                    case 2:
                        gpio_put(DS2, 1);
                        break;
                    case 3:
                        gpio_put(DS3, 1);
                        break;

                    default:
                        gpio_put(DS0, 0);
                        gpio_put(DS1, 0);
                        gpio_put(DS2, 0);
                        gpio_put(DS3, 0);
                        break;
                    }
                    vTaskDelay(get_switching_period() / num_active);

                    gpio_put(DS0, 0);
                    gpio_put(DS1, 0);
                    gpio_put(DS2, 0);
                    gpio_put(DS3, 0);
                }
            }
        }
        else
        {
            gpio_put(DS0, 0);
            gpio_put(DS1, 0);
            gpio_put(DS2, 0);
            gpio_put(DS3, 0);
        }
    }
}

void init_charging()
{

    gpio_init(DS0);
    gpio_set_dir(DS0, GPIO_OUT);
    gpio_pull_down(DS0);

    gpio_init(DS1);
    gpio_set_dir(DS1, GPIO_OUT);
    gpio_pull_down(DS1);

    gpio_init(DS2);
    gpio_set_dir(DS2, GPIO_OUT);
    gpio_pull_down(DS2);

    gpio_init(DS3);
    gpio_set_dir(DS3, GPIO_OUT);
    gpio_pull_down(DS3);

    gpio_put(DS0, 0);
    gpio_put(DS1, 0);
    gpio_put(DS2, 0);
    gpio_put(DS3, 0);

    gChargingPortStateMutex = xSemaphoreCreateMutex();
    gChargingSwitchingPeriodMutex = xSemaphoreCreateMutex();

    ROUTER_REGISTER(1, set_port_state);
    ROUTER_REGISTER(2, set_switching_period);
}
