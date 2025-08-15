#include "crc.h"
#include "uart.h"
#include "stdio.h"
#include <router.h>
#include <string.h>
#include <limits.h>
#include "defines.h"

#include "stream_header.h"
#include "motors.h"

// #include "inclueds/CAN.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <FreeRTOS.h>
#include "semphr.h"

#define BAUDRATE (921600)

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
    NOT_IMPLEMENTED = -5,

} uart_errors_t;

typedef struct
{
    uint16_t apid;
    uint16_t length;
    uint8_t data[MAX_MSG_SIZE];
} queue_data_t;

const uint8_t SYNC_PATTERN[] = {0xDE, 0xAD, 0xBE, 0xEF};

QueueHandle_t gUartTxQueue;

void uart_send_message(uint16_t apid, uint8_t *data, size_t length)
{
    queue_data_t temp_data;
    memset(&temp_data, 0, sizeof(temp_data));

    if (MAX_MSG_SIZE - (sizeof(SYNC_PATTERN) + STREAM_HEADER_SIZE + CRC_SIZE) >= length)
    {
        memcpy(temp_data.data, data, length);

        temp_data.apid = apid;
        temp_data.length = length;
        xQueueSend(gUartTxQueue, &temp_data, 0);
    }
    else
    {
        DEBUG_PRINTF("Message too large!!");
    }
}

void uart_tx_task(void *params)
{
    DEBUG_PRINTF("Starting UART TX Task");
    vTaskDelay(10000/1024);
    uint16_t ret = 0;
    uint16_t crc = 0;

    queue_data_t temp_data;

    TickType_t last_wake_time = xTaskGetTickCount();

    uint8_t buff[MAX_MSG_SIZE];

    while (pdTRUE)
    {
        memset(&temp_data, 0, sizeof(temp_data));
        ret = xQueueReceive(gUartTxQueue, &temp_data, 99999);
        if (pdTRUE == ret)
        {
            ret = wrap_pkt(temp_data.apid, temp_data.data, buff, temp_data.length);

            crc = calculate_crc(buff, ret);
    
            memcpy(&buff[ret], &crc, sizeof(crc));
    
            memmove(&buff[sizeof(SYNC_PATTERN)], buff, ret + CRC_SIZE);
    
            memcpy(buff, SYNC_PATTERN, sizeof(SYNC_PATTERN));
    
            Serial1.write(buff,ret);
        }
    }
}

uart_errors_t read_incoming_data(uint8_t *buff, size_t *length)
{
    uart_errors_t error = NO_ERROR;
    uint16_t read_len = 0;
    uart_state_t state = SYNC;

    do
    {
        error = NO_ERROR;

        switch (state)
        {
        case SYNC:
        {
            DEBUG_PRINTF("Reading Sync");
            for (; *length < sizeof(SYNC_PATTERN) && Serial1.available(); (*length)++)
            {
                buff[*length] = Serial1.read();
                DEBUG_PRINTF("%02X ", buff[*length]);
                if (0 != memcmp(&buff[*length], &SYNC_PATTERN[*length], sizeof(buff[*length])))
                {
                    error = SYNC_ERROR;
                    break;
                }
            }
            if (sizeof(SYNC_PATTERN) != *length)
            {
                error = NO_MORE_BITS;
            }
            else
            {
                DEBUG_PRINTF("");
                state = HEADER;
                *length = 0;
            }
            break;
        }
        case HEADER:
        {
            DEBUG_PRINTF("Reading Header");
            for (; *length < sizeof(stream_header_t) && Serial1.available(); (*length)++)
            {
                buff[*length] = Serial1.read();
                DEBUG_PRINTF("%02X ", buff[*length]);
            }

            if (STREAM_HEADER_SIZE != *length)
            {
                error = NO_MORE_BITS;
            }
            else
            {
                state = DATA;
                DEBUG_PRINTF("");
                // 4 is the start index of the data length in the header
                memcpy(&read_len, &buff[4], sizeof(uint16_t));
                if ( MAX_MSG_SIZE < read_len)
                {
                    error = MAX_SIZE_EXCEEDED;
                    break;
                }
            }
            break;
        }
        case DATA:
        {
            for (int32_t i = 0; i < read_len && Serial1.available(); i++)
            {
                buff[*length] = Serial1.read();
                DEBUG_PRINTF("%02X ", buff[*length]);
                (*length)++;
            }

            if (STREAM_HEADER_SIZE +  read_len != *length)
            {
                error = NO_MORE_BITS;
            }
            else
            {
                DEBUG_PRINTF("");
            }
            DEBUG_PRINTF("");
            break;
        }
        }

    } while (Serial1.available() && (NO_ERROR == error));

    while (Serial1.available())
    {
        Serial1.read();
    }

    return error;
}

void uart_rx_task(void *parameters)
{
    DEBUG_PRINTF("Starting UART RX Task");
    vTaskDelay(10000/1024);

    int32_t ret = 0;

    uint16_t crc = 0;

    size_t length = 0;

    stream_pkt_t incoming_pkt;

    uint8_t buff[MAX_MSG_SIZE];

    while (pdTRUE)
    {
        if (Serial1.available())
        {
            DEBUG_PRINTF("Data available");
            length = 0;
            memset(buff, 0, sizeof(buff));

            ret = read_incoming_data(buff, &length);

            if (ret >= 0)
            {
                ret = (int32_t)read_stream_pkt(buff, length, &incoming_pkt);
                crc = calculate_crc(buff, ret - CRC_SIZE);

                if (crc != incoming_pkt.crc)
                {
                    DEBUG_PRINTF("CRC error");
                }
                else
                {
                    router_dispatch(incoming_pkt.header.apid, incoming_pkt.header.length, incoming_pkt.payload);
                }
            }
            else
            {
            }
        }
        vTaskDelay(5000/1024);
    }
}

/// @brief
void init_uart()
{
    gUartTxQueue = xQueueCreate(MAX_QUEUE_DEPTH,sizeof(queue_data_t));
    Serial1.begin(BAUDRATE);
}
