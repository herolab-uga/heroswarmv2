#include "Adafruit_TinyUSB.h"
#include <FreeRTOS.h>
#include <semphr.h>

#pragma once
#define MAX_QUEUE_DEPTH (5)
#define MAX_MSG_SIZE (1024)
#define SEMAPHORE_WAIT_TIME (100)
#define MUTEX_WAIT_TIME     (SEMAPHORE_WAIT_TIME)

#define READ_TIMEOUT_MSEC   500 

#define LOCK_SEMAPHORE(x)      (xSemaphoreTake(x, MUTEX_WAIT_TIME))
#define UNLOCK_SEMAPHORE(x)    (xSemaphoreGive(x))

#if defined(DEBUG)
#define DEBUG_PRINTF(format, ...)   Serial.printf(format"\t(%s:%d)\r\n", ##__VA_ARGS__, __func__,__LINE__)
#else
#define DEBUG_PRINTF(format,...)
#endif

#define MIN(x,y) (x > y ? y : x)
#define MAX(x,y) (x > y ? x : y)