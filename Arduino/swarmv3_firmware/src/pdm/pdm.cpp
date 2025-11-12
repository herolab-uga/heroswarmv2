#include "PDM.h"
#include "uart.h"
#include "router.h"
#include "defines.h"

#include <FreeRTOS.h>
#include <semphr.h>

// number of samples read
volatile int32_t samples_read = 0;

// buffer to read samples into, each sample is 16-bits
short gPDMSampleBuffer[256];

static SemaphoreHandle_t gPDMSemaphore;

int get_mic_reading(uint16_t length, void* args)
{
    uint32_t mic_reading[1] = {0};
    LOCK_SEMAPHORE(gPDMSemaphore);
    mic_reading[0] = samples_read;
    UNLOCK_SEMAPHORE(gPDMSemaphore);

    uart_send_message(0x3, mic_reading, sizeof(mic_reading));
}

void onPDMdata()
{
    
    // query the number of bytes available
    int32_t bytes_available = PDM.available();

    // read into the sample buffer
    PDM.read(gPDMSampleBuffer, bytes_available);

    // 16-bit, 2 bytes per sample
    LOCK_SEMAPHORE(gPDMSemaphore);
    samples_read = bytes_available / 2;
    UNLOCK_SEMAPHORE(gPDMSemaphore);
}

void init_pdm()
{
    gPDMSemaphore = xSemaphoreCreateMutex();

    PDM.onReceive(onPDMdata);

    if (!PDM.begin(1, 16000))
    {
        Serial.println("Failed to start PDM!");
        while (1)
        yield();
    }
    PDM.setGain(.75);

    ROUTER_REGISTER(0x2, get_mic_reading);
}