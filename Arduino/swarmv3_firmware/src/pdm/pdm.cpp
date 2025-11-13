#include "PDM.h"
#include "uart.h"
#include "router.h"
#include "defines.h"

#include <FreeRTOS.h>
#include <semphr.h>

// buffer to read samples into, each sample is 16-bits
short gPDMSampleBuffer[256];

static int32_t onPDMdata()
{
    
    // query the number of bytes available
    int32_t bytes_available = PDM.available();

    // read into the sample buffer
    PDM.read(gPDMSampleBuffer, bytes_available);

    // 16-bit, 2 bytes per sample
    return (bytes_available / 2);
}

int get_mic_reading(uint16_t length, void* args)
{
    uint32_t mic_reading[1] = {0};

    mic_reading[0] = onPDMdata();

    uart_send_message(0x3, mic_reading, sizeof(mic_reading));

    return 0;
}

void init_pdm()
{
    if (!PDM.begin(1, 16000))
    {
        Serial.println("Failed to start PDM!");
        while (1)
        yield();
    }
    PDM.setGain(.75);

    ROUTER_REGISTER(0x2, get_mic_reading);
}