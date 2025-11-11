#include "router.h"
#include <Adafruit_NeoPixel.h>

#define RED_INDEX                       (0)
#define GREEN_INDEX                     (1)
#define BLUE_INDEX                      (2)
#define NEOPIXEL_COLOR_ARG_LEN          (3)
#define NEOPIXEL_BRIGHTNESS_ARG_LEN     (1)

static Adafruit_NeoPixel gNeoPixelLED = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);

static int neopixel_set_color(uint16_t length, void* args)
{
    if (NEOPIXEL_COLOR_ARG_LEN == length)
    {
        gNeoPixelLED.clear();
        gNeoPixelLED.setPixelColor(0, gNeoPixelLED.Color((((uint8_t*) args)[RED_INDEX]),
                                                         (((uint8_t*) args)[GREEN_INDEX]),
                                                         (((uint8_t*) args)[BLUE_INDEX]))
                                                        );
        gNeoPixelLED.show();

        return true;
    }
    else
    {
        return false;
    }
}

static int neopixel_set_brightness(uint16_t length, void* args)
{
    if (NEOPIXEL_BRIGHTNESS_ARG_LEN == length)
    {
        gNeoPixelLED.setBrightness(((uint8_t*) args)[0]);

        return true;
    }
    else
    {
        return false;
    }
}

void init_neopixel()
{
    gNeoPixelLED.begin();
    gNeoPixelLED.clear();
    gNeoPixelLED.setBrightness(255);
    gNeoPixelLED.show();

    ROUTER_REGISTER(0x1, neopixel_set_color);
    ROUTER_REGISTER(0x2, neopixel_set_brightness);
}