#include "router.h"
#include <Adafruit_NeoPixel.h>

#define RED_INDEX                       (0)
#define GREEN_INDEX                     (1)
#define BLUE_INDEX                      (2)
#define BRIGHTNESS_INDEX                (3)
#define NEOPIXEL_COLOR_ARG_LEN          (4)

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
        gNeoPixelLED.setBrightness(((uint8_t*) args)[BRIGHTNESS_INDEX]);
        gNeoPixelLED.show();

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

    gNeoPixelLED.setPixelColor(0,gNeoPixelLED.Color(0,0,0));
    gNeoPixelLED.setBrightness(0);
    gNeoPixelLED.show();

    ROUTER_REGISTER(0x1, neopixel_set_color);
}