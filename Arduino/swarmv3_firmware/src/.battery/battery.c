#include <stdio.h>
#include <stdint.h>
#include "hardware/adc.h"

#define CONVERSION_FACTOR	(4844/(1 << 12))


uint16_t read_battery_voltage()
{
	uint16_t voltage = 0;
	voltage = adc_read();
	voltage = (uint16_t) CONVERSION_FACTOR * voltage;

    return voltage;
}

void init_battery_adc()
{
    printf("Initializing ADC\r\n");

    // Enable the adc for battery readings
    adc_init();

    adc_gpio_init(27);

    adc_select_input(1);
}
