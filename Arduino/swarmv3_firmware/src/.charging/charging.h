#include <stdint.h>

uint8_t get_port_state(uint8_t *buff);
int set_port_state(uint16_t len, void* args);
int set_switching_period(uint16_t len, void* args);
void init_charging();
void charging_task();