#include <stdint.h>


int set_tlm_period_ms(uint16_t len, void* args);
uint32_t get_tlm_period_ms();
void tlm_task(void*);
void init_tlm();