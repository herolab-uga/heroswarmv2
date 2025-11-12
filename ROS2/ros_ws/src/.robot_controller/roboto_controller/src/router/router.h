#include <stdint.h>

#define ROUTER_REGISTER(apid, func) router_register_apid(apid, (router_dispatch_fuction_t) func)

typedef int (*router_dispatch_fuction_t) (uint16_t length, void* args);

void init_router();
int router_dispatch(uint16_t apid, uint16_t legnth, void* args);
void router_register_apid(uint16_t apid, router_dispatch_fuction_t func);