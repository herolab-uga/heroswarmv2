#include <stdio.h>
#include "router.h"
#include <string.h>
#include "defines.h"

#include <Arduino.h>
#include "Adafruit_TinyUSB.h"

#define MAX_REGISTERED_APIDS    (256)

static router_dispatch_fuction_t gRouterDispatchFunctions [MAX_REGISTERED_APIDS];

int router_dispatch(uint16_t apid, uint16_t legnth, void* args)
{
    if (MAX_REGISTERED_APIDS < apid)
    {
        DEBUG_PRINTF("APID out of range");
    }
    else
    {
        if (NULL != gRouterDispatchFunctions[apid])
        {
            return gRouterDispatchFunctions[apid](legnth, args);
        }
        else
        {
            DEBUG_PRINTF("No function registered for this APID");
            return -1;
        }
        
    }
    return -1;
}

void router_register_apid(uint16_t apid, router_dispatch_fuction_t func)
{
    if (MAX_REGISTERED_APIDS < apid)
    {
        DEBUG_PRINTF("Apid too large");
    }
    else
    {
        if (NULL == gRouterDispatchFunctions[apid])
        {
            gRouterDispatchFunctions[apid] = func;
        }
        else
        {
            DEBUG_PRINTF("An function is already registered for this APID");
        }
        
    }
}

void init_router()
{
    memset(gRouterDispatchFunctions, 0, sizeof(gRouterDispatchFunctions));
}