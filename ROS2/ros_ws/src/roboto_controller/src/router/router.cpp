#include <stdio.h>
#include "router.h"
#include <string.h>

#define MAX_REGISTERED_APIDS    (256)

static router_dispatch_fuction_t gRouterDispatchFunctions [MAX_REGISTERED_APIDS];

int router_dispatch(uint16_t apid, uint16_t legnth, void* args)
{
    if (MAX_REGISTERED_APIDS < apid)
    {
        // Serial.println("APID out of range");
        return -1;
    }
    else
    {
        if (0 != gRouterDispatchFunctions[apid])
        {
            return gRouterDispatchFunctions[apid](legnth, args);
        }
        else
        {
            // printf("No function registered for this APID\n");
            return -1;
        }
        
    }
}

void router_register_apid(uint16_t apid, router_dispatch_fuction_t func)
{
    if (MAX_REGISTERED_APIDS < apid)
    {
        // Serial.println("Apid too large");
    }
    else
    {
        if (NULL == gRouterDispatchFunctions[apid])
        {
            gRouterDispatchFunctions[apid] = func;
        }
        else
        {
            // Serial.println("An function is already registered for this APID");
        }
        
    }
}

void init_router()
{
    memset(gRouterDispatchFunctions, 0, sizeof(gRouterDispatchFunctions));
}