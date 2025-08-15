#if not defined(MOTORS)
#define MOTORS

#include "stdio.h"
#include "stdlib.h"

#include <FreeRTOS.h>
#include "semphr.h"

#define ERROR_WINDOW            (10)

typedef struct
{
    float x;
    float y;
    float theta;

    float x_vel;
    float y_vel;
    float omega;

    float time;
    float last_time;
    float delta_time;

} odom_t;

extern odom_t gRobotOdom;

void send_odom(odom_t* data);

void motor_task(void* parameters);

void init_motor_control();

#endif
