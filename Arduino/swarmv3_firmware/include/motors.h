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

typedef struct
{
    float P;
    float I;
    float D;

    volatile int8_t current_encoder;
    volatile int8_t last_encoder;

    volatile int32_t last_ticks;
    volatile int32_t current_ticks;
    volatile int32_t delta_ticks;

    float correction_factor;
    uint8_t error_pointer;
    float error_list[ERROR_WINDOW];

    int slice;
    uint8_t forward_pin;
    uint8_t backward_pin;

    uint8_t state_machine;

    float motor_speed;
    float motor_pid_speed;
    float motor_set_speed;

    SemaphoreHandle_t mutex;

} motor_t;

extern odom_t gRobotOdom;

extern motor_t gMotor1;
extern motor_t gMotor2;

extern motor_t* gMotorList[];

void send_odom(odom_t* data);

void motor_task(void* parameters);

void init_motor_control();

#endif
