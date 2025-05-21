#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "router.h"
#include <string.h>
#include "defines.h"
#include <stdint.h>
#include "pid_controller.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include "semphr.h"
#include <task.h>

static float successive_sum(float* list, uint8_t len);
static float successive_difference(float* array, uint8_t size); 

static float successive_sum(float* list, uint8_t len)
{  

    float sum = 0;
    for ( uint8_t i = 0; i < len; i++)
    {
        sum += list[i];
    }

    return sum;
}

static float successive_difference(float* array, uint8_t size) 
{

    // Ensure there's at least two elements to calculate differences
    if (size < 2) 
    {
        return 0.0f;  // Return 0 if no differences can be calculated
    }

    float sum = 0.0f;
    int count = size - 1;  // Number of differences

    // Calculate the sum of successive differences
    for (int i = 0; i < count; i++) 
    {
        sum += array[i + 1] - array[i];
    }

    // Return the average
    return sum / (float) count;
}

void calculate_correction_factor(motor_t* motor, odom_t* odom)
{

    // Calculate error in motor speed and the set point in radians per second
    xSemaphoreTake(motor->mutex, portTICK_PERIOD_MS);
    float error = motor->motor_set_speed - motor->motor_speed;
    xSemaphoreGive(motor->mutex);

#if not defined(TIMESTATS)
    DEBUG_PRINTF("Set Point: %f | Motor Speed: %f | Error: %f", motor->motor_set_speed, motor->motor_speed,error);
#endif
    // Add the error to the list
    motor->error_list[motor->error_pointer] = error;
    
    // Maintain list pointer position
    motor->error_pointer = (motor->error_pointer + 1) % ERROR_WINDOW;

    // Calculate the correction factor this is a change in radians per second
    motor->correction_factor = (motor->P * error) + (motor->I * successive_sum(motor->error_list, motor->error_pointer) * odom->delta_time) + (motor->D * successive_difference(motor->error_list, motor->error_pointer) / odom->delta_time);

}


