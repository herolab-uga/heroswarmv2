#include "math.h"
#include <stdio.h>
#include "motors.h"
#include "stdlib.h"
#include "router.h"
#include <string.h>
#include "defines.h"
#include "pid_controller.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include "semphr.h"
#include <task.h>
#include <HardwareSerial.h>

// Motor Webpage: https://www.adafruit.com/product/4639

// Motor Parameters
#define MAX_PWM                 (255)
#define GEAR_RATIO              (100)
#define TICK_PER_ROTATION       (28)
#define MAX_ROTATIONS_MIN       (65.0f)
#define MAX_RADIANS_SEC         ((MAX_ROTATIONS_MIN * 2 * PI) / 60.f)

#define NUM_MOTORS              (2)

// Define the motor control and encoder pins
#define MOTOR1_FORWARD          (12)
#define MOTOR1_BACKWARD         (13)
#define MOTOR1_ENCODER_A        (A3)
#define MOTOR1_ENCODER_B        (A2)

#define MOTOR2_FORWARD          (9)
#define MOTOR2_BACKWARD         (10)
#define MOTOR2_ENCODER_A        (A1)
#define MOTOR2_ENCODER_B        (A0)

#define MOTOR_FEED_FORWARD      (100)

uint8_t left_sum = 0;
uint8_t left_encoder = 0;

uint8_t right_sum = 0;
uint8_t right_encoder = 0;

uint32_t start = 0;

odom_t gRobotOdom;

motor_t gMotor1;
motor_t gMotor2;

motor_t* gMotorList[] = {&gMotor1, &gMotor2};

static void update_pid();
static void get_ticks(motor_t* motor);
static void configure_pwm(motor_t* motor);
static void update_odom(odom_t* robot_odom);
static int _set_velocity(uint16_t length, void* args);
void set_velocity(float xVel, float yVel, float thetaVel);
static void motor_set_velocity(motor_t* motor, float vel);
static void calculate_radian_change(motor_t* motor, float time);
static void calculate_forward_kinematics(float delta1, float delta2, odom_t* odom);

void update_right_encoder()
{
    //converting the 2 pin value to single number
    right_encoder = (digitalRead(MOTOR2_ENCODER_A) << 1) | digitalRead(MOTOR2_ENCODER_B);

    //adding it to the previous right_encoder value
    right_sum = ((gMotor2.last_encoder << 2) | right_encoder) & 0xF; 
    if (right_sum == 0b1101 || right_sum == 0b0100 || right_sum == 0b0010 || right_sum == 0b1011)
    {
        gMotor2.current_ticks--;
    }
    else if (right_sum == 0b1110 || right_sum == 0b0111 || right_sum == 0b0001 || right_sum == 0b1000)
    {
        gMotor2.current_ticks++;
    }

    gMotor2.last_encoder = right_encoder; //store this value for next time
}

void update_left_encoder()
{
    //converting the 2 pin value to single number
    left_encoder = (digitalRead(MOTOR1_ENCODER_A) << 1) | digitalRead(MOTOR1_ENCODER_B);

    //adding it to the previous encoded value
    left_sum = ((gMotor1.last_encoder << 2) | left_encoder) & 0xF; 
    if (left_sum == 0b1101 || left_sum == 0b0100 || left_sum == 0b0010 || left_sum == 0b1011)
    {
        gMotor1.current_ticks++;   
    }
    else if (left_sum == 0b1110 || left_sum == 0b0111 || left_sum == 0b0001 || left_sum == 0b1000)
    {
        gMotor1.current_ticks--;
    }

    gMotor1.last_encoder = left_encoder; //store this value for next time
}

static void configure_pwm(motor_t* motor)
{
    // Set up the PWM outputs
    // The PWM frequency should be 10 kHz

    pinMode(motor->forward_pin,OUTPUT);
    pinMode(motor->backward_pin,OUTPUT);
}

static void get_ticks(motor_t* motor)
{

    // Calculate the change in motor position
    motor->delta_ticks = motor->current_ticks - motor->last_ticks;

    // Store last tick position for next iteration
    motor->last_ticks = motor->current_ticks;

}

static void calculate_forward_kinematics(float delta1, float delta2, odom_t* odom)
{

    odom->x_vel = (-0.008125 * delta1) + (-0.008125 * delta2);
    odom->y_vel = 0;
    odom->omega = (0.44521 * delta1) + (-0.44521 * delta2);

}

static void get_wheel_vel(float xVel, float yVel, float thetaVel)
{
    LOCK_SEMAPHORE(gMotor1.mutex);
    gMotor1.motor_set_speed = (61.53846 * xVel) + (-2.2462 * thetaVel);
    gMotor1.motor_pid_speed = gMotor1.motor_set_speed;
    UNLOCK_SEMAPHORE(gMotor1.mutex);

    LOCK_SEMAPHORE(gMotor2.mutex);
    gMotor2.motor_set_speed = (61.53846 * xVel) + (2.2462 * thetaVel);
    gMotor2.motor_pid_speed = gMotor2.motor_set_speed;
    UNLOCK_SEMAPHORE(gMotor2.mutex);
}

static void calculate_radian_change(motor_t* motor, float time)
{

    // Get the number of rotations since the last call
    motor->motor_speed = ((float) motor->delta_ticks) / ((float) (TICK_PER_ROTATION * GEAR_RATIO));

    // Convert rotations into radians
    motor->motor_speed = motor->motor_speed * 2.0 * PI;
    // Convert radian change to radians per second
    motor->motor_speed = motor->motor_speed / time;
}

static void update_odom(odom_t* robot_odom)
{
    
    // Get the time
    robot_odom->time = pdMS_TO_TICKS(xTaskGetTickCount()) / 1000.0;
    robot_odom->delta_time = robot_odom->time - robot_odom->last_time;
    robot_odom->last_time = robot_odom->time;

    // Calculate the number of radians since the last call
    for ( int i = 0; i < NUM_MOTORS; i++)
    {
        get_ticks(gMotorList[i]);
        calculate_radian_change(gMotorList[i],robot_odom->delta_time);
    }

    // Convert radian change to x,y,omega velocity in meters per second
    calculate_forward_kinematics(gMotor1.motor_speed, gMotor2.motor_speed, &gRobotOdom);

    // Calculate X, Y, Omega position
    robot_odom->x = robot_odom->x_vel * robot_odom->delta_time;
    robot_odom->y = robot_odom->y_vel * robot_odom->delta_time;
    robot_odom->theta = fmod((robot_odom->omega * robot_odom->delta_time), (2.0 * PI));

#if defined(DEBUGODOM)
    static int count = 0;
    if ((count % 1000) == 0)
    {
        DEBUG_PRINTF("Motor 1 Ticks: %d | Motor 2 Ticks: %d | Delta: %f | X Vel: %f | Y Vel: %f | Omega: %f", 
            gMotor1.delta_ticks, gMotor2.delta_ticks, robot_odom->delta_time, robot_odom->x_vel, robot_odom->y_vel,robot_odom->omega);
    }
#endif

}

static void motor_set_velocity(motor_t* motor, float vel)
{
    // Speed is the magnitude of the wheel rotation as a PWM duty cycle.
    // The max duty cycle is 66535
    // Max radians per second, the vel argument unit, is defined with MAX_RADIANS_SEC
    uint16_t speed = (uint16_t) abs((vel * MAX_PWM) / MAX_RADIANS_SEC);

    if (0 == motor->motor_speed)
    {
        if (0 < speed)
        {
            speed += MOTOR_FEED_FORWARD;
        }
        else
        {
            speed -= MOTOR_FEED_FORWARD;
        }
    }

    // Bound the speed to MAX_PWM
    speed = speed > MAX_PWM ? MAX_PWM : speed;

    // Set the speed and direction of the motor
    if (vel > 0)
    {
        analogWrite(motor->backward_pin, 0);
        analogWrite(motor->forward_pin, speed);
    }
    else
    {
        analogWrite(motor->forward_pin, 0);
        analogWrite(motor->backward_pin, speed);
    }
}

static int _set_velocity(uint16_t length, void* args)
{
    float xVel = 0;
    float yVel = 0;
    float phi = 0;

    if (12 != length)
    {
        return -1;
    }

    memcpy(&xVel, ((float*) args), sizeof(float));

    memcpy(&yVel, ((float*) args) + 1, sizeof(float));
    
    memcpy(&phi, ((float*) args) + 2, sizeof(float));

    set_velocity(xVel,yVel,phi);

    return 0;
}

void set_velocity(float xVel, float yVel, float thetaVel)
{
    DEBUG_PRINTF("X Vel: %f | Y Vel: %f | Omega: %f",xVel,yVel,thetaVel);
   
    get_wheel_vel(xVel,yVel,thetaVel);
	
    for (int i = 0; i < NUM_MOTORS; i++)
    {
    	DEBUG_PRINTF("Motor %i: %f", i, gMotorList[i]->motor_set_speed);
    }

    motor_set_velocity(&gMotor1, gMotor1.motor_set_speed);
    motor_set_velocity(&gMotor2, gMotor2.motor_set_speed);

}

void send_odom(odom_t* data)
{
    data->x_vel = gRobotOdom.x_vel;
    data->y_vel = gRobotOdom.y_vel;
    data->omega = gRobotOdom.omega;
    data->x = gRobotOdom.x;
    data->y = gRobotOdom.y;
    data->theta = gRobotOdom.theta;
}

void init_motor_control()
{
    // Initialize the odom struct
    memset(&gRobotOdom, 0, sizeof(gRobotOdom));

    // Initialize the motor_data_structs
    memset(&gMotor1, 0, sizeof(gMotor1));
    gMotor1.P = 1.5;
    gMotor1.I = 0;
    gMotor1.D = 0.75;
    gMotor1.forward_pin = MOTOR1_FORWARD;
    gMotor1.backward_pin = MOTOR1_BACKWARD;
    gMotor1.mutex = xSemaphoreCreateMutex();

    memset(&gMotor2, 0, sizeof(gMotor2));
    gMotor2.P = 1.5;
    gMotor2.I = 0;
    gMotor2.forward_pin = MOTOR2_FORWARD;
    gMotor2.backward_pin = MOTOR2_BACKWARD;
    gMotor2.mutex = xSemaphoreCreateMutex();

    configure_pwm(&gMotor1);
    configure_pwm(&gMotor2);

    pinMode(MOTOR1_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR1_ENCODER_B, INPUT_PULLUP);

    pinMode(MOTOR2_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR2_ENCODER_B, INPUT_PULLUP);

    attachInterrupt(MOTOR2_ENCODER_A,update_right_encoder,CHANGE);
    attachInterrupt(MOTOR2_ENCODER_B,update_right_encoder,CHANGE);  

    attachInterrupt(MOTOR1_ENCODER_A,update_left_encoder,CHANGE);
    attachInterrupt(MOTOR1_ENCODER_B,update_left_encoder,CHANGE);  

    ROUTER_REGISTER(0x0, _set_velocity);
}


static void update_pid()
{

    // Updated the velocity of each motor
    for ( int i = 0; i < 1; i++)
    {
        calculate_correction_factor(gMotorList[i], &gRobotOdom);
#if not defined(TIMESTATS)
        DEBUG_PRINTF("Motor %i PID Speed: %f | Motor %i Correction %f", i, gMotorList[i]->motor_pid_speed, i, gMotorList[i]->correction_factor);
#endif
        gMotorList[i]->motor_pid_speed += gMotorList[i]->correction_factor;
        MAX(-MAX_RADIANS_SEC,MIN(gMotorList[i]->motor_pid_speed,MAX_RADIANS_SEC));
        motor_set_velocity(gMotorList[i],gMotorList[i]->motor_pid_speed);
    }
}

void motor_task(void* parameters)
{
    DEBUG_PRINTF("Staring Motor Task");
    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t count = 0;
    while(pdTRUE)
    {
        vTaskDelayUntil(&last_wake_time, 10/portTICK_PERIOD_MS);
        DEBUG_PRINTF("Motor Running %u", count);
        count++;
        update_right_encoder();
        // update_odom(&gRobotOdom);
        // if (!(gMotorList[0]->motor_set_speed == 0 && gMotorList[1]->motor_set_speed == 0 ))
        // {
        //     update_pid();
        // }
    }

}
