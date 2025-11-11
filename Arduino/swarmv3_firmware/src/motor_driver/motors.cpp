#include "math.h"
#include <stdio.h>
#include "stdlib.h"
#include "router.h"
#include <string.h>
#include "defines.h"
#include "pid_controller.h"

#include "dynamixel_driver/dynamixel_2_protocol.hpp"

#include <Arduino.h>
#include <FreeRTOS.h>
#include "semphr.h"
#include <task.h>
#include "softwareserial/SoftwareSerial.hpp"

// Motor Webpage: https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/

#define NUM_MOTORS                      (2)

#define TX_PIN                          (12)
#define RX_PIN                          (11)
#define CONTROL_PIN                     (10)

#define RPM_TO_RADS                     ((2.0*3.14)/60.0)
#define RPM_CONVERSION                  (9.55f)

#define MAX_RPM                         (103.0f)
#define MAX_PWM                         (445.0f)
#define PRESENT_VELOCITY_TO_RAD_SEC     (0.023981f)

typedef struct
{
    dynamixel_t dynamixel;
    float set_velocity;
} motor_t;

uint8_t left_sum = 0;
uint8_t left_encoder = 0;

uint8_t right_sum = 0;
uint8_t right_encoder = 0;

uint32_t start = 0;

odom_t gRobotOdom;

motor_t gMotor1;
motor_t gMotor2;

motor_t* gMotorList[] = {&gMotor1, &gMotor2};

void set_velocity(float xVel, float yVel, float thetaVel);

static void calculate_forward_kinematics(float delta1, float delta2, odom_t* odom)
{
    delta1 = delta1 * PRESENT_VELOCITY_TO_RAD_SEC;
    delta2 = delta2 * PRESENT_VELOCITY_TO_RAD_SEC;
    odom->x_vel = (0.008125 * (delta1)) - (0.008125 * delta2);
    odom->y_vel = 0;
    odom->omega = (-0.44521 * delta1) - (0.44521 * delta2);

}

// The numbers here are in radians/sec then converted to rpm
static void get_wheel_vel(float xVel, float yVel, float thetaVel)
{

    LOCK_SEMAPHORE(gMotor1.dynamixel.mutex);
    gMotor1.set_velocity = (((61.53846 * xVel) + (-2.2462 * thetaVel)));
    gMotor1.set_velocity = MAX(-MAX_RPM,MIN((gMotor1.set_velocity * RPM_CONVERSION),MAX_RPM));
    UNLOCK_SEMAPHORE(gMotor1.dynamixel.mutex);

    LOCK_SEMAPHORE(gMotor2.dynamixel.mutex);
    gMotor2.set_velocity = (((61.53846 * xVel) + (2.2462 * thetaVel))); 
    gMotor2.set_velocity = MAX(-MAX_RPM,MIN((gMotor2.set_velocity * RPM_CONVERSION),MAX_RPM));
    UNLOCK_SEMAPHORE(gMotor2.dynamixel.mutex);
}

static void read_velocity(dynamixel_t* motor)
{
    int32_t ret = 0;
    dynamixel_2_status_packet_t ret_status;

    dynamixel_2_instruction_packet_t ram_read_packet = 
    {
        .id = motor->id,
        .param_length = 4,
        .instruction = READ,
    };

    uint8_t param_list[] = {0x80,0x0,0x04,0x00};
    ram_read_packet.param_list = param_list;
    TickType_t start_time = xTaskGetTickCount();
    do
    {
        ret = write_cmd(&ram_read_packet, &ret_status);
    } while((-1 == ret ) && 
        (xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(READ_TIMEOUT_MSEC * 4));

    if (-1 != ret)
    {
        memcpy(&motor->ram_data.present_velocity, ret_status.param_list,sizeof(motor->ram_data.present_velocity));
    }
    else
    {
        motor->ram_data.present_velocity = 0;
    }
}

static void update_odom(odom_t* robot_odom)
{  
    digitalWrite(A3, HIGH);
    // Read the current speed of each motor not the entire ram table
    read_velocity(&gMotor1.dynamixel);
    read_velocity(&gMotor2.dynamixel);
    
    // Get the time
    robot_odom->time = xTaskGetTickCount() / 1000.0;
    robot_odom->delta_time = robot_odom->time - robot_odom->last_time;
    robot_odom->last_time = robot_odom->time;

    // Convert radian change to x,y,omega velocity in meters per second
    calculate_forward_kinematics((gMotor1.dynamixel.ram_data.present_velocity), 
        (gMotor2.dynamixel.ram_data.present_velocity), &gRobotOdom);

    // Calculate X, Y, Omega position
    robot_odom->x += robot_odom->x_vel * robot_odom->delta_time;
    robot_odom->y += robot_odom->y_vel * robot_odom->delta_time;
    robot_odom->theta += fmod((robot_odom->omega * robot_odom->delta_time), (2.0 * 3.14));

#if defined(DEBUGODOM)
    static int count = 0;
    if ((count % 1000) == 0)
    {
        DEBUG_PRINTF("Motor 1 Speed: %d | Motor 2 Speed: %d | Delta: %f | X Vel: %f | Y Vel: %f | Omega: %f", 
            gMotor1.ram_data.present_velocity, gMotor2.ram_data.present_velocity, robot_odom->delta_time, robot_odom->x_vel, robot_odom->y_vel,robot_odom->omega);
    }
#endif
    digitalWrite(A3, LOW);
}

// The vel input is in rpm
static void motor_set_velocity(dynamixel_t* motor, float vel, bool immediate=false)
{
    DEBUG_PRINTF("Setting motor velocity");
    // int32_t ret = vel;
    uint8_t param_list[6] = {0};

    // Set the speed and direction of the motor
    dynamixel_2_instruction_packet_t velocity_packet =
    {
        .id = motor->id,
        .param_length = sizeof(param_list),
        .instruction = immediate == false ? REG_WRITE : WRITE,
        .param_list = param_list
    };

    param_list[0] = 104;
    param_list[1] = 0x0;

    vel = MAX(-MAX_RPM,MIN((vel * RPM_CONVERSION),MAX_RPM));
    int32_t vel_conv = (int32_t) (((vel) * MAX_PWM)/MAX_RPM);
    

    memcpy(&param_list[2], &vel_conv, sizeof(vel_conv));

    write_cmd(&velocity_packet);
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
   
    get_wheel_vel(xVel,yVel,thetaVel);

#if defined(DEBUG)
    DEBUG_PRINTF("X Vel: %f | Y Vel: %f | Omega: %f",xVel,yVel,thetaVel);
    for (int i = 0; i < NUM_MOTORS; i++)
    {
    	DEBUG_PRINTF("Motor %i: %f", i, gMotorList[i]->set_velocity);
    }
#endif 

    motor_set_velocity(&gMotor1.dynamixel, gMotor1.set_velocity);
    motor_set_velocity(&gMotor2.dynamixel, gMotor2.set_velocity);

    dynamixel_2_instruction_packet_t sync_action_instruction =
    {
        .id = BROADCAST,
        .param_length = 0,
        .instruction = ACTION,
        .param_list = NULL,
    };

    DEBUG_PRINTF("Triggering Action");

    write_cmd(&sync_action_instruction, NULL);

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

static void init_motor(dynamixel_t* motor)
{
    uint8_t params[3] = {0};
    
    dynamixel_2_instruction_packet_t init_write;
    init_write.id = motor->id;
    init_write.instruction = WRITE;
    init_write.param_list = params;
    init_write.param_length = sizeof(params);

    // disable torque
    params[0] = 64;
    params[1] = 0;
    params[2] = 0;

    write_cmd(&init_write);

    // set operating mode to velocity
    params[0] = 11;
    params[1] = 0;
    params[2] = 1;

    write_cmd(&init_write);

    // Enable torque
    params[0] = 64;
    params[1] = 0;
    params[2] = 1;

    write_cmd(&init_write);

    motor->ram_data.torque_enable = 1;
}

void init_motor_control()
{
    DEBUG_PRINTF("Init Motors");
    memset(&gRobotOdom, 0, sizeof(gRobotOdom));

    // Initialize the dynamixel
    init_dynamixel(RX_PIN, TX_PIN);

    pinMode(A3, OUTPUT);
    digitalWrite(A3, LOW);

    // Initialize the odom struct
    memset(&gRobotOdom, 0, sizeof(gRobotOdom));

    dynamixel_2_instruction_packet_t reboot_write =
    {
        .id = BROADCAST,
        .param_length = 0,
        .instruction = REBOOT,
        .param_list = NULL,

    };

    write_cmd(&reboot_write);

    delay(3);

    // Initialize the dynamixel structs
    gMotor1.dynamixel.id = 1;

    gMotor1.dynamixel.mutex = xSemaphoreCreateMutex();

    gMotor2.dynamixel.id = 2;

    gMotor2.dynamixel.mutex = xSemaphoreCreateMutex();

    init_motor(&gMotor1.dynamixel);
    init_motor(&gMotor2.dynamixel);

    ROUTER_REGISTER(0x0, _set_velocity);
}

void motor_task(void* parameters)
{
    DEBUG_PRINTF("Starting Motor Task");

    TickType_t last_wake_time = xTaskGetTickCount();

    while(pdTRUE)
    {
        update_odom(&gRobotOdom);

        // why is the tick define 1000/1024? did the clock rate change
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }

}