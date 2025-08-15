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

#define NUM_MOTORS              (2)

#define TX_PIN                  (12)
#define RX_PIN                  (11)
#define CONTROL_PIN             (10)

#define RPM_CONVERSION          (9.55)
#define MPS_TO_RPM              (597.0f)

uint8_t left_sum = 0;
uint8_t left_encoder = 0;

uint8_t right_sum = 0;
uint8_t right_encoder = 0;

uint32_t start = 0;

odom_t gRobotOdom;

dynamixel_t gMotor1;
dynamixel_t gMotor2;

dynamixel_t* gMotorList[] = {&gMotor1, &gMotor2};

void set_velocity(float xVel, float yVel, float thetaVel);

static void calculate_forward_kinematics(float delta1, float delta2, odom_t* odom)
{

    odom->x_vel = (-0.008125 * delta1) + (-0.008125 * delta2);
    odom->y_vel = 0;
    odom->omega = (0.44521 * delta1) + (-0.44521 * delta2);

}

// The numbers here are in radians/sec then converted to rpm
static void get_wheel_vel(float xVel, float yVel, float thetaVel)
{
    LOCK_SEMAPHORE(gMotor1.mutex);
    gMotor1.ram_data.goal_velocity = ((61.53846 * xVel) + (-2.2462 * thetaVel)) * RPM_CONVERSION;
    UNLOCK_SEMAPHORE(gMotor1.mutex);

    LOCK_SEMAPHORE(gMotor2.mutex);
    gMotor2.ram_data.goal_velocity = ((61.53846 * xVel) + (2.2462 * thetaVel)) * RPM_CONVERSION;
    UNLOCK_SEMAPHORE(gMotor2.mutex);
}

static void read_velocity(dynamixel_t* motor)
{
    dynamixel_2_status_packet_t ret_status;

    dynamixel_2_instruction_packet_t ram_read_packet = 
    {
        .id = motor->id,
        .param_length = 4,
        .instruction = READ,
    };

    uint8_t param_list[] = {0x80,0x0,0x04,0x00};
    ram_read_packet.param_list = param_list;
    write_cmd(&ram_read_packet, &ret_status);
    memcpy(&motor->ram_data.present_velocity, ret_status.param_list,sizeof(motor->ram_data.present_velocity));
}

static void update_odom(odom_t* robot_odom)
{  
    digitalWrite(A3, HIGH);
    // Read the current speed of each motor not the entire ram table
    read_velocity(&gMotor1);
    read_velocity(&gMotor2);
    
    // Get the time
    robot_odom->time = pdMS_TO_TICKS(xTaskGetTickCount()) / 1000.0;
    robot_odom->delta_time = robot_odom->time - robot_odom->last_time;
    robot_odom->last_time = robot_odom->time;

    // Convert radian change to x,y,omega velocity in meters per second
    calculate_forward_kinematics(gMotor1.ram_data.present_velocity * RPM_CONVERSION, 
        gMotor2.ram_data.present_velocity * RPM_CONVERSION, &gRobotOdom);

    // Calculate X, Y, Omega position
    robot_odom->x = robot_odom->x_vel * robot_odom->delta_time;
    robot_odom->y = robot_odom->y_vel * robot_odom->delta_time;
    robot_odom->theta = fmod((robot_odom->omega * robot_odom->delta_time), (2.0 * PI));

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

    int32_t vel_conv = (int) (((vel) * 445)/101);

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
    	DEBUG_PRINTF("Motor %i: %f", i, gMotorList[i]->ram_data.goal_velocity);
    }
#endif 

    motor_set_velocity(&gMotor1, gMotor1.ram_data.goal_velocity);
    motor_set_velocity(&gMotor2, gMotor2.ram_data.goal_velocity);

    dynamixel_2_instruction_packet_t sync_action_instruction =
    {
        .id = BROADCAST,
        .param_length = 0,
        .instruction = ACTION,
        .param_list = NULL,
    };

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
    
    dynamixel_2_instruction_packet_t init_write =
    {
        .id = motor->id,
    };

    write_cmd(&init_write);

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
    gMotor1 = 
    {
        .id = 1,
    };

    gMotor1.mutex = xSemaphoreCreateMutex();

    gMotor2 =
    {
        .id = 2,
    };

    gMotor2.mutex = xSemaphoreCreateMutex();

    init_motor(&gMotor1);
    init_motor(&gMotor2);

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
        vTaskDelayUntil(&last_wake_time, 10);
    }

}