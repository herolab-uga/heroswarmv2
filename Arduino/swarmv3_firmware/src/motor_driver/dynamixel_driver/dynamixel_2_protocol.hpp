#include <stdio.h>
#include <stdint.h>
#include "dynamixel_headers/dynamixel_xl330_header.hpp"

#include <FreeRTOS.h>
#include "semphr.h"

// Protocol Reference: https://emanual.robotis.com/docs/en/dxl/protocol2/

#define MAX_PACKET_LENGTH 512
#define MAX_PARAM_LENGTH (MAX_PACKET_LENGTH - 3)
#define MAX_BUFFER_SIZE (MAX_PACKET_LENGTH + 7)

typedef enum
{
    BAUDRATE_9600,
    BAUDRATE_57600,
    BAUDRATE_115200,
    BAUDRATE_1M,
    BAUDRATE_2M,
    BAUDRATE_3M,
    BAUDRATE_4M,
} baudrates_t;

// Dynamixel Instructions
typedef enum
{
    PING = 0x1,
    READ,
    WRITE,
    REG_WRITE,
    ACTION,
    FACTORY_RESET,
    REBOOT = 0x08,
    CLEAR = 0x10,
    CONTROL_TABLE_BACKUP = 0x20,
    RETURN_STATUS = 0x55,
    SYNC_READ = 0x82,
    SYNC_WRITE = 0x83,
    FAST_SYNC_READ = 0x8A,
    BULK_READ = 0x92,
    BULD_WRITE,
    FAST_BULK_READ = 0x9A,
} dynamixel_2_instructions;

typedef struct 
{
    uint8_t id;
    uint16_t param_length; // This is sent little endian
    dynamixel_2_instructions instruction;
    uint8_t* param_list;
    uint16_t crc;
} dynamixel_2_instruction_packet_t;

// Dynamixel Error types
typedef enum
{
    NO_ERROR,
    FAIL,
    INSTRUCTION_ERROR,
    CRC_ERROR,
    DATA_RANGE_ERROR,
    DATA_LENGTH_ERROR,
    DATA_LIMIT_ERROR,
    ACCESS_ERROR
} dynamixel_2_errors_t;

typedef struct
{
    uint8_t id;
    uint16_t param_length; // This is sent little endian
    uint8_t instruction;
    dynamixel_2_errors_t error;
    uint8_t* param_list;
    uint16_t crc;
} dynamixel_2_status_packet_t;

typedef struct
{
    uint8_t id;
    eeprom_struct_t eeprom_data;
    ram_struct_t ram_data;
    SemaphoreHandle_t mutex;
} dynamixel_t;


int32_t write_cmd(dynamixel_2_instruction_packet_t* instruction, dynamixel_2_status_packet_t* status = NULL);
int8_t read_status(dynamixel_2_status_packet_t* status);
void read_eeprom(dynamixel_t* motor);
void print_eeprom(dynamixel_t* motor);
void read_ram(dynamixel_t* motor);
void print_ram(dynamixel_t* motor);
int init_dynamixel(uint8_t rx_pin, uint8_t tx_pin);

