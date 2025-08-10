#include <Arduino.h>

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "softwareserial/SoftwareSerial.h"

#include "defines.h"
#include "dynamixel_2_protocol.hpp"
#include "dynamixel_headers/dynamixel_xl330_header.hpp"

typedef enum
{
    SYNC,
    HEADER,
    DATA
} packet_reconstruction_states_t;


SoftwareSerial gDynamixelSerial;
uint8_t gControlPin = 0;
ram_struct_t gDynamixelRamStruct;
eeprom_struct_t gDynamixelEepromStruct;


const uint8_t gPacketHeader[] = {0xFF, 0xFF, 0xFD, 0x00};

void print_eeprom(dynamixel_t* motor)
{
    DEBUG_PRINTF("Model Number: %u", motor->eeprom_data.model_number);
    DEBUG_PRINTF("Model Information: %lu", motor->eeprom_data.model_information);
    DEBUG_PRINTF("Firmware Version: %u", motor->eeprom_data.firmware_version);
    DEBUG_PRINTF("ID: %u", motor->eeprom_data.id);
    DEBUG_PRINTF("Baud Rate: %u", motor->eeprom_data.baud_rate);
    DEBUG_PRINTF("Return Delay Time: %u", motor->eeprom_data.return_delay_time);
    DEBUG_PRINTF("Drive Mode: %u", motor->eeprom_data.drive_mode);
    DEBUG_PRINTF("Operating Mode: %u", motor->eeprom_data.operating_mode);
    DEBUG_PRINTF("Secondary(Shadow) ID: %u", motor->eeprom_data.secondary_shadow_id);
    DEBUG_PRINTF("Protocol Type: %u", motor->eeprom_data.protocol_type);
    DEBUG_PRINTF("Homing Offset: %lu", motor->eeprom_data.homing_offset);
    DEBUG_PRINTF("Moving Threshold: %lu", motor->eeprom_data.moving_threshold);
    DEBUG_PRINTF("Temperature Limit: %u", motor->eeprom_data.temperature_limit);
    DEBUG_PRINTF("Max Voltage Limit: %u", motor->eeprom_data.max_voltage_limit);
    DEBUG_PRINTF("Min Voltage Limit: %u", motor->eeprom_data.min_voltage_limit);
    DEBUG_PRINTF("PWM Limit: %u", motor->eeprom_data.pwm_limit);
    DEBUG_PRINTF("Current Limit: %u", motor->eeprom_data.current_limit);
    DEBUG_PRINTF("Velocity Limit: %lu", motor->eeprom_data.velocity_limit);
    DEBUG_PRINTF("Max Position Limit: %lu", motor->eeprom_data.max_position_limit);
    DEBUG_PRINTF("Min Position Limit: %lu", motor->eeprom_data.min_position_limit);
    DEBUG_PRINTF("Startup Configuration: %u", motor->eeprom_data.startup_configuration);
    DEBUG_PRINTF("PWM Slope: %u", motor->eeprom_data.pwm_slope);
    DEBUG_PRINTF("Shutdown: %u", motor->eeprom_data.shutdown);
}

void print_ram(dynamixel_t* motor)
{
    DEBUG_PRINTF("Torque Enable: %u", motor->ram_data.torque_enable);
    DEBUG_PRINTF("LED: %u", motor->ram_data.led);
    DEBUG_PRINTF("Status Return Level: %u", motor->ram_data.status_return_level);
    DEBUG_PRINTF("Registered Instruction: %u", motor->ram_data.registered_instruction);
    DEBUG_PRINTF("Hardware Error Status: %u", motor->ram_data.hardware_error_status);
    DEBUG_PRINTF("Velocity I Gain: %u", motor->ram_data.velocity_i_gain);
    DEBUG_PRINTF("Velocity P Gain: %u", motor->ram_data.velocity_p_gain);
    DEBUG_PRINTF("Position D Gain: %u", motor->ram_data.position_d_gain);
    DEBUG_PRINTF("Position I Gain: %u", motor->ram_data.position_i_gain);
    DEBUG_PRINTF("Position P Gain: %u", motor->ram_data.position_p_gain);
    DEBUG_PRINTF("Feedforward 2nd Gain: %u", motor->ram_data.feedforward_2nd_gain);
    DEBUG_PRINTF("Feedforward 1st Gain: %u", motor->ram_data.feedforward_1st_gain);
    DEBUG_PRINTF("Bus Watchdog: %u", motor->ram_data.bus_watchdog);
    DEBUG_PRINTF("Goal PWM: %u", motor->ram_data.goal_pwm);
    DEBUG_PRINTF("Goal Current: %u", motor->ram_data.goal_current);
    DEBUG_PRINTF("Goal Velocity: %lu", motor->ram_data.goal_velocity);
    DEBUG_PRINTF("Profile Acceleration: %lu", motor->ram_data.profile_acceleration);
    DEBUG_PRINTF("Profile Velocity: %lu", motor->ram_data.profile_velocity);
    DEBUG_PRINTF("Goal Position: %lu", motor->ram_data.goal_position);
    DEBUG_PRINTF("Realtime Tick: %u", motor->ram_data.realtime_tick);
    DEBUG_PRINTF("Moving: %u", motor->ram_data.moving);
    DEBUG_PRINTF("Moving Status: %u", motor->ram_data.moving_status);
    DEBUG_PRINTF("Present PWM: %u", motor->ram_data.present_pwm);
    DEBUG_PRINTF("Present Current: %u", motor->ram_data.present_current);
    DEBUG_PRINTF("Present Velocity: %lu", motor->ram_data.present_velocity);
    DEBUG_PRINTF("Present Position: %lu", motor->ram_data.present_position);
    DEBUG_PRINTF("Velocity Trajectory: %lu", motor->ram_data.velocity_trajectory);
    DEBUG_PRINTF("Position Trajectory: %lu", motor->ram_data.position_trajectory);
    DEBUG_PRINTF("Present Input Voltage: %u", motor->ram_data.present_input_voltage);
    DEBUG_PRINTF("Present Temperature: %u", motor->ram_data.present_temperature);
    DEBUG_PRINTF("Backup Ready: %u", motor->ram_data.backup_ready);
    DEBUG_PRINTF("Indirect Address 1: %u", motor->ram_data.indirect_address_1);
    DEBUG_PRINTF("Indirect Address 2: %u", motor->ram_data.indirect_address_2);
    DEBUG_PRINTF("Indirect Address 3: %u", motor->ram_data.indirect_address_3);
    DEBUG_PRINTF("Indirect Address 18: %u", motor->ram_data.indirect_address_18);
    DEBUG_PRINTF("Indirect Address 19: %u", motor->ram_data.indirect_address_19);
    DEBUG_PRINTF("Indirect Address 20: %u", motor->ram_data.indirect_address_20);
    DEBUG_PRINTF("Indirect Data 1: %u", motor->ram_data.indirect_data_1);
    DEBUG_PRINTF("Indirect Data 2: %u", motor->ram_data.indirect_data_2);
    DEBUG_PRINTF("Indirect Data 3: %u", motor->ram_data.indirect_data_3);
    DEBUG_PRINTF("Indirect Data 18: %u", motor->ram_data.indirect_data_18);
    DEBUG_PRINTF("Indirect Data 19: %u", motor->ram_data.indirect_data_19);
    DEBUG_PRINTF("Indirect Data 20: %u", motor->ram_data.indirect_data_20);
}

static uint16_t update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    DEBUG_PRINTF("Calculating CRC");
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

size_t write_cmd(dynamixel_2_instruction_packet_t* instruction, dynamixel_2_status_packet_t* status)
{
    DEBUG_PRINTF("Writing Command:");
    ssize_t ret = 0;
    uint16_t crc = 0;
    size_t command_len = 0;
    uint16_t total_length = instruction->param_length + 3;
    static uint8_t command[MAX_BUFFER_SIZE];

    if (instruction->param_length > MAX_PARAM_LENGTH)
    {
        DEBUG_PRINTF("Too many parameters.");
        return -1; // Add a function error enum
    }

    // Zero out the command buffer
    memset(command, 0, sizeof(command));

    // Begin constructin packet
    // Copy the header
    memcpy(&command[command_len], gPacketHeader, sizeof(gPacketHeader));
    command_len += sizeof(gPacketHeader);

    // Copy the struct into the buffer (this will not work cause i have a pointer in the stuct)
    memcpy(&command[command_len], &instruction->id ,sizeof(instruction->id));
    command_len += sizeof(instruction->id);

    memcpy(&command[command_len], &total_length,sizeof(total_length));
    command_len += sizeof(total_length);

    memcpy(&command[command_len], &instruction->instruction,sizeof(instruction->instruction));
    command_len += sizeof(instruction->instruction);

    for ( uint16_t i = 0; i < instruction->param_length; i++)
    {
        command[command_len] = instruction->param_list[i];
        command_len += 1;
    }


    // Calculate the CRC
    crc = update_crc(0, command, command_len);
    memcpy(&command[command_len], &crc, sizeof(crc));

    command_len += 2;

    digitalWrite(gControlPin, HIGH);

    DEBUG_PRINTF("Writing Command Length %u:", command_len);
#if defined(DEBUG)
    for (uint32_t i = 0; i < command_len; i++)
    {
        Serial.printf("%2X ", command[i]);
    }
    Serial.printf("\n");
#endif
    ret = gDynamixelSerial.write_buffer(command, command_len);

    // This cast is fine, command_len will never be greater than 519
    if (ret != (ssize_t) command_len)
    {
        return -1;
    }

    digitalWrite(gControlPin, LOW);

    // return read_status(status);
    delay(1000);
    return 0;

}

int8_t read_status(dynamixel_2_status_packet_t* status)
{
    uint16_t cacl_crc = 0;
    uint16_t packet_crc = 0;
    uint16_t param_length = 0;
    ssize_t buffer_length = 0;
    packet_reconstruction_states_t state = SYNC;
    uint8_t buffer[MAX_BUFFER_SIZE];

    memset(buffer, 0 , sizeof(buffer));

    switch (state)
    {
        case SYNC:
            digitalWrite(gControlPin, LOW);
            gDynamixelSerial.readBytes(buffer, sizeof(gPacketHeader));
            while (0 != memcmp(buffer, gPacketHeader, sizeof(gPacketHeader)))
            {
                memmove(buffer, buffer + 1, sizeof(gPacketHeader) - 1);
                digitalWrite(gControlPin, LOW);
                buffer[3] = gDynamixelSerial.read();
            }
            buffer_length = 4;
            state = HEADER;
        case HEADER:
            // Read the packet ID
            digitalWrite(gControlPin, LOW);
            buffer[buffer_length] = gDynamixelSerial.read();
            buffer_length++;
            // Read the pacekt length
            digitalWrite(gControlPin, LOW);
            gDynamixelSerial.readBytes(&buffer[buffer_length], 2);
            memcpy(&param_length, &buffer[buffer_length], 2);
            buffer_length += 2;
            state = DATA;
        case DATA:
            digitalWrite(gControlPin, LOW);
            gDynamixelSerial.readBytes(&buffer[buffer_length], param_length);
            buffer_length += param_length;
            break;
        default:
            break;
    }

    memcpy(&packet_crc, &buffer[buffer_length - sizeof(packet_crc)], sizeof(packet_crc));
    cacl_crc = update_crc(0, buffer, buffer_length - sizeof(packet_crc));

    if (cacl_crc != packet_crc)
    {
        DEBUG_PRINTF("CRC error");
        return -1;
    }
    else
    {
        if (NULL != status)
        {
            memcpy(status, &buffer[sizeof(gPacketHeader)], 4);
            status->param_list = &buffer[9];
            memcpy(&status->param_length, &buffer[5], sizeof(status->param_length));
            status->param_length -= 2; 
            status->crc = packet_crc;
            status->error = (dynamixel_2_errors_t) buffer[8];
            status->instruction = buffer[7];
        }
    }
    return 0;
}

// read_eeprom

void read_eeprom(dynamixel_t* motor)
{   
    DEBUG_PRINTF("Reading EEPROM");
    dynamixel_2_status_packet_t ret_status;
    dynamixel_2_instruction_packet_t eeprom_read_packet;

    eeprom_read_packet = 
    {
        .id = motor->id,
        .param_length = 4,
        .instruction = READ,
    };
    
    for (int reg = eeprom_regs_enum_t::model_number; reg != eeprom_regs_enum_t::LAST_EEPROM; reg++)
    {
        memset(&ret_status, 0, sizeof(ret_status));
        switch (reg)
        {

        case eeprom_regs_enum_t::model_number:
        {
            uint8_t param_list[] = {0x0,0x0,0x02,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.model_number, ret_status.param_list,sizeof(motor->eeprom_data.model_number));
            break;
        }
        
        case eeprom_regs_enum_t::model_information:
        {
            uint8_t param_list[] = {0x2,0x0,0x04,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.model_information, ret_status.param_list,sizeof(motor->eeprom_data.model_information));
            break;
        }
        
        case eeprom_regs_enum_t::firmware_version:
        {
            uint8_t param_list[] = {0x6,0x0,0x01,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.firmware_version, ret_status.param_list,sizeof(motor->eeprom_data.firmware_version));
            break;
        }
        
        case eeprom_regs_enum_t::id:
        {
            uint8_t param_list[] = {0x7,0x0,0x01,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.id, ret_status.param_list,sizeof(motor->eeprom_data.id));
            break;
        }
        
        case eeprom_regs_enum_t::baud_rate:
        {
            uint8_t param_list[] = {0x8,0x0,0x01,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.baud_rate, ret_status.param_list,sizeof(motor->eeprom_data.baud_rate));
            break;
        }
        
        case eeprom_regs_enum_t::return_delay_time:
        {
            uint8_t param_list[] = {0x9,0x0,0x01,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.return_delay_time, ret_status.param_list,sizeof(motor->eeprom_data.return_delay_time));
            break;
        }
        
        case eeprom_regs_enum_t::drive_mode:
        {
            uint8_t param_list[] = {0xa,0x0,0x01,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.drive_mode, ret_status.param_list,sizeof(motor->eeprom_data.drive_mode));
            break;
        }
        
        case eeprom_regs_enum_t::operating_mode:
        {
            uint8_t param_list[] = {0xb,0x0,0x01,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.operating_mode, ret_status.param_list,sizeof(motor->eeprom_data.operating_mode));
            break;
        }
        
        case eeprom_regs_enum_t::secondary_shadow_id:
        {
            uint8_t param_list[] = {0xc,0x0,0x01,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.secondary_shadow_id, ret_status.param_list,sizeof(motor->eeprom_data.secondary_shadow_id));
            break;
        }
        
        case eeprom_regs_enum_t::protocol_type:
        {
            uint8_t param_list[] = {0xd,0x0,0x01,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.protocol_type, ret_status.param_list,sizeof(motor->eeprom_data.protocol_type));
            break;
        }
        
        case eeprom_regs_enum_t::homing_offset:
        {
            uint8_t param_list[] = {0x14,0x0,0x04,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.homing_offset, ret_status.param_list,sizeof(motor->eeprom_data.homing_offset));
            break;
        }
        
        case eeprom_regs_enum_t::moving_threshold:
        {
            uint8_t param_list[] = {0x18,0x0,0x04,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.moving_threshold, ret_status.param_list,sizeof(motor->eeprom_data.moving_threshold));
            break;
        }
        
        case eeprom_regs_enum_t::temperature_limit:
        {
            uint8_t param_list[] = {0x1f,0x0,0x01,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.temperature_limit, ret_status.param_list,sizeof(motor->eeprom_data.temperature_limit));
            break;
        }
        
        case eeprom_regs_enum_t::max_voltage_limit:
        {
            uint8_t param_list[] = {0x20,0x0,0x02,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.max_voltage_limit, ret_status.param_list,sizeof(motor->eeprom_data.max_voltage_limit));
            break;
        }
        
        case eeprom_regs_enum_t::min_voltage_limit:
        {
            uint8_t param_list[] = {0x22,0x0,0x02,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.min_voltage_limit, ret_status.param_list,sizeof(motor->eeprom_data.min_voltage_limit));
            break;
        }
        
        case eeprom_regs_enum_t::pwm_limit:
        {
            uint8_t param_list[] = {0x24,0x0,0x02,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.pwm_limit, ret_status.param_list,sizeof(motor->eeprom_data.pwm_limit));
            break;
        }
        
        case eeprom_regs_enum_t::current_limit:
        {
            uint8_t param_list[] = {0x26,0x0,0x02,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.current_limit, ret_status.param_list,sizeof(motor->eeprom_data.current_limit));
            break;
        }
        
        case eeprom_regs_enum_t::velocity_limit:
        {
            uint8_t param_list[] = {0x2c,0x0,0x04,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.velocity_limit, ret_status.param_list,sizeof(motor->eeprom_data.velocity_limit));
            break;
        }
        
        case eeprom_regs_enum_t::max_position_limit:
        {
            uint8_t param_list[] = {0x30,0x0,0x04,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.max_position_limit, ret_status.param_list,sizeof(motor->eeprom_data.max_position_limit));
            break;
        }
        
        case eeprom_regs_enum_t::min_position_limit:
        {
            uint8_t param_list[] = {0x34,0x0,0x04,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.min_position_limit, ret_status.param_list,sizeof(motor->eeprom_data.min_position_limit));
            break;
        }
        
        case eeprom_regs_enum_t::startup_configuration:
        {
            uint8_t param_list[] = {0x3c,0x0,0x01,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.startup_configuration, ret_status.param_list,sizeof(motor->eeprom_data.startup_configuration));
            break;
        }
        
        case eeprom_regs_enum_t::pwm_slope:
        {
            uint8_t param_list[] = {0x3e,0x0,0x01,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.pwm_slope, ret_status.param_list,sizeof(motor->eeprom_data.pwm_slope));
            break;
        }
        
        case eeprom_regs_enum_t::shutdown:
        {
            uint8_t param_list[] = {0x3f,0x0,0x01,0x00};
            eeprom_read_packet.param_list = param_list;
            write_cmd(&eeprom_read_packet, &ret_status);
            memcpy(&motor->eeprom_data.shutdown, ret_status.param_list,sizeof(motor->eeprom_data.shutdown));
            break;
        }
        
        };
    }
}




// read_ram
void read_ram(dynamixel_t* motor)
{
    DEBUG_PRINTF("Reading RAM");
    dynamixel_2_status_packet_t ret_status;
    dynamixel_2_instruction_packet_t ram_read_packet;

    ram_read_packet = 
    {
        .id = motor->id,
        .param_length = 4,
        .instruction = READ,
    };
    
    for (int reg = ram_regs_enum_t::torque_enable; reg != LAST_RAM; reg++)
    {
        switch (reg)
        {
            
        case torque_enable:
        {
            uint8_t param_list[] = {0x40,0x0,0x01,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.torque_enable, ret_status.param_list,sizeof(motor->ram_data.torque_enable));
            break;
        }
        
        case led:
        {
            uint8_t param_list[] = {0x41,0x0,0x01,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.led, ret_status.param_list,sizeof(motor->ram_data.led));
            break;
        }
        
        case status_return_level:
        {
            uint8_t param_list[] = {0x44,0x0,0x01,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.status_return_level, ret_status.param_list,sizeof(motor->ram_data.status_return_level));
            break;
        }
        
        case registered_instruction:
        {
            uint8_t param_list[] = {0x45,0x0,0x01,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.registered_instruction, ret_status.param_list,sizeof(motor->ram_data.registered_instruction));
            break;
        }
        
        case hardware_error_status:
        {
            uint8_t param_list[] = {0x46,0x0,0x01,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.hardware_error_status, ret_status.param_list,sizeof(motor->ram_data.hardware_error_status));
            break;
        }
        
        case velocity_i_gain:
        {
            uint8_t param_list[] = {0x4c,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.velocity_i_gain, ret_status.param_list,sizeof(motor->ram_data.velocity_i_gain));
            break;
        }
        
        case velocity_p_gain:
        {
            uint8_t param_list[] = {0x4e,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.velocity_p_gain, ret_status.param_list,sizeof(motor->ram_data.velocity_p_gain));
            break;
        }
        
        case position_d_gain:
        {
            uint8_t param_list[] = {0x50,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.position_d_gain, ret_status.param_list,sizeof(motor->ram_data.position_d_gain));
            break;
        }
        
        case position_i_gain:
        {
            uint8_t param_list[] = {0x52,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.position_i_gain, ret_status.param_list,sizeof(motor->ram_data.position_i_gain));
            break;
        }
        
        case position_p_gain:
        {
            uint8_t param_list[] = {0x54,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.position_p_gain, ret_status.param_list,sizeof(motor->ram_data.position_p_gain));
            break;
        }
        
        case feedforward_2nd_gain:
        {
            uint8_t param_list[] = {0x58,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.feedforward_2nd_gain, ret_status.param_list,sizeof(motor->ram_data.feedforward_2nd_gain));
            break;
        }
        
        case feedforward_1st_gain:
        {
            uint8_t param_list[] = {0x5a,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.feedforward_1st_gain, ret_status.param_list,sizeof(motor->ram_data.feedforward_1st_gain));
            break;
        }
        
        case bus_watchdog:
        {
            uint8_t param_list[] = {0x62,0x0,0x01,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.bus_watchdog, ret_status.param_list,sizeof(motor->ram_data.bus_watchdog));
            break;
        }
        
        case goal_pwm:
        {
            uint8_t param_list[] = {0x64,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.goal_pwm, ret_status.param_list,sizeof(motor->ram_data.goal_pwm));
            break;
        }
        
        case goal_current:
        {
            uint8_t param_list[] = {0x66,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.goal_current, ret_status.param_list,sizeof(motor->ram_data.goal_current));
            break;
        }
        
        case goal_velocity:
        {
            uint8_t param_list[] = {0x68,0x0,0x04,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.goal_velocity, ret_status.param_list,sizeof(motor->ram_data.goal_velocity));
            break;
        }
        
        case profile_acceleration:
        {
            uint8_t param_list[] = {0x6c,0x0,0x04,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.profile_acceleration, ret_status.param_list,sizeof(motor->ram_data.profile_acceleration));
            break;
        }
        
        case profile_velocity:
        {
            uint8_t param_list[] = {0x70,0x0,0x04,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.profile_velocity, ret_status.param_list,sizeof(motor->ram_data.profile_velocity));
            break;
        }
        
        case goal_position:
        {
            uint8_t param_list[] = {0x74,0x0,0x04,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.goal_position, ret_status.param_list,sizeof(motor->ram_data.goal_position));
            break;
        }
        
        case realtime_tick:
        {
            uint8_t param_list[] = {0x78,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.realtime_tick, ret_status.param_list,sizeof(motor->ram_data.realtime_tick));
            break;
        }
        
        case moving:
        {
            uint8_t param_list[] = {0x7a,0x0,0x01,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.moving, ret_status.param_list,sizeof(motor->ram_data.moving));
            break;
        }
        
        case moving_status:
        {
            uint8_t param_list[] = {0x7b,0x0,0x01,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.moving_status, ret_status.param_list,sizeof(motor->ram_data.moving_status));
            break;
        }
        
        case present_pwm:
        {
            uint8_t param_list[] = {0x7c,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.present_pwm, ret_status.param_list,sizeof(motor->ram_data.present_pwm));
            break;
        }
        
        case present_current:
        {
            uint8_t param_list[] = {0x7e,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.present_current, ret_status.param_list,sizeof(motor->ram_data.present_current));
            break;
        }
        
        case present_velocity:
        {
            uint8_t param_list[] = {0x80,0x0,0x04,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.present_velocity, ret_status.param_list,sizeof(motor->ram_data.present_velocity));
            break;
        }
        
        case present_position:
        {
            uint8_t param_list[] = {0x84,0x0,0x04,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.present_position, ret_status.param_list,sizeof(motor->ram_data.present_position));
            break;
        }
        
        case velocity_trajectory:
        {
            uint8_t param_list[] = {0x88,0x0,0x04,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.velocity_trajectory, ret_status.param_list,sizeof(motor->ram_data.velocity_trajectory));
            break;
        }
        
        case position_trajectory:
        {
            uint8_t param_list[] = {0x8c,0x0,0x04,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.position_trajectory, ret_status.param_list,sizeof(motor->ram_data.position_trajectory));
            break;
        }
        
        case present_input_voltage:
        {
            uint8_t param_list[] = {0x90,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.present_input_voltage, ret_status.param_list,sizeof(motor->ram_data.present_input_voltage));
            break;
        }
        
        case present_temperature:
        {
            uint8_t param_list[] = {0x92,0x0,0x01,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.present_temperature, ret_status.param_list,sizeof(motor->ram_data.present_temperature));
            break;
        }
        
        case backup_ready:
        {
            uint8_t param_list[] = {0x93,0x0,0x01,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.backup_ready, ret_status.param_list,sizeof(motor->ram_data.backup_ready));
            break;
        }
        
        case indirect_address_1:
        {
            uint8_t param_list[] = {0xa8,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.indirect_address_1, ret_status.param_list,sizeof(motor->ram_data.indirect_address_1));
            break;
        }
        
        case indirect_address_2:
        {
            uint8_t param_list[] = {0xaa,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.indirect_address_2, ret_status.param_list,sizeof(motor->ram_data.indirect_address_2));
            break;
        }
        
        case indirect_address_3:
        {
            uint8_t param_list[] = {0xac,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.indirect_address_3, ret_status.param_list,sizeof(motor->ram_data.indirect_address_3));
            break;
        }
        
        case indirect_address_18:
        {
            uint8_t param_list[] = {0xca,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.indirect_address_18, ret_status.param_list,sizeof(motor->ram_data.indirect_address_18));
            break;
        }
        
        case indirect_address_19:
        {
            uint8_t param_list[] = {0xcc,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.indirect_address_19, ret_status.param_list,sizeof(motor->ram_data.indirect_address_19));
            break;
        }
        
        case indirect_address_20:
        {
            uint8_t param_list[] = {0xce,0x0,0x02,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.indirect_address_20, ret_status.param_list,sizeof(motor->ram_data.indirect_address_20));
            break;
        }
        
        case indirect_data_1:
        {
            uint8_t param_list[] = {0xd0,0x0,0x01,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.indirect_data_1, ret_status.param_list,sizeof(motor->ram_data.indirect_data_1));
            break;
        }
        
        case indirect_data_2:
        {
            uint8_t param_list[] = {0xd1,0x0,0x01,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.indirect_data_2, ret_status.param_list,sizeof(motor->ram_data.indirect_data_2));
            break;
        }
        
        case indirect_data_3:
        {
            uint8_t param_list[] = {0xd2,0x0,0x01,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.indirect_data_3, ret_status.param_list,sizeof(motor->ram_data.indirect_data_3));
            break;
        }
        
        case indirect_data_18:
        {
            uint8_t param_list[] = {0xe1,0x0,0x01,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.indirect_data_18, ret_status.param_list,sizeof(motor->ram_data.indirect_data_18));
            break;
        }
        
        case indirect_data_19:
        {
            uint8_t param_list[] = {0xe2,0x0,0x01,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.indirect_data_19, ret_status.param_list,sizeof(motor->ram_data.indirect_data_19));
            break;
        }
        
        case indirect_data_20:
        {
            uint8_t param_list[] = {0xe3,0x0,0x01,0x00};
            ram_read_packet.param_list = param_list;
            write_cmd(&ram_read_packet, &ret_status);
            memcpy(&motor->ram_data.indirect_data_20, ret_status.param_list,sizeof(motor->ram_data.indirect_data_20));
            break;
        }
        
        };
    }
}


int init_dynamixel(uint8_t rx_pin, uint8_t tx_pin, uint8_t control_pin)
{
    DEBUG_PRINTF("Init Dynamixel");
    gDynamixelSerial.init(rx_pin, tx_pin);
    DEBUG_PRINTF("Begin");
    gDynamixelSerial.begin(57600);
    DEBUG_PRINTF("Set Control");
    gControlPin = control_pin;
    DEBUG_PRINTF("Set Pin mode");
    pinMode(gControlPin, OUTPUT);
    DEBUG_PRINTF("Finished Dynamixel");
    return 0;
}
