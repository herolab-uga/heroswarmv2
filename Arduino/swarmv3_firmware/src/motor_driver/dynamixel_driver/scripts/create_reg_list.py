import struct
import pandas


STRUCT_TEMPLATE = '''
typedef struct
{{
    {0}
}} {1}_struct_t;
'''

PRINT_FUNCTION = '''
void print_{0}({0}_struct_t* {0})
{{
    {1}
}}
'''

REG_READ_FUNCTION = '''
void read_{0}(dynamixel_t* motor)
{{
    dynamixel_2_status_packet_t ret_status;
    dynamixel_2_instruction_packet_t {0}_read_packet;

    {0}_read_packet = 
    {{
        .id = motor->id,
        .param_length = 4,
        .instruction = READ,
    }};
    
    for (int reg = {0}_regs_enum_t::{1}; reg != {0}_regs_enum_t::LAST; reg++)
    {{
        switch (reg)
        {{
            {2}
        }};
    }}
}}
'''

ENUM_TEMPLATE = '''
typedef enum
{{
    {0}
    LAST_{1.capitalize()}
}}{1}_regs_enum_t;
'''

def create_reg_enum(table_path, struct_name):
    enum_template = "{name},"
    enum_contents = ""
    df = pandas.read_table(table_path)
    for index,reg in enumerate(df["Size(Byte) "]):
        name = df["Data Name "][index].strip().replace(" ", "_").lower()
        
        while "(" in name:
            start = name.index("(")
            end = name.index(")")
            name = name[0:start] + "_" + name[start+1:end] + name[end+1:]

        match reg:
            case 1:
                type = "uint8_t"
            case 2:
                type = "uint16_t"
            case 4:
                type = "uint32_t"
            case _:
                type = "unknown"

        enum_contents += enum_template.format(name = name)

        if not len(df["Size(Byte) "]) - 1 == index:
            enum_contents += "\n    "

    return ENUM_TEMPLATE.format(enum_contents,struct_name)

def create_reg_struct(table_path, struct_name):
    struct_contents_template = "{type} {name};"
    struct_contents = ""
    df = pandas.read_table(table_path)
    for index,reg in enumerate(df["Size(Byte) "]):
        name = df["Data Name "][index].strip().replace(" ", "_").lower()
        
        while "(" in name:
            start = name.index("(")
            end = name.index(")")
            name = name[0:start] + "_" + name[start+1:end] + name[end+1:]

        match reg:
            case 1:
                type = "uint8_t"
            case 2:
                type = "uint16_t"
            case 4:
                type = "uint32_t"
            case _:
                type = "unknown"

        struct_contents += struct_contents_template.format(type = type, name = name)

        if not len(df["Size(Byte) "]) - 1 == index:
            struct_contents += "\n    "

    return STRUCT_TEMPLATE.format(struct_contents,struct_name)

def create_reg_print(table_path, struct_name):
    print_function_contents_template = "printf(\"{field_name}: %lu\\n\\r\", {struct_name}->{struct_variable_name});"
    print_function_contents = ""

    df = pandas.read_table(table_path)
    for index,reg in enumerate(df["Size(Byte) "]):
        name = df["Data Name "][index].strip().replace(" ", "_").lower()

        while "(" in name:
                start = name.index("(")
                end = name.index(")")
                name = name[0:start] + "_" + name[start+1:end] + name[end+1:]

        print_function_contents += print_function_contents_template.format(field_name = df["Data Name "][index].strip(), 
                                                                           struct_variable_name = name,
                                                                           struct_name = struct_name)
        if not len(df["Size(Byte) "]) - 1 == index:
                print_function_contents += "\n    "
    
    return PRINT_FUNCTION.format(struct_name, print_function_contents)

def create_read_function(table_path, struct_name):
    reg_read_case = """
        case {struct_name}_regs_enum_t::{name}:
        {{
            uint8_t param_list[] = {arguments};
            {reg_type}_read_packet.param_list = param_list;
            write_cmd(&{reg_type}_read_packet);
            read_status(&ret_status);
            memcpy(&motor->{reg_type}_data.{name}, ret_status.param_list,sizeof(motor->{reg_type}_data.{name}));
            break;
        }}
        """
    print_function_contents = ""

    df = pandas.read_table(table_path)
    for index,reg in enumerate(df["Size(Byte) "]):
        arguments = "{"
        name = df["Data Name "][index].strip().replace(" ", "_").lower()

        while "(" in name:
            start = name.index("(")
            end = name.index(")")
            name = name[0:start] + "_" + name[start+1:end] + name[end+1:]

        arguments += str(hex(struct.pack("<H", df["Address "][index])[0])) + ","
        arguments += str(hex(struct.pack("<H", df["Address "][index])[1])) + ","

        match reg:
            case 1:
                arguments += "0x01,0x00"
            case 2:
                arguments += "0x02,0x00"
            case 4:
                arguments += "0x04,0x00"
            case _:
                type = "unknown"

        arguments += "}"

        print_function_contents += reg_read_case.format(name = name, arguments = arguments, reg_type = struct_name, struct_name = struct_name)
    return REG_READ_FUNCTION.format(struct_name, df["Data Name "][0].strip().replace(" ", "_").lower(),print_function_contents)

def gen_headers():
    with open("../dynamixel_headers/dynamixel_xl330_header.hpp","a+") as file:
        file.write("//This is an autogenerated file.\n #pragma once\n")
        file.write("\n\n#include <stdio.h>\n#include <stdint.h>\n")

        file.write(create_reg_enum("xl330/xl330_eeprom_register_list.tsv","eeprom"))
        file.write(create_reg_struct("xl330/xl330_eeprom_register_list.tsv","eeprom"))
        

        file.write(create_reg_enum("xl330/xl330_ram_register_list.tsv","ram"))
        file.write(create_reg_struct("xl330/xl330_ram_register_list.tsv","ram"))

def gen_reg_read():
    print(create_read_function("xl330/xl330_eeprom_register_list.tsv","eeprom"))

gen_headers()