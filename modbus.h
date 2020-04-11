#ifndef MODBUS_H
#define MODBUS_H

#include "si_toolchain.h"
#include "F120_FlashPrimitives.h"

#define MODBUS_RECEIVER_LENGTH 1024
#define MODBUS_TRANSMITTER_LENGTH 1024
#define MODBUS_DATA_LENGTH 3000//4096
#define MODBUS_ERROR_CODE 128

#define MODBUS_ADDRESS 0
#define MODBUS_ADDRESS_IN_MEMORY 2497
#define MODBUS_FUNCTION 1
#define MODBUS_FIRST_REGISTER_HI 2
#define MODBUS_FIRST_REGISTER_LO 3
#define MODBUS_NUMBER_OF_REGISTER_TO_READ_HI 4
#define MODBUS_NUMBER_OF_REGISTER_TO_READ_LO 5
#define MODBUS_NUMBER_OF_REGISTER_TO_WRITE_HI 4
#define MODBUS_NUMBER_OF_REGISTER_TO_WRITE_LO 5
#define MODBUS_ERROR 1
#define MODBUS_EXCEPTION 2
#define MODBUS_ERROR_ILLEGAL_FUNCTION_CODE 1
#define MODBUS_ERROR_ILLEGAL_DATA_ADDRESS 2
#define MODBUS_WRITE_REGISTER_OFFSET 7
#define MODBUS_FUNCTION_16_BASE_LENGTH 9
#define MODBUS_BROADCAST_ADDRESS 255
#define MODBUS_REFRESH_FLASH_MEMORY_ADDRESS_0 2812
#define MODBUS_REFRESH_FLASH_MEMORY_ADDRESS_1 2813
#define MODBUS_REFRESH_FLASH_MEMORY_ADDRESS_2 2814
#define MODBUS_REFRESH_FLASH_MEMORY_ADDRESS_3 2815
#define MODBUS_FLASH_ADDRESS 0x10000

uint16_t crc16_update(uint16_t crc, uint8_t a);

bool modbus_command_received();

int get_modbus_receiver_counter();

void set_modbus_receiver_counter(int mrc);

void inc_modbus_receiver_counter();

void modbus_byte_receive(char input);

void modbus_transmit_byte();

void modbus_push_transmit_buffer(char output);
 
bool modbus_was_sendind_received();

void modbus_init_from_flash();

uint8_t * getModbusBufferData();

bool modbus_transmit_buffer_is_empty();

#endif