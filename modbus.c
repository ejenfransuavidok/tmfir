#include <stdio.h>
#include "si_toolchain.h"
#include "C8051F120_defs.h"

#include "modbus.h"
#include "F120_FlashUtils.h"

int modbus_receiver_pointer = 0;
int modbus_transmitter_pointer_right = 0;
int modbus_transmitter_pointer_left = 0;
int sender_pause_timer = 0;



SI_SEGMENT_VARIABLE(modbus_command_receiver[MODBUS_RECEIVER_LENGTH], uint8_t, xdata);
SI_SEGMENT_VARIABLE(modbus_command_transmitter[MODBUS_TRANSMITTER_LENGTH], uint8_t, xdata);
SI_SEGMENT_VARIABLE(modbus_buffer_data[MODBUS_DATA_LENGTH], uint8_t, xdata);
SI_SEGMENT_VARIABLE(modbus_error_response[5], uint8_t, xdata);
void (*init_after_flash_reload_func_pointer)(void);

#pragma NOAREGS
uint8_t * getModbusBufferData() {
	return modbus_buffer_data;
}

#pragma NOAREGS
void restore_fir() {
	SI_SEGMENT_VARIABLE(SFRPAGE_save, unsigned char, xdata);
	SFRPAGE_save = SFRPAGE;
	SFRPAGE = TMR4_PAGE;
	TR4 = 1;
	SFRPAGE = UART0_PAGE;
	AD0EN = 1;
	SFRPAGE = SFRPAGE_save;
}

#pragma NOAREGS
void modbus_init_from_flash(void (*init_after_flash_reload)(void)) {
	FLASH_Read (modbus_buffer_data, MODBUS_FLASH_ADDRESS, 3000, 0);
	init_after_flash_reload_func_pointer = init_after_flash_reload;
	init_after_flash_reload_func_pointer();
}

#pragma NOAREGS
void resetFlashUpdate() {
	modbus_buffer_data [MODBUS_REFRESH_FLASH_MEMORY_ADDRESS_0] = 0;
  modbus_buffer_data [MODBUS_REFRESH_FLASH_MEMORY_ADDRESS_1] = 0; 
	modbus_buffer_data [MODBUS_REFRESH_FLASH_MEMORY_ADDRESS_2] = 0;
  modbus_buffer_data [MODBUS_REFRESH_FLASH_MEMORY_ADDRESS_3] = 0; 
}

#pragma NOAREGS
bool isNeedFlashUpdate() {
	return
	modbus_buffer_data [MODBUS_REFRESH_FLASH_MEMORY_ADDRESS_0] == 0x11 &&
  modbus_buffer_data [MODBUS_REFRESH_FLASH_MEMORY_ADDRESS_1] == 0x11 &&
	modbus_buffer_data [MODBUS_REFRESH_FLASH_MEMORY_ADDRESS_2] == 0x22 &&
  modbus_buffer_data [MODBUS_REFRESH_FLASH_MEMORY_ADDRESS_3] == 0x22;
}

#pragma NOAREGS
uint16_t crc16_update(uint16_t crc, uint8_t a) {
	SI_SEGMENT_VARIABLE(i, int, xdata);
	crc ^= (uint16_t)a;
	for (i = 0; i < 8; ++i) {
		if (crc & 1)
			crc = (crc >> 1) ^ 0xA001;
		else
			crc = (crc >> 1);
	}
	return crc;
}

#pragma NOAREGS
uint16_t calc_crc(uint8_t * command, int size_command) {
	SI_SEGMENT_VARIABLE(crc, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(i, int, xdata);
	crc = 0xFFFF;
	i = 0;
	for(i=0; i<size_command; i++) {
		crc = crc16_update(crc, (uint8_t) command [i]);
	}
	return crc;
}

#pragma NOAREGS
bool modbus_check_crc(uint8_t * command_receiver, int receiver_pointer) {
	if(receiver_pointer > 2) {
		uint16_t crc_calc = calc_crc(command_receiver, receiver_pointer - 2);
		uint16_t crc = command_receiver [receiver_pointer - 1];
		crc = (crc << 8) + command_receiver [receiver_pointer - 2];
		return crc_calc == crc;
	}
	return false;
}

#pragma NOAREGS
uint8_t modbus_get_address() {
	return modbus_buffer_data [MODBUS_ADDRESS_IN_MEMORY];
}

#pragma NOAREGS
bool modbus_check_address() {
	return modbus_get_address() == modbus_command_receiver [MODBUS_ADDRESS]
		|| modbus_command_receiver [MODBUS_ADDRESS] == MODBUS_BROADCAST_ADDRESS;
}

#pragma NOAREGS
uint8_t modbus_get_function() {
	return modbus_command_receiver [MODBUS_FUNCTION];
}

#pragma NOAREGS
void modbus_response_error(uint8_t error) {
	SI_SEGMENT_VARIABLE(crc, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(i, int, xdata);
	crc = 0xFFFF;
	i = 0;
	modbus_error_response [MODBUS_ADDRESS] = modbus_get_address();
	modbus_error_response [MODBUS_ERROR] = MODBUS_ERROR_CODE;
	modbus_error_response [MODBUS_EXCEPTION] = error;
	crc = calc_crc(modbus_error_response, 3);
	modbus_error_response [3] = (uint8_t)(crc >> 8);
	modbus_error_response [4] = (uint8_t)(crc);
	for(i = 0; i<5; i++) {
		modbus_push_transmit_buffer(modbus_error_response [i]);
	}
}

#pragma NOAREGS
int modbus_process_function_3() {
	SI_SEGMENT_VARIABLE(crc, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(i, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(register_hi, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(register_lo, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(modbus_data, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(address_hi, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(address_lo, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(address, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(registers_hi, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(registers_lo, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(registers, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(number, uint8_t, xdata);
	crc = 0xFFFF;

	address_hi = modbus_command_receiver [MODBUS_FIRST_REGISTER_HI];
	address_lo = modbus_command_receiver [MODBUS_FIRST_REGISTER_LO];
	address = (address_hi << 8) + address_lo;
	registers_hi = modbus_command_receiver [MODBUS_NUMBER_OF_REGISTER_TO_READ_HI];
	registers_lo = modbus_command_receiver [MODBUS_NUMBER_OF_REGISTER_TO_READ_LO];
	registers = (registers_hi << 8) + registers_lo;
	number = registers << 1;
	if((address << 1) + number >= MODBUS_DATA_LENGTH) {
		modbus_response_error(MODBUS_ERROR_ILLEGAL_DATA_ADDRESS);
	  return MODBUS_FAIL;
	}
	else {
		modbus_data = modbus_get_address();
		crc = crc16_update(crc, modbus_data);
		modbus_push_transmit_buffer(modbus_data);
		
		modbus_data = 3;
		crc = crc16_update(crc, modbus_data);
		modbus_push_transmit_buffer(modbus_data);
		
		modbus_data = number;
		crc = crc16_update(crc, modbus_data);
		modbus_push_transmit_buffer(modbus_data);
		
		for(i=0; i<registers; i++) {
			register_lo = modbus_buffer_data [(address << 1) + (i << 1)];
			register_hi = modbus_buffer_data [(address << 1) + (i << 1) + 1];
			modbus_data = register_lo;
			crc = crc16_update(crc, modbus_data);
			modbus_push_transmit_buffer(modbus_data);
			modbus_data = register_hi;
			crc = crc16_update(crc, modbus_data);
			modbus_push_transmit_buffer(modbus_data);
		}
		modbus_push_transmit_buffer((uint8_t)(crc));
		modbus_push_transmit_buffer((uint8_t)(crc >> 8));
		EA = 0;
		init_after_flash_reload_func_pointer();
		EA = 1;
		TI0 = 1;
		return MODBUS_GOOD;
	}
}

#pragma NOAREGS
bool modbus_check_size_of_func16(int registers_num) {
	return modbus_receiver_pointer == (MODBUS_FUNCTION_16_BASE_LENGTH + (registers_num << 1));
}

#pragma NOAREGS
int modbus_process_function_16() {
	SI_SEGMENT_VARIABLE(crc, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(i, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(p, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(register_hi, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(register_lo, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(register_in_hi, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(register_in_lo, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(modbus_data, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(address_hi, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(address_lo, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(address, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(registers_hi, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(registers_lo, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(registers, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(number, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(SFRPAGE_save, unsigned char, xdata);
	
	SFRPAGE_save = SFRPAGE;
	crc = 0xFFFF;
	
	address_hi = modbus_command_receiver [MODBUS_FIRST_REGISTER_HI];
	address_lo = modbus_command_receiver [MODBUS_FIRST_REGISTER_LO];
	address = (address_hi << 8) + address_lo;
	registers_hi = modbus_command_receiver [MODBUS_NUMBER_OF_REGISTER_TO_WRITE_HI];
	registers_lo = modbus_command_receiver [MODBUS_NUMBER_OF_REGISTER_TO_WRITE_LO];
	registers = (registers_hi << 8) + registers_lo;
	number = registers << 1;
	if(((address << 1) + number >= MODBUS_DATA_LENGTH) || !modbus_check_size_of_func16(registers)) {
		modbus_response_error(MODBUS_ERROR_ILLEGAL_DATA_ADDRESS);
		return MODBUS_FAIL;
	}
	else {
		resetFlashUpdate();
		for(i=0; i<registers; i++) {
			register_in_hi = modbus_command_receiver [(i << 1) + MODBUS_WRITE_REGISTER_OFFSET];
			register_in_lo = modbus_command_receiver [(i << 1) + 1 + MODBUS_WRITE_REGISTER_OFFSET];
			register_hi = (address << 1) + (i << 1);
			register_lo = (address << 1) + (i << 1) + 1;
			modbus_buffer_data [register_lo] = register_in_lo;
			modbus_buffer_data [register_hi] = register_in_hi;
		}
		modbus_data = modbus_get_address();
		crc = crc16_update(crc, modbus_data);
		modbus_push_transmit_buffer(modbus_data);
		
		modbus_data = 16;
		crc = crc16_update(crc, modbus_data);
		modbus_push_transmit_buffer(modbus_data);
		
		modbus_data = address_hi;
		crc = crc16_update(crc, modbus_data);
		modbus_push_transmit_buffer(modbus_data);
		
		modbus_data = address_lo;
		crc = crc16_update(crc, modbus_data);
		modbus_push_transmit_buffer(modbus_data);
		
		modbus_data = registers_hi;
		crc = crc16_update(crc, modbus_data);
		modbus_push_transmit_buffer(modbus_data);
		
		modbus_data = registers_lo;
		crc = crc16_update(crc, modbus_data);
		modbus_push_transmit_buffer(modbus_data);
		
		modbus_push_transmit_buffer((uint8_t)(crc));
		modbus_push_transmit_buffer((uint8_t)(crc >> 8));
		
		if (isNeedFlashUpdate()) {
			for (p=0; p<4; p++) {
				// 4 - pages (one page size is 1024)
				FLASH_Update(MODBUS_FLASH_ADDRESS + p * 1024, modbus_buffer_data + p * 1024, 1024, 0);
			}
			init_after_flash_reload_func_pointer();
		}
		TI0 = 1;
		return MODBUS_GOOD;
		//AD0EN = 1;
	}
}

#pragma NOAREGS
void modbus_command_received() {
	SI_SEGMENT_VARIABLE(modbus_result, char, xdata);
	SI_SEGMENT_VARIABLE(SFRPAGE_save, unsigned char, xdata);
	
	modbus_result = MODBUS_FAIL;
	SFRPAGE_save = SFRPAGE;
	
	if(modbus_check_crc(modbus_command_receiver, modbus_receiver_pointer) && modbus_check_address()) {
		uint8_t function = modbus_get_function();
		if(function == 3 || function == 16) {
			if(function ==3) {
				modbus_result = modbus_process_function_3();
			}
			else {
				modbus_result = modbus_process_function_16();
			}
			if (modbus_receiver_pointer < 0xFF) {
				restore_fir();
			}
		}
		else {
			modbus_response_error(MODBUS_ERROR_ILLEGAL_FUNCTION_CODE);
		}
		//modbus_receiver_pointer = 0;
	}
	sender_pause_timer = 0;
	modbus_receiver_pointer = 0;
	if (modbus_result == MODBUS_FAIL) {
		restore_fir();
	} else {
	  //init_after_flash_reload_func_pointer();
	}
}

#pragma NOAREGS
int get_modbus_receiver_counter() {
	return sender_pause_timer;
}

#pragma NOAREGS
void set_modbus_receiver_counter(int mrc) {
	sender_pause_timer = mrc;
}

#pragma NOAREGS
void inc_modbus_receiver_counter() {
	sender_pause_timer++;
}

#pragma NOAREGS
void modbus_byte_receive(uint8_t input) {
	sender_pause_timer = 0;
	modbus_command_receiver [modbus_receiver_pointer++] = input;
	if(modbus_receiver_pointer == MODBUS_RECEIVER_LENGTH) {
		modbus_receiver_pointer = 0;
	}
}

#pragma NOAREGS
void modbus_transmit_byte() {
	if(modbus_transmitter_pointer_right > 0) {
		SBUF0 = modbus_command_transmitter [modbus_transmitter_pointer_left++];
		if (modbus_transmitter_pointer_left == MODBUS_TRANSMITTER_LENGTH) {
			modbus_transmitter_pointer_left = 0;
		}
	}
	if(modbus_transmitter_pointer_right == modbus_transmitter_pointer_left) {
		modbus_transmitter_pointer_right = 0;
		modbus_transmitter_pointer_left = 0;
	}
}

#pragma NOAREGS
void modbus_push_transmit_buffer(uint8_t output) {
	 /*
	 if(output == '\n') {
	   modbus_command_transmitter[modbus_transmitter_pointer_right++] = 0x0d;
	 }
	 else {
	   modbus_command_transmitter[modbus_transmitter_pointer_right++] = output;
	 }
	 */
	 modbus_command_transmitter[modbus_transmitter_pointer_right++] = output;
	 if(modbus_transmitter_pointer_right == MODBUS_TRANSMITTER_LENGTH) {
				modbus_transmitter_pointer_right = 0;
	 }
}

#pragma NOAREGS
bool modbus_was_sendind_received() {
	sender_pause_timer++;
	return (sender_pause_timer > 6 && modbus_receiver_pointer > 0);
}

#pragma NOAREGS
bool modbus_transmit_buffer_is_empty() {
	return modbus_transmitter_pointer_right == 0;
}

#pragma NOAREGS
unsigned char modbus_get_freq_divider() {
	return modbus_buffer_data [MODBUS_FREQ_DIVIDER_ADDRESS];
}

#pragma NOAREGS
void modbus_init_freqs(unsigned long * freqs) {
	 SI_SEGMENT_VARIABLE(i, char, xdata);
	 SI_SEGMENT_VARIABLE(hi, uint8_t, xdata);
	 SI_SEGMENT_VARIABLE(lo, uint8_t, xdata);
	 SI_SEGMENT_VARIABLE(address, unsigned short int, xdata);
	 address = MODBUS_FREQ_VALUES_START_ADDRESS;
	 for (i=0; i<12; i++) {
	    hi = modbus_buffer_data [address];
		  lo = modbus_buffer_data [address + 1];
		  freqs [i] = (hi << 8) + lo;
		  address += 2;
	 }
}
