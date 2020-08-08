#ifndef FIR_H
#define FIR_H

#include <stdio.h>
#include <math.h>
#include "si_toolchain.h"
#include "C8051F120_defs.h"
#include "modbus.h"

#define KP_CONDITION 1
#define DP_CONDITION 0
sbit CONDSELECTOR = P3^7;	// 0 - DP; 1 - KP
sbit DC24OUTPUT = P4^2;
sbit DC24INPUT = P4^3;

#define MODBUS_FILTER_ORDER_START_REGISTER 36
#define FILTER_MAX_ORDER 100
#define FILTER_MAX_ORDER_IN_MODBUS 100
#define MODBUS_FILTER_COEFFICIENT_START_REGISTER 48
#define MODBUS_FREQUENCY_VALUE_START 12
#define MODBUS_FREQUENCY_AMPLITUDES_VALUE_START 1262
#define MODBUS_AMPLITUDES_THREASHOLS_VALUE_START 0
#define MODBUS_OUTPUT_FREQ_FLAG_REGISTER_OFFSET 24

//-----------------------------------------------------------------------------
// Commands
//-----------------------------------------------------------------------------
#define CMD_1 0x22
#define CMD_2 0x05
#define CMD_3 0x09
#define CMD_4 0x06
#define CMD_5 0x11
#define CMD_6 0x12

#define CMD_ADDRESS_1 1278
#define CMD_ADDRESS_2 1279
#define CMD_ADDRESS_3 1280
#define CMD_ADDRESS_4 1281
#define CMD_ADDRESS_5 1282
#define CMD_ADDRESS_6 1283

void setDC24OUTPUT(uint8_t value);

uint8_t getDC24INPUT();

/**
 *
 * FLASH P5 && P6 DIODES by number
 *
 */
void flashP5P6(uint8_t number, uint8_t flag);

/**
 *
 * is it DP = 0 or KP = 1
 *
 */
int getCondition();

/**
 *
 * Flash diodes on command
 * MUST WRITE COMMAND TO MODBUS FROM 1278 TO 1283 FOR DP
 *
 */
void flashDiodesOnCommand(uint8_t d, uint8_t kp_or_dp);

/**
 * @coefficients - FIR-coeffincients for populate
 * @number - sign of frequency [0-12]
 * @return - filter order
 */
uint8_t populateFirCoefficients(SI_UU16_t * coefficients, int number);

void putRms2Modbus(int value, uint8_t number);

int getFreqFromModbusForDAC(int number);

void bit_set_P5(uint8_t position);

void bit_clear_P5(uint8_t position);

void bit_set_P6(uint8_t position);

void bit_clear_P6(uint8_t position);

void bit_set_P7(uint8_t position);

void bit_clear_P7(uint8_t position);

uint8_t bit_set(uint8_t d, uint8_t position);

uint8_t bit_clear(uint8_t d, uint8_t position);

/*-----------------------------------------------------------------------------
// RMS_Calc
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//   1) input_samples - pointer to an array of the data to be used for the RMS
//                        Value calculations
//   2) num_samples - the number of data elements in the <input_samples> array
//
// This routine takes a pointer to an array and a number of samples and first
// computes the average value of the data.  Then, it uses this average to
// calculate the RMS Value by using the following equation:
//
//                 N
//                 __
//                 \
//                 /_  (x-x_avg)^2
//                 n=10
// RMS_Value^2 =  -----------------
//                       N-10
//
//
// The above routine skips the first <TAPS> samples where the filter hasn't quite
// settled.
//---------------------------------------------------------------------------*/
int RMS_Calc (int *input_samples, int num_samples, int TAPS);

#endif