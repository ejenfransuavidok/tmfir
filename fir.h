#ifndef FIR_H
#define FIR_H

#include <stdio.h>
#include <math.h>
#include "si_toolchain.h"
#include "C8051F120_defs.h"
#include "modbus.h"

#define MODBUS_FILTER_ORDER_START_REGISTER 36
#define FILTER_MAX_ORDER 61
#define MODBUS_FILTER_COEFFICIENT_START_REGISTER 48
#define MODBUS_FREQUENCY_VALUE_START 12
#define MODBUS_OUTPUT_FREQ_FLAG_REGISTER_OFFSET 24

/**
 * @coefficients - FIR-coeffincients for populate
 * @number - sign of frequency [0-12]
 * @return - filter order
 */
int populateFirCoefficients(SI_UU16_t * coefficients, int number);

void populateFirAll(SI_UU16_t * coefficients);

void putRms2Modbus(int value, int number);

int getFreqFromModbusForDAC(int number);

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