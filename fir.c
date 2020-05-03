#include "fir.h"
/*
#define BIGINT_SIZE 16

typedef struct {
	unsigned char d [16];
} BIGINT;

void bigintSummator(BIGINT *accumulator, BIGINT *argument) {
	SI_SEGMENT_VARIABLE(i, unsigned char, xdata);
	SI_SEGMENT_VARIABLE(p, unsigned char, xdata);
	p = 0;
	for (i=0; i<BIGINT_SIZE; i++) {
		if (((*accumulator).d [i] + (*argument).d [i]) > 0xFF) {
		  p = 1;
		} else {
		  p = 0;
		}
	  (*accumulator).d [i] = (*accumulator).d [i] + (*argument).d [i] + p;
	}
}
*/
int getFreqFromModbusForDAC(int number) {
	uint8_t hi;
	uint8_t lo;
	uint8_t * modbus_buffer_data;
	
	modbus_buffer_data = getModbusBufferData();
	
	hi = modbus_buffer_data[((MODBUS_OUTPUT_FREQ_FLAG_REGISTER_OFFSET + number) << 1)];
	lo = modbus_buffer_data[(((MODBUS_OUTPUT_FREQ_FLAG_REGISTER_OFFSET + number) << 1) + 1)];
	
	return ((hi << 8) + lo);
}

int populateFirCoefficients(SI_UU16_t * coefficients, int number) {
	SI_SEGMENT_VARIABLE(i, int, xdata);
	SI_SEGMENT_VARIABLE(order, int, xdata);
	SI_SEGMENT_VARIABLE(temp, int, xdata);
	SI_SEGMENT_VARIABLE(coefficientIndex, int, xdata);
	SI_SEGMENT_VARIABLE(hi, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(lo, uint8_t, xdata);
	
	uint8_t * modbus_buffer_data;
	if (number > 11 || number < 0) {
		return 0;
	}
	coefficientIndex = MODBUS_FILTER_COEFFICIENT_START_REGISTER + FILTER_MAX_ORDER_IN_MODBUS * number;
	modbus_buffer_data = getModbusBufferData();
	order = MODBUS_FILTER_ORDER_START_REGISTER;
	order = order + number;
	// reg number to byte number
	order = order << 1;
	// get filter order
	order = modbus_buffer_data [order + 1];
	if (order > FILTER_MAX_ORDER) {
	  order = FILTER_MAX_ORDER;
	}
	for (i = 0; i<order && i<FILTER_MAX_ORDER; i++) {
		temp = coefficientIndex + i;
		hi = modbus_buffer_data [temp << 1];
		lo = modbus_buffer_data [(temp << 1) + 1];
		coefficients [i].u16 = (hi << 8) + lo;
	}
	return order;
}

void putRms2Modbus(int value, int number) {
	uint8_t * modbus_buffer_data;
	SI_SEGMENT_VARIABLE(hi, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(lo, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(address, int, xdata);
	SI_SEGMENT_VARIABLE(amplitude_reference, int, xdata);
	SI_SEGMENT_VARIABLE(flag, uint8_t, xdata);
	
	if (number > 11 || number < 0) {
		return;
	}
	//---------------------- PUT FIR RESULT TO MODBUS ----------------------
	address = MODBUS_FREQUENCY_AMPLITUDES_VALUE_START + number;
	address = address << 1;
	hi = (value >> 8);
	lo = (value & 0xFF);
	modbus_buffer_data = getModbusBufferData();
	modbus_buffer_data [address] = hi;
	modbus_buffer_data [address + 1] = lo;
	//---------------------- READ REFERENCE FROM MODBUS ----------------------
	address = MODBUS_AMPLITUDES_THREASHOLS_VALUE_START + number;
	address = address << 1;
	hi = modbus_buffer_data [address];
	lo = modbus_buffer_data [address + 1];
	amplitude_reference = (hi << 8) + lo;
	//---------------------- COMPARE FIR RESULT AND REFEREBCE ----------------------
	if (value > amplitude_reference) {
		 flag = 1;
	} else {
	   flag = 0;
	}
	//---------------------- PUT COMPARATIVE TO MODBUS LIKE A FLAG ----------------------
	address = MODBUS_FREQUENCY_VALUE_START + number;
	address = address << 1;
	modbus_buffer_data [address] = 0;
	modbus_buffer_data [address + 1] = flag;
}

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
/*
int RMS_Calc (int *input_samples, int num_samples, int TAPS)
{
   int count = 0;
   float average = 0;
   float RMS_summation = 0;
   float RMS_Value;
   float temp;
	
	 //EA = 0;
   // Calculate the average value (x_avg) of the <input_samples> array
   average = 0.0;

   for (count = TAPS; count < num_samples; count++)
   {
      average += (float) input_samples[count];
   }
   average = (float)(average / (num_samples-TAPS));

   // Calculate the RMS Value using the average computed above
   // Calculate the sum from 1 to N of (x-x_avg)^2
	 for (count = TAPS; count < num_samples; count++)
   {
      // calculate difference from mean
      temp = input_samples[count] - average;
      // square it
      temp *= temp;
      // and add it to sum
      RMS_summation += temp;
   }
   // Calculate sum from above / N
   RMS_summation = (float)RMS_summation / (num_samples-TAPS);
	 RMS_Value = RMS_summation / 20000;
	 //EA = 1;
	 return (int)RMS_Value;
}
*/