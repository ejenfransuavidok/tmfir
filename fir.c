#include "fir.h"

int getFreqFromModbusForDAC(int number) {
	uint8_t hi;
	uint8_t lo;
	uint8_t * modbus_buffer_data;
	
	modbus_buffer_data = getModbusBufferData();
	
	hi = modbus_buffer_data[((MODBUS_OUTPUT_FREQ_FLAG_REGISTER_OFFSET + number) << 1)];
	lo = modbus_buffer_data[(((MODBUS_OUTPUT_FREQ_FLAG_REGISTER_OFFSET + number) << 1) + 1)];
	
	return ((hi << 8) + lo);
}

void populateFirAll(SI_UU16_t * coefficients) {
	SI_SEGMENT_VARIABLE(i, int, xdata);
	SI_SEGMENT_VARIABLE(order, int, xdata);
	SI_SEGMENT_VARIABLE(temp, int, xdata);
	SI_SEGMENT_VARIABLE(coefficientIndex, int, xdata);
	SI_SEGMENT_VARIABLE(hi, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(lo, uint8_t, xdata);
	
	uint8_t * modbus_buffer_data;
	coefficientIndex = MODBUS_FILTER_COEFFICIENT_START_REGISTER;
	modbus_buffer_data = getModbusBufferData();
	
	for (i=0; i<1200; i++) {
		temp = coefficientIndex + i;
		hi = modbus_buffer_data [temp << 1];
		lo = modbus_buffer_data [(temp << 1) + 1];
		coefficients [i].u16 = (hi << 8) + lo;
	}
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
	coefficientIndex = MODBUS_FILTER_COEFFICIENT_START_REGISTER + FILTER_MAX_ORDER * number;
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
	for (i = 0; i<order, i<FILTER_MAX_ORDER; i++) {
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
	
	if (number > 11 || number < 0) {
		return;
	}
	address = MODBUS_FREQUENCY_VALUE_START + number;
	address = address << 1;
	hi = (value >> 8);
	lo = (value & 0xFF);
	modbus_buffer_data = getModbusBufferData();
	modbus_buffer_data [address] = hi;
	modbus_buffer_data [address + 1] = lo;
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
int RMS_Calc (int *input_samples, int num_samples, int TAPS)
{
   SI_SEGMENT_VARIABLE(count, int, xdata);
   SI_SEGMENT_VARIABLE(average, float, xdata);
   float RMS_summation = 0;
   float RMS_Value;
   float temp;


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

#if defined __C51__
   RMS_Value = sqrt(RMS_summation);
#elif defined SDCC
   RMS_Value = sqrtf(RMS_summation);
#endif

   return (int)RMS_Value;
}
