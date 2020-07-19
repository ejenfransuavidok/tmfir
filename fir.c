#include "fir.h"

SI_SEGMENT_VARIABLE(FOUND_1_OR_2_FREQ_FLAG, uint8_t, xdata);

/**
 *
 * MUST WRITE COMMAND TO MODBUS FROM 1278 TO 1283 FOR DP
 *
 */
#pragma NOAREGS
void flashDiodesOnCommand(uint8_t d, uint8_t kp_or_dp) {
	 SI_SEGMENT_VARIABLE(SFRPAGE_save, uint8_t, xdata);
	 SFRPAGE_save = SFRPAGE;
	 SFRPAGE = CONFIG_PAGE;
	 //-----------------------------------------------------------------------
	 // CLEAR - INVERSE LOGIC
	 P7 =  0xFF;
	 if ((d & CMD_1) == (uint8_t)CMD_1) {
	    bit_clear_P7(0);
		  if (kp_or_dp == DP_CONDITION) {
				 modbus_write_register(CMD_ADDRESS_1, 1);
			}
	 }
	 if ((d & CMD_2) == CMD_2) {
		  bit_clear_P7(1);
		  if (kp_or_dp == DP_CONDITION) {
				 modbus_write_register(CMD_ADDRESS_2, 1);
			}
	 }
	 if ((d & CMD_3) == CMD_3) {
			bit_clear_P7(2);
		  if (kp_or_dp == DP_CONDITION) {
				 modbus_write_register(CMD_ADDRESS_3, 1);
			}
	 }
	 if ((d & CMD_4) == CMD_4) {
			bit_clear_P7(3);
		  if (kp_or_dp == DP_CONDITION) {
				 modbus_write_register(CMD_ADDRESS_4, 1);
			}
	 }
	 if ((d & CMD_5) == CMD_5) {
			bit_clear_P7(4);
		  if (kp_or_dp == DP_CONDITION) {
				 modbus_write_register(CMD_ADDRESS_5, 1);
			}
	 }
	 if ((d & CMD_6) == CMD_6) {
	    bit_clear_P7(5);
		  if (kp_or_dp == DP_CONDITION) {
				 modbus_write_register(CMD_ADDRESS_6, 1);
			}
	 }
	 SFRPAGE = SFRPAGE_save;
}

#pragma NOAREGS
int getFreqFromModbusForDAC(int number) {
	uint8_t hi;
	uint8_t lo;
	uint8_t * modbus_buffer_data;
	
	modbus_buffer_data = getModbusBufferData();
	
	hi = modbus_buffer_data[((MODBUS_OUTPUT_FREQ_FLAG_REGISTER_OFFSET + number) << 1)];
	lo = modbus_buffer_data[(((MODBUS_OUTPUT_FREQ_FLAG_REGISTER_OFFSET + number) << 1) + 1)];
	
	return ((hi << 8) + lo);
}

#pragma NOAREGS
uint8_t populateFirCoefficients(SI_UU16_t * coefficients, int number) {
	SI_SEGMENT_VARIABLE(i, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(order, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(result, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(temp, uint16_t, xdata);
	SI_SEGMENT_VARIABLE(coefficientIndex, uint16_t, xdata);
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
	result = modbus_buffer_data [order + 1];
	if (result != 61) {
		result = 0;
	}
	for (i = 0; i<result; i++) {
		temp = coefficientIndex + i;
		hi = modbus_buffer_data [temp << 1];
		lo = modbus_buffer_data [(temp << 1) + 1];
		coefficients [i].u16 = (hi << 8) + lo;
	}
	if (result != 61) {
		NOP();
	}
	return result;
}


#pragma NOAREGS
void putRms2Modbus(int value, uint8_t number) {
	uint8_t * modbus_buffer_data;
	SI_SEGMENT_VARIABLE(i, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(hi, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(lo, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(address, unsigned int, xdata);
	SI_SEGMENT_VARIABLE(amplitude_reference, unsigned int, xdata);
	SI_SEGMENT_VARIABLE(flag, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(SFRPAGE_save, unsigned char, xdata);
	SI_SEGMENT_VARIABLE(d, uint8_t, xdata);
	
	SFRPAGE_save = SFRPAGE;
	
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
	//-------------------------------------------------------------------------------------
	// DP
	d = 0;
	for (i=0; i<8; i++) {
		address = MODBUS_FREQUENCY_VALUE_START + i;
		address = address << 1;
	  address += 1;
		if (modbus_buffer_data [address] == 1) {
		   d = bit_set(d, i);
		}
		if ((d & 0x01 == 0x01) || (d & 0x02 == 0x02)) {
		   FOUND_1_OR_2_FREQ_FLAG = TRUE;
		} else {
		   FOUND_1_OR_2_FREQ_FLAG = FALSE;
		}
	}
	flashDiodesOnCommand(d, DP_CONDITION);
	//-------------------------------------------------------------------------------------
}
//-----------------------------------------------------------------------------
// bits operations
//-----------------------------------------------------------------------------
#pragma NOAREGS
void bit_set_P5(uint8_t position)
{
	 SI_SEGMENT_VARIABLE(SFRPAGE_save, uint8_t, xdata);
	 SI_SEGMENT_VARIABLE(d, uint8_t, xdata);
	 SFRPAGE_save = SFRPAGE;
	 SFRPAGE = CONFIG_PAGE;
   d = (0x01<<position);
	 P5|= d;
	 SFRPAGE = SFRPAGE_save;
}
#pragma NOAREGS
void bit_clear_P5(uint8_t position)
{
	 SI_SEGMENT_VARIABLE(SFRPAGE_save, uint8_t, xdata);
	 SI_SEGMENT_VARIABLE(d, uint8_t, xdata);
	 SFRPAGE_save = SFRPAGE;
	 SFRPAGE = CONFIG_PAGE;
   d = (1u<<position);
	 P5&= ~d;
	 SFRPAGE = SFRPAGE_save;
}
#pragma NOAREGS
void bit_set_P6(uint8_t position)
{
	 SI_SEGMENT_VARIABLE(SFRPAGE_save, uint8_t, xdata);
	 SI_SEGMENT_VARIABLE(d, uint8_t, xdata);
	 SFRPAGE_save = SFRPAGE;
	 SFRPAGE = CONFIG_PAGE;
   d = (1u<<position);
	 P6|= d;
	 SFRPAGE = SFRPAGE_save;
}
#pragma NOAREGS
void bit_clear_P6(uint8_t position)
{
	 SI_SEGMENT_VARIABLE(SFRPAGE_save, uint8_t, xdata);
	 SI_SEGMENT_VARIABLE(d, uint8_t, xdata);
	 SFRPAGE_save = SFRPAGE;
	 SFRPAGE = CONFIG_PAGE;
   d = (1u<<position);
	 P6&= ~d;
	 SFRPAGE = SFRPAGE_save;
}
#pragma NOAREGS
void bit_set_P7(uint8_t position)
{
	 SI_SEGMENT_VARIABLE(SFRPAGE_save, uint8_t, xdata);
	 SI_SEGMENT_VARIABLE(d, uint8_t, xdata);
	 SFRPAGE_save = SFRPAGE;
	 SFRPAGE = CONFIG_PAGE;
   d = (1u<<position);
	 P7|= d;
	 SFRPAGE = SFRPAGE_save;
}
#pragma NOAREGS
void bit_clear_P7(uint8_t position)
{
	 SI_SEGMENT_VARIABLE(SFRPAGE_save, uint8_t, xdata);
	 SI_SEGMENT_VARIABLE(d, uint8_t, xdata);
	 SFRPAGE_save = SFRPAGE;
	 SFRPAGE = CONFIG_PAGE;
   d = (1u<<position);
	 P7&=~d;
	 SFRPAGE = SFRPAGE_save;
}
#pragma NOAREGS
uint8_t bit_set(uint8_t d, uint8_t position)
{
   d |= (1u<<position);
	 return d;
}
#pragma NOAREGS
uint8_t bit_clear(uint8_t d, uint8_t position)
{
	 d &= ~(1u<<position);
	 return d;
}
//-----------------------------------------------------------------------------
// KP - 1 / DP - 0
#pragma NOAREGS
int getCondition() {
	SI_SEGMENT_VARIABLE(SFRPAGE_save, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(result, int, xdata);
	SFRPAGE_save = SFRPAGE;
	SFRPAGE = CONFIG_PAGE;
	result = CONDSELECTOR;
	SFRPAGE = SFRPAGE_save;
	return result;
}
//-----------------------------------------------------------------------------
// FLASH DIODES P5 and P6
#pragma NOAREGS
void flashP5P6(uint8_t number, uint8_t flag) {
	if (number < 8) {
		 flag == 1 ? bit_set_P5(number) : bit_clear_P5(number);
  } else {
		 flag == 1 ? bit_set_P6(number - 4) : bit_clear_P6(number - 4);
  }
}
//-----------------------------------------------------------------------------
// get DC24 input P4^3 value
#pragma NOAREGS
uint8_t getDC24INPUT() {
	SI_SEGMENT_VARIABLE(SFRPAGE_save, uint8_t, xdata);
	SI_SEGMENT_VARIABLE(value, uint8_t, xdata);
	SFRPAGE_save = SFRPAGE;
  SFRPAGE = CONFIG_PAGE;
	value = DC24INPUT;
	SFRPAGE = SFRPAGE_save;
	return value;
}
//-----------------------------------------------------------------------------
// set DC24 output P4^2 value
#pragma NOAREGS
void setDC24OUTPUT(uint8_t value) {
  SI_SEGMENT_VARIABLE(SFRPAGE_save, uint8_t, xdata);
	SFRPAGE_save = SFRPAGE;
  SFRPAGE = CONFIG_PAGE;
	DC24OUTPUT = value;
	SFRPAGE = SFRPAGE_save;
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
#pragma NOAREGS
int RMS_Calc (int *input_samples, int num_samples, int TAPS)
{
   int count = 0;
   float average = 0;
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
