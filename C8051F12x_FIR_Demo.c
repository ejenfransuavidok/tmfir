//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include "si_toolchain.h"             // Compiler-specific declarations
                                       // (Keil/SDCC)
#include "C8051F120_defs.h"            // SFR declarations
#include <stdio.h>
#include <math.h>
#include "fir.h"
#include "modbus.h"

//-------------------------- BIPOLIAR SELECTOR --------------------------------
//#define BIPOLIAR_ADC
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

#define BAUDRATE     115200            // Baud rate of UART in bps

#define INTCLK       24500000          // Internal oscillator frequency in Hz    

#define SYSCLK       98000000          // Output of PLL derived from (INTCLK*2)

#define SAMPLE_RATE  11000             // Sample frequency in Hz

#define N            256               // Number of samples to capture at
                                       // each DAC frequency

#define PHASE_PRECISION  65536         // Range of phase accumulator

#define OUTPUT_RATE_DAC  20000L        // DAC output rate in Hz

#define START_FREQUENCY  10            // Define the starting frequency
#define STOP_FREQUENCY   4999          // Define the ending frequency
#define FREQ_STEP        10            // Define the number of Hz the frequency
                                       // will step for the frequency sweep
#define DAC1_VALUE       0x8000        // value for DAC1
#define SECOND_INTERVAL  1024
#define MILLISECONDS_10  10
//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------

#if defined __C51__
#include <intrins.h>
#define NOP() \
   _nop_();
#elif defined SDCC
#define NOP() \
   _asm \
   nop \
   _endasm;
#endif // defined SDCC

// Single FIR_TAP macro takes advantage of mirroring
// (i.e. the FIR coefficients are mirrored, so the coefficient only needs to be
// loaded into the MAC registers once).
#define FIR_TAP_MIRROR(X,Y,Z) \
   MAC0A = X; \
   MAC0BH = Y.u8[MSB]; \
   MAC0BL = Y.u8[LSB]; \
   MAC0BH = Z.u8[MSB]; \
   MAC0BL = Z.u8[LSB];

// Single FIR_TAP macro
#define FIR_TAP(X,Y) \
   MAC0A = X; \
   MAC0BH = Y.u8[MSB]; \
   MAC0BL = Y.u8[LSB];

#define BREAK_MACRO \
   SFRPAGE = UART0_PAGE; \
	 if(TI0 == 1 || RI0 == 1){ \
	   break; \
	 } \
	 SFRPAGE = SFRPAGE_save;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
// For the FIR filter
// 'x' holds the 'delay line' of input samples
//idata SI_UU16_t x[TAPS];
SI_SEGMENT_VARIABLE(x[FILTER_MAX_ORDER], SI_UU16_t, xdata);
SI_SEGMENT_VARIABLE(B_FIR[FILTER_MAX_ORDER], SI_UU16_t, xdata);
SI_SEGMENT_VARIABLE(TAPS, uint8_t, xdata);
SI_SEGMENT_VARIABLE(data_for_filter[N], SI_UU16_t, xdata);
SI_SEGMENT_VARIABLE(data_for_filter_counter, int, xdata);
SI_SEGMENT_VARIABLE(filtered_samples[N], int, xdata);
SI_SEGMENT_VARIABLE(freq_number, unsigned char, xdata);
SI_SEGMENT_VARIABLE(phase_acc[12], SI_UU16_t, xdata) = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
SI_SEGMENT_VARIABLE(FREQS[12], unsigned long, xdata) = {1343, 1445, 1547, 1649, 1751, 1853, 1955, 2057, 2159, 2261, 2363, 2465};
SI_SEGMENT_VARIABLE(frequency, unsigned long, xdata);
SI_SEGMENT_VARIABLE(delay_index_arr[12], unsigned char, xdata) = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
SI_SEGMENT_VARIABLE(freq_divider, unsigned char, xdata);
SI_SEGMENT_VARIABLE(freq_dac_flags[12], unsigned char, xdata);

sbit LED = P1^6;                                         // LED='1' means ON
sbit LED485 = P7^7;                                      // LED for 485
sbit SELECT485 = P4^4;																	 // Select 485 transmit/receive

SI_SEGMENT_VARIABLE(Sample, SI_UU16_t, xdata);           // Filter output
SI_SEGMENT_VARIABLE(Phase_Add[12], unsigned int, xdata); // For the frequency sweep
SI_SEGMENT_VARIABLE(TimerForDC24Output, unsigned int, xdata);
SI_SEGMENT_VARIABLE(DividerForDC24Output, unsigned int, xdata);
SI_SEGMENT_VARIABLE(isNeedGetADCValuesFlag, unsigned int, xdata);
SI_SEGMENT_VARIABLE(TIMER, unsigned short, xdata);
SI_SEGMENT_VARIABLE(modbus_16_post_func_invoke_flag, uint8_t, xdata);
SI_SEGMENT_VARIABLE(FOUND_1_OR_2_FREQ_FLAG, extern uint8_t, xdata);
//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

void SYSCLK_Init (void);               // Configure system clock
void PORT_Init (void);                 // Configure port output
void UART0_Init (void);                // Configure UART operation
void Timer0_Init(void);								 // Configure Timer0
void ADC0_Init (void);                 // Configure ADC
void DAC0_Init(void);									 // Configure DAC0
void DAC1_Init(void);									 // Configure DAC1
void Timer3_Init (int counts);         // Configure Timer 3
void Timer4_Init (int counts);         // Configure Timer 4
void Set_DAC_Frequency (unsigned long frequency);
void init_after_flash_reload();
void delay(unsigned short timer);
void toTransmit485();
void toReceive485();

// Define the UART printing functions
#if defined __C51__
char putchar (char c);                 // Define putchar for Keil
#elif defined SDCC
void putchar (char c);
#endif // defined SDCC

SI_INTERRUPT_PROTO(UART0_ISR, INTERRUPT_UART0);
SI_INTERRUPT_PROTO(TIMER0_ISR, INTERRUPT_TIMER0);
SI_INTERRUPT_PROTO(ADC0_ISR, INTERRUPT_ADC0_EOC);
// A full cycle, 16-bit, 2's complement sine wave lookup table
//int code SINE_TABLE[256] = {
SI_SEGMENT_VARIABLE(SINE_TABLE[256], int, code) = {
   0x0000, 0x0324, 0x0647, 0x096a, 0x0c8b, 0x0fab, 0x12c8, 0x15e2,
   0x18f8, 0x1c0b, 0x1f19, 0x2223, 0x2528, 0x2826, 0x2b1f, 0x2e11,
   0x30fb, 0x33de, 0x36ba, 0x398c, 0x3c56, 0x3f17, 0x41ce, 0x447a,
   0x471c, 0x49b4, 0x4c3f, 0x4ebf, 0x5133, 0x539b, 0x55f5, 0x5842,
   0x5a82, 0x5cb4, 0x5ed7, 0x60ec, 0x62f2, 0x64e8, 0x66cf, 0x68a6,
   0x6a6d, 0x6c24, 0x6dca, 0x6f5f, 0x70e2, 0x7255, 0x73b5, 0x7504,
   0x7641, 0x776c, 0x7884, 0x798a, 0x7a7d, 0x7b5d, 0x7c29, 0x7ce3,
   0x7d8a, 0x7e1d, 0x7e9d, 0x7f09, 0x7f62, 0x7fa7, 0x7fd8, 0x7ff6,
   0x7fff, 0x7ff6, 0x7fd8, 0x7fa7, 0x7f62, 0x7f09, 0x7e9d, 0x7e1d,
   0x7d8a, 0x7ce3, 0x7c29, 0x7b5d, 0x7a7d, 0x798a, 0x7884, 0x776c,
   0x7641, 0x7504, 0x73b5, 0x7255, 0x70e2, 0x6f5f, 0x6dca, 0x6c24,
   0x6a6d, 0x68a6, 0x66cf, 0x64e8, 0x62f2, 0x60ec, 0x5ed7, 0x5cb4,
   0x5a82, 0x5842, 0x55f5, 0x539b, 0x5133, 0x4ebf, 0x4c3f, 0x49b4,
   0x471c, 0x447a, 0x41ce, 0x3f17, 0x3c56, 0x398c, 0x36ba, 0x33de,
   0x30fb, 0x2e11, 0x2b1f, 0x2826, 0x2528, 0x2223, 0x1f19, 0x1c0b,
   0x18f8, 0x15e2, 0x12c8, 0x0fab, 0x0c8b, 0x096a, 0x0647, 0x0324,
   0x0000, 0xfcdc, 0xf9b9, 0xf696, 0xf375, 0xf055, 0xed38, 0xea1e,
   0xe708, 0xe3f5, 0xe0e7, 0xdddd, 0xdad8, 0xd7da, 0xd4e1, 0xd1ef,
   0xcf05, 0xcc22, 0xc946, 0xc674, 0xc3aa, 0xc0e9, 0xbe32, 0xbb86,
   0xb8e4, 0xb64c, 0xb3c1, 0xb141, 0xaecd, 0xac65, 0xaa0b, 0xa7be,
   0xa57e, 0xa34c, 0xa129, 0x9f14, 0x9d0e, 0x9b18, 0x9931, 0x975a,
   0x9593, 0x93dc, 0x9236, 0x90a1, 0x8f1e, 0x8dab, 0x8c4b, 0x8afc,
   0x89bf, 0x8894, 0x877c, 0x8676, 0x8583, 0x84a3, 0x83d7, 0x831d,
   0x8276, 0x81e3, 0x8163, 0x80f7, 0x809e, 0x8059, 0x8028, 0x800a,
   0x8000, 0x800a, 0x8028, 0x8059, 0x809e, 0x80f7, 0x8163, 0x81e3,
   0x8276, 0x831d, 0x83d7, 0x84a3, 0x8583, 0x8676, 0x877c, 0x8894,
   0x89bf, 0x8afc, 0x8c4b, 0x8dab, 0x8f1e, 0x90a1, 0x9236, 0x93dc,
   0x9593, 0x975a, 0x9931, 0x9b18, 0x9d0e, 0x9f14, 0xa129, 0xa34c,
   0xa57e, 0xa7be, 0xaa0b, 0xac65, 0xaecd, 0xb141, 0xb3c1, 0xb64c,
   0xb8e4, 0xbb86, 0xbe32, 0xc0e9, 0xc3aa, 0xc674, 0xc946, 0xcc22,
   0xcf05, 0xd1ef, 0xd4e1, 0xd7da, 0xdad8, 0xdddd, 0xe0e7, 0xe3f5,
   0xe708, 0xea1e, 0xed38, 0xf055, 0xf375, 0xf696, 0xf9b9, 0xfcdc,
};

// Similar to STARTUP.A51 for Keil, this function stub for SDCC allows us to
// disable the WDT before memory is initialized.
#if defined SDCC
void _sdcc_external_startup (void);

void _sdcc_external_startup (void)
{
   WDTCN = 0xDE;                       // Disable watchdog timer
   WDTCN = 0xAD;
}
#endif // defined SDCC

//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------

void main (void)
{
	 //-----------------------------------------------------------------------------
	 // FIR VARIABLES
	 //-----------------------------------------------------------------------------
   static unsigned char delay_index = 0;
	 SI_SEGMENT_VARIABLE(coeff_index, unsigned char, xdata);
	 SI_SEGMENT_VARIABLE(sample_index, unsigned char, xdata);
	 SI_SEGMENT_VARIABLE(opposite_sample_index, unsigned char, xdata);
	 SI_SEGMENT_VARIABLE(i, int, xdata);
	 SI_SEGMENT_VARIABLE(SFRPAGE_SAVE, char, xdata);
	 SI_SEGMENT_VARIABLE(hi, uint8_t, xdata);
	 SI_SEGMENT_VARIABLE(lo, uint8_t, xdata);
	 SI_SEGMENT_VARIABLE(freq_quantity, uint8_t, xdata);
	 unsigned int RMS_Value = 0;
	//------------------------------------------------------------------------------
	 void (*init_func_pointer)(void) = init_after_flash_reload;
	 //-----------------------------------------------------------------------------
   WDTCN = 0xDE;                       // Disable watchdog timer
   WDTCN = 0xAD;

   SYSCLK_Init ();                     // Initialize oscillator
   PORT_Init ();                       // Initialize crossbar and GPIO
   UART0_Init ();                      // Initialize UART0
	 Timer0_Init ();
	
	 // Initialize Timer3 to overflow at the ADC sample rate
   Timer3_Init (SYSCLK/SAMPLE_RATE);
	
	 // Initialize Timer4 to overflow at the DAC sample rate
   Timer4_Init (SYSCLK/OUTPUT_RATE_DAC);	
	
	 DAC0_Init ();                       // Initialize the DAC0
	 DAC1_Init ();                       // Initialize the DAC1
	 ADC0_Init ();                       // Initialize the ADC	
	
   SFRPAGE = ADC0_PAGE;

   AD0EN = 1;                          // Enable ADC

   SFRPAGE = MAC0_PAGE;

   MAC0CF = 0x06;                      // Fractional mode; Saturate
                                       // enabled
	 data_for_filter_counter = 0;
	 
	 //
	 freq_number = 0;
	 
	 frequency = START_FREQUENCY;
	 
	 isNeedGetADCValuesFlag = 0;
	 
	 EA = 1;
	
	 modbus_init_from_flash(init_func_pointer);
	 
	 modbus_16_post_func_invoke_flag = FALSE;
	 
	 FOUND_1_OR_2_FREQ_FLAG = FALSE;
	 
	 //------------------------------------------------------------------------
	 // DP VERSION !!!!!!!!!!!!!!!!! FOR FAST REASON !!!!!!!!!!!!!!!!!!!!!!!!!!
	 freq_quantity = 2;
//-----------------------------------------------------------------------------	 
   while (1) {
		  //-----------------------------------------------------------------------
		  if (modbus_16_post_func_invoke_flag == TRUE) {
				 EA = 0; EA = 0;
				 init_after_flash_reload();
				 EA = 1;
			   modbus_16_post_func_invoke_flag = FALSE;
			}
		  //-----------------------------------------------------------------------
		  if (getDC24INPUT() == 0) {
		    setDC24InputRegister(1);
			}
			//------------------------------------------------------------------------
      if (data_for_filter_counter == N) {
			   for (freq_number = 0; freq_number < freq_quantity; freq_number++) {
            delay_index = delay_index_arr [freq_number];
					  // Initialize the delay line for the FIR filter
					  for (i = 0; i < FILTER_MAX_ORDER; i++)
					  {
						   x[i].s16 = 0;
					  }
					  // Initialize the sample array
					  for (i = 0; i < N; i ++)
					  {
						   filtered_samples[i] = 0;
						   //--------------------------------------------------------------------------------------------------
							 if (isNeedGetADCValuesFlag != 0) {
							   SFRPAGE_SAVE = SFRPAGE;
								 SFRPAGE = UART0_PAGE;
								 if(i == 0) {
								   modbus_push_transmit_buffer(0xAA);
									 modbus_push_transmit_buffer(0xAA);
								 }
								 hi = ((data_for_filter [i].u16 >> 8) & 0x00FF);
								 lo = (data_for_filter [i].u16 & 0x00FF);
								 modbus_push_transmit_buffer(hi);
                 modbus_push_transmit_buffer(lo);								 
								 if ((i + 1) % 128 == 0) {
							     TI0 = 1;
									 delay(100);
							   }
								 if (i == N - 1) {
								   modbus_push_transmit_buffer(0xBB);
									 modbus_push_transmit_buffer(0xBB);
									 isNeedGetADCValuesFlag = 0;
									 TI0 = 1;
									 delay(100);
								 }
								 SFRPAGE = SFRPAGE_SAVE;
							 }
							 //--------------------------------------------------------------------------------------------------
					  }
						//--------------------------------------------------------------------------------------------------
						TAPS = populateFirCoefficients(B_FIR, freq_number);
						if (TAPS != FILTER_MAX_ORDER) {
						   NOP();
						}
						if (TAPS == FILTER_MAX_ORDER) {
							for (i=0; i<N; i++) {					
								 // Store ADC result in the delay line
								 x[delay_index].u16 = data_for_filter[i].u16;
								 // Sample_index points to newest data
								 sample_index = delay_index;         
								 // Update delay index
								 if (delay_index == (TAPS - 1))
								 {
										delay_index = 0;
								 }
								 else
								 {
										delay_index++;
								 }

								 MAC0CF |= 0x08;                  // Clear accumulator
						
								 // Mirror algorithm
								 if (sample_index == TAPS - 1)
								 {
										opposite_sample_index = 0;
								 }
								 else
								 {
										opposite_sample_index = sample_index + 1;
								 }
								 for (coeff_index = 0; coeff_index < (TAPS / 2); coeff_index++)
								 {
										FIR_TAP_MIRROR (B_FIR[coeff_index].u16, x[sample_index],
										x[opposite_sample_index]);
									 
										if (sample_index == 0)
										{
											 sample_index = TAPS - 1;
										}
										else
										{
											 sample_index--;
										}

										if (opposite_sample_index == TAPS - 1)
										{
											 opposite_sample_index = 0;
										}
										else
										{
											 opposite_sample_index++;
										}
								 }
								 if ((TAPS % 2) == 1)             // Handle middle tap of odd order filter
								 {
										FIR_TAP (B_FIR[coeff_index].u16, x[sample_index]);
										NOP ();
										NOP ();
										NOP ();
								 }
								 Sample.u16 = MAC0RND;
								 filtered_samples[i] = Sample.u16;
							}
							RMS_Value = RMS_Calc(filtered_samples, N, TAPS);
							
							putRms2Modbus(RMS_Value, freq_number);
							delay_index_arr [freq_number] = delay_index;
						}
				 }
			   LED = !LED;
				 if (FOUND_1_OR_2_FREQ_FLAG == TRUE && freq_quantity == 2) {
				   freq_quantity = 6;
				 } else {
					 freq_quantity = 2;
				   data_for_filter_counter = 0;
				 }
			}
   }
//-----------------------------------------------------------------------------	 
}

//-----------------------------------------------------------------------------
// Initialization Subroutines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// SYSCLK_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// This routine initializes the system clock to use the internal 24.5MHz*4
// oscillator as its clock source.
//
//-----------------------------------------------------------------------------
void SYSCLK_Init (void)
{
   SI_SEGMENT_VARIABLE(i, char, xdata);
	
   SI_SEGMENT_VARIABLE(SFRPAGE_SAVE, char, xdata);
	
	 SFRPAGE_SAVE = SFRPAGE;             // Save Current SFR page

   SFRPAGE = CONFIG_PAGE;              // Switch to the necessary SFRPAGE

   OSCICN = 0x83;

   // Step 2. Set the PLLSRC bit (PLL0CN.2) to select the desired
   // clock source for the PLL.
   PLL0CN &= ~0x04;                    // Internal oscillator

   // Step 3. Program the Flash read timing bits, FLRT (FLSCL.5-4) to the
   // appropriate value for the new clock rate (see Section 15. Flash Memory
   // on page 199).
   SFRPAGE = LEGACY_PAGE;
   FLSCL |= 0x30;                      // >= 100 MHz
   SFRPAGE = CONFIG_PAGE;

   // Step 4. Enable power to the PLL by setting PLLPWR (PLL0CN.0) to ‘1’.
   PLL0CN |= 0x01;

   // Step 5. Program the PLL0DIV register to produce the divided reference
   // frequency to the PLL.
   PLL0DIV = 0x01;

   // Step 6. Program the PLLLP3-0 bits (PLL0FLT.3-0) to the appropriate
   // range for the divided reference frequency.
   PLL0FLT |= 0x01;

   // Step 7. Program the PLLICO1-0 bits (PLL0FLT.5-4) to the appropriate
   // range for the PLL output frequency.
   PLL0FLT &= ~0x30;

   // Step 8. Program the PLL0MUL register to the desired clock multiplication
   // factor.
   PLL0MUL = 0x04;

   // Step 9. Wait at least 5 µs, to provide a fast frequency lock.
   for (i = 100; i > 0; i--);

   // Step 10. Enable the PLL by setting PLLEN (PLL0CN.1) to ‘1’.
   PLL0CN |= 0x02;

   // Step 11. Poll PLLLCK (PLL0CN.4) until it changes from ‘0’ to ‘1’.
   while ((PLL0CN & 0x10) != 0x10);

   // Step 12. Switch the System Clock source to the PLL using the CLKSEL
   // register.
   CLKSEL = 0x02;

   SFRPAGE = SFRPAGE_SAVE;             // Restore the SFRPAGE
}

//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// Configure the Crossbar and GPIO ports.
//
// Pinout:
//
// P0.0 - UART TX1 (push-pull)
// P0.1 - UART RX1
//
// P1.6 - LED (push-pull)
//
// DAC0 - DAC0 output
//
// AIN0.0 - ADC0 analog input
//
// Note: DAC0 and AIN0.0 must be tied together.
//
//-----------------------------------------------------------------------------
void PORT_Init (void)
{
   SI_SEGMENT_VARIABLE(SFRPAGE_save, unsigned char, xdata);
	 SFRPAGE_save = SFRPAGE;             // Save the current SFRPAGE

   SFRPAGE = CONFIG_PAGE;              // Switch to the necessary SFRPAGE

   XBR0     = 0x04;
   XBR1     = 0x00;
   XBR2     = 0x40;                    // Enable crossbar and weak pull-up
                                       // Enable UART0

   P0MDOUT |= 0x01;                    // Set TX1 pin to push-pull
   P1MDOUT |= 0x40;                    // Set P1.6(LED) to push-pull
	 
	 P3MDOUT &= ~0x80;									 // Set P3.7 to input
	 
	 P4MDOUT |= 0x04;                    // Set P4.2 to push-pull
	 P4MDOUT |= 0x10;                    // Set P4.4 to push-pull
	 P4MDOUT &= ~0x08;                   // Set P4.3 to input
	
	 P5MDOUT |= 0xFF;
	 P6MDOUT |= 0xFF;
	 P7MDOUT |= 0xFF;
	 P5 =  0x00;
	 P6 |= 0x0F;
	 P7 =  0xFF;
	 DC24OUTPUT = 1;
	 DC24INPUT = 1;
	 SELECT485 = 0;											 // to receive
	 CONDSELECTOR = 1;                   // set to KP condition
	 
   SFRPAGE = SFRPAGE_save;             // Restore the SFRPAGE
}

//-----------------------------------------------------------------------------
// UART0_Init
//-----------------------------------------------------------------------------
void Timer0_Init(void)
{
	SI_SEGMENT_VARIABLE(SFRPAGE_save, unsigned char, xdata);
	SFRPAGE_save = SFRPAGE;
	
	SFRPAGE = TIMER01_PAGE;
	
	TMOD   &= 0xFD;
	TMOD	 |= 0x01;
	TH0			= 0x00;
  TL0			= 0x00;
	ET0			= 1;
	TR0			= 1;
	CKCON  |= 0x08;
	SFRPAGE = SFRPAGE_save;
}
//-----------------------------------------------------------------------------
// UART0_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// Configure the UART0 using Timer1, for <BAUDRATE> and 8-N-1.
//
//-----------------------------------------------------------------------------
void UART0_Init (void)
{
   SI_SEGMENT_VARIABLE(SFRPAGE_save, unsigned char, xdata); 
	 SFRPAGE_save = SFRPAGE; // Save the current SFRPAGE

   SFRPAGE = UART0_PAGE;               // Switch to the necessary SFRPAGE
	 	
   SCON0  = 0x70;
	 TMOD   = 0x20;
	 TH1    = 0x5D;///0xE5;///0x5D;////0xE5;// - 115200;
	 TR1    = 1;
	 CKCON |= 0x10;
	 PCON  |= 0x80;//SMOD0 = 1 
	
   TI0    = 0;                         // Indicate TX0 ready
	 
	 PS0    = 1;
	
	 ES0    = 1; 
	
   SFRPAGE = SFRPAGE_save;             // Restore the SFRPAGE
}
//-----------------------------------------------------------------------------
// DAC0_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// Configure DAC1 to update on Timer4 overflows.  VREF is already enabled by
// the ADC initialization code.
//
//-----------------------------------------------------------------------------
void DAC0_Init(void){

   SI_SEGMENT_VARIABLE(SFRPAGE_SAVE, char, xdata);
	 SFRPAGE_SAVE = SFRPAGE;             // Save Current SFR page

   SFRPAGE = DAC0_PAGE;

   DAC0CN = 0x94;                      // Enable DAC0 in left-justified mode
                                       // managed by Timer4 overflows

   SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}

//-----------------------------------------------------------------------------
// DAC1_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// Configure DAC1 to update on write to DAC1H.  VREF is already enabled by
// the ADC initialization code.
//
//-----------------------------------------------------------------------------
void DAC1_Init(void){

   SI_SEGMENT_VARIABLE(SFRPAGE_SAVE, char, xdata);
	 SFRPAGE_SAVE = SFRPAGE;             // Save Current SFR page

   SFRPAGE = DAC1_PAGE;

   DAC1CN = 0x84;                      // Enable DAC1 in left-justified mode
                                       // managed by write data to DAC1H
	 
	 DAC1 = DAC1_VALUE;                  // Write to DAC1

   SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}

//-----------------------------------------------------------------------------
// ADC0_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// Configures ADC0 to make single-ended analog measurements on pin AIN0.0 for
// the FIR filter.  Timer3 overflows are the conversion source and the data is
// left-justified.  This function also enables the ADC end-of-conversion
// interrupt and leaves the ADC disabled.
//
//-----------------------------------------------------------------------------
void ADC0_Init (void)
{
   SI_SEGMENT_VARIABLE(SFRPAGE_SAVE, char, xdata);
	
	 SFRPAGE_SAVE = SFRPAGE;             // Save Current SFR page

   SFRPAGE = ADC0_PAGE;

   ADC0CN = 0x05;                      // ADC0 disabled; normal tracking
                                       // mode; ADC0 conversions are initiated
                                       // on overflow of Timer3; ADC0 data is
                                       // left-justified

   REF0CN = 0x03;                      // Enable on-chip VREF and VREF output
                                       // buffer
#ifndef BIPOLIAR_ADC
   AMX0SL = 0x00;                      // Select AIN0.0 as ADC mux input
#else	
	 AMX0CF = 0x01;
	 AMX0SL = 0x00;
#endif
	
   ADC0CF = (SYSCLK/2500000) << 3;     // ADC conversion clock = 2.5MHz

   EIE2 |= 0x02;                       // Enable ADC interrupts

   SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}

//-----------------------------------------------------------------------------
// Timer3_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// Configure Timer3 to auto-reload at interval specified by <counts> (no
// interrupt generated) using SYSCLK as its time base.
//
// Timer 3 overflow automatically triggers ADC0 conversion.
//
//-----------------------------------------------------------------------------
void Timer3_Init (int counts)
{
   SI_SEGMENT_VARIABLE(SFRPAGE_SAVE, char, xdata);
	
	 SFRPAGE_SAVE = SFRPAGE;             // Save Current SFR page

   SFRPAGE = TMR3_PAGE;

   TMR3CN = 0x00;                      // Stop Timer3; Clear TF3;
   TMR3CF = 0x08;                      // use SYSCLK as timebase

   RCAP3   = -counts;                  // Init reload values
   TMR3    = RCAP3;                    // set to reload immediately
   EIE2   &= ~0x01;                    // Disable Timer3 interrupts
   TR3 = 1;                            // Start Timer3

   SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}
//-----------------------------------------------------------------------------
// Timer4_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:
//   1) counts - the number of timer clocks to count before a timer interrupt
//           should occur
//
// Configure Timer4 to auto-reload mode and to generate interrupts
// at intervals specified in <counts> using SYSCLK as its time base.
//
// Timer 4 overflow controls the DAC update rate.
//
//-----------------------------------------------------------------------------
void Timer4_Init (int counts)
{
   SI_SEGMENT_VARIABLE(SFRPAGE_SAVE, char, xdata);
	
	 SFRPAGE_SAVE = SFRPAGE;          // Save Current SFR page

   SFRPAGE = TMR4_PAGE;

   TMR4CN = 0x00;                   // Stop Timer4; Clear overflow flag (TF4);
                                    // Set to Auto-Reload Mode

   TMR4CF = 0x08;                   // Configure Timer4 to increment;
                                    // Timer4 counts SYSCLKs

   RCAP4 = -counts;                 // Set reload value
   TMR4 = RCAP4;                    // Initialzie Timer4 to reload value
	
   EIE2 |= 0x04;                    // Enable Timer4 interrupts
   TR4 = 1;                         // Start Timer4

   SFRPAGE = SFRPAGE_SAVE;          // Restore SFR page
}
//-----------------------------------------------------------------------------
// ADC0_ISR
//-----------------------------------------------------------------------------
//
// ADC0 end-of-conversion ISR
//
// This interrupt service routine is called on ADC0 conversion complete.
// The ADC result is converted to signed and stored in the FIR delay line.
//
// If the global <FIR_On> bit is set to a '1', then the FIR output is computed
// and stored in the global variable 'Sample'.  The 'Sample_Ready' indicator
// bit is set to indicate the value is ready.
//
// If <FIR_On> is set to '0', then the ADC sample is copied to the global
// 'Sample' variable.  The 'Sample_Ready' indicator bit is set to indicate
// that the value is ready.
//
//-----------------------------------------------------------------------------
//void ADC0_ISR (void) interrupt 15
#pragma NOAREGS
SI_INTERRUPT(ADC0_ISR, INTERRUPT_ADC0_EOC)
{
	 volatile SI_UU16_t input;
	
   AD0INT = 0;                         // Clear ADC conversion complete
                                       // indicator

   input.s16 = ADC0^0x8000;            // Convert to bipolar value
	 
	 
   if (data_for_filter_counter < N) {
			data_for_filter [data_for_filter_counter++].u16 = input.u16;
	 }
}

#pragma NOAREGS
SI_INTERRUPT(TIMER0_ISR, INTERRUPT_TIMER0)
{
	unsigned char SFRPAGE_save = SFRPAGE;
	SFRPAGE = TIMER01_PAGE;
	TH0			= 0x00;
  TL0			= 0x00;
	TF0 		= 0;
	SFRPAGE = SFRPAGE_save;
	if(modbus_was_sendind_received()) {
		modbus_command_received();
		SFRPAGE = CONFIG_PAGE;
		LED485 	= !LED485;
	}
	SFRPAGE = CONFIG_PAGE;
	if (DC24OUTPUT == 0) {
	  if (TimerForDC24Output++ % DividerForDC24Output == 0) {
	    DC24OUTPUT = 1;
		}
  }
	TIMER++;
	SFRPAGE = SFRPAGE_save;
}

#pragma NOAREGS
SI_INTERRUPT(UART0_ISR, INTERRUPT_UART0)
{
	unsigned char SFRPAGE_save = SFRPAGE; // Save the current SFRPAGE
  SFRPAGE = UART0_PAGE;
	if(RI0 == 1) {
		modbus_byte_receive(SBUF0);
		RI0 = 0;
	}
	if(TI0 == 1) {
		TI0 = 0;
		toTransmit485();
		if (modbus_transmit_buffer_is_empty()) {
			toReceive485();
		}
		else {
			modbus_transmit_byte();
		}
	}
	SFRPAGE = SFRPAGE_save;               // Restore the SFRPAGE	
}
//-----------------------------------------------------------------------------
// Timer4_ISR
//-----------------------------------------------------------------------------
//
// This ISR is called on Timer4 overflows.  Timer4 is set to auto-reload mode
// and is used to schedule the DAC output sample rate in this example.
// Note that the value that is written to DAC1 during this ISR call is
// actually transferred to DAC1 at the next Timer4 overflow.
//
//-----------------------------------------------------------------------------
//void Timer4_ISR (void) interrupt 16
#pragma NOAREGS
SI_INTERRUPT(Timer4_ISR, INTERRUPT_TIMER4)
{ 
	 char number = 0;
	 int temp1 = 0;											 // The temporary value that passes
                                       // through 3 stages before being written
                                       // to the IDAC
   TMR3CN &= ~0x80;                    // Clear Timer3 overflow flag
	
   for (number=0; number<12; number++) {
		 if (freq_dac_flags [number] == 1) {
				phase_acc[number].u16 += Phase_Add [number];
				temp1 += (SINE_TABLE[phase_acc[number].u8[MSB]] / 8);
	 		}
	 }
		 
   SFRPAGE = DAC0_PAGE;

   // Add a DC bias to make the rails 0 to 65535
   // Note: the XOR with 0x8000 translates the bipolar quantity into
   // a unipolar quantity.

   DAC0 = 0x8000 ^ temp1;              // Write to DAC0
}

#pragma NOAREGS
void init_after_flash_reload() {
	 //-----------------------------------------------------------------------
	 SI_SEGMENT_VARIABLE(i, uint8_t, xdata);
	 SI_SEGMENT_VARIABLE(d, uint8_t, xdata);
	 SI_SEGMENT_VARIABLE(SFRPAGE_save, uint8_t, xdata);
	 //-----------------------------------------------------------------------
	 SFRPAGE_save = SFRPAGE;
	 d = 0;
	 //----------------------- FREQ DIVIDER INIT -----------------------------
   freq_divider = modbus_get_freq_divider();
	 if (freq_divider == 0) {
      freq_divider = 1;
   }
   //--------------------------- FREQ INIT ---------------------------------
   modbus_init_freqs(FREQS);
	 for (i=0; i<12; i++) {
	    Phase_Add [i] = (unsigned int)((unsigned long)((FREQS [i] *
                PHASE_PRECISION) / OUTPUT_RATE_DAC));
	    if (getFreqFromModbusForDAC(i) != 0) {
				 freq_dac_flags [i] = 1;
				 if (i < 8) {
				    d = bit_set(d, i);
				 }
			} else {
			   freq_dac_flags [i] = 0;
			}
			if (getCondition() == DP_CONDITION) {
			   // DP
			   flashP5P6(i, freq_dac_flags [i]);
	    }
	 }
	 if (getCondition() == KP_CONDITION) {
	    // KP
			flashDiodesOnCommand(d, KP_CONDITION);
	 }
	 d = getDC24DurationTimeIfEnabed();
	 if (d != 0) {
		  setDC24OUTPUT(0);
		  DividerForDC24Output = d * MILLISECONDS_10;
		  TimerForDC24Output = 1;
	 }
	 //--------------------------------------------------------------------------
	 if (isNeedGetADCValues() == 1) {
	   isNeedGetADCValuesFlag = 1;
	 }
	 SFRPAGE = SFRPAGE_save;
}
//-----------------------------------------------------------------------------
// delay
void delay(unsigned short timer) {
	TIMER = 0;
	while(TIMER < timer); 
}
//-----------------------------------------------------------------------------
// transmit 485
void toTransmit485() {
	SI_SEGMENT_VARIABLE(SFRPAGE_save, uint8_t, xdata);
	SFRPAGE_save = SFRPAGE;
	SFRPAGE = CONFIG_PAGE;
	SELECT485 = 1;
	SFRPAGE = SFRPAGE_save;
}
//-----------------------------------------------------------------------------
// receive 485
void toReceive485() {
  SI_SEGMENT_VARIABLE(SFRPAGE_save, uint8_t, xdata);
	SFRPAGE_save = SFRPAGE;
	SFRPAGE = CONFIG_PAGE;
	SELECT485 = 0;
	SFRPAGE = SFRPAGE_save;
}
//-----------------------------------------------------------------------------
// putchar
//-----------------------------------------------------------------------------
//
// Return Value:
//   1) char c - returns the char c that was passed as a parameter
// Parameters:
//   1) char c - the character to be printed
//
// Print the character <c> using UART0 at <BAUDRATE>.
//
//-----------------------------------------------------------------------------
#if defined __C51__
char putchar (char c)
#elif defined SDCC
void putchar (char c)
#endif
{
	 modbus_push_transmit_buffer(c);
#if defined __C51__
   return c;                           // Print the character
#endif
}
//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
