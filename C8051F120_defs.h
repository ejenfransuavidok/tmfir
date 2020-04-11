//-----------------------------------------------------------------------------
// C8051F120_defs.h
//-----------------------------------------------------------------------------
// Copyright 2006 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
//
// Register/bit definitions for the C8051F12x family.
//
//
// FID:
// Target:         C8051F120, 'F121, 'F122, 'F123, 'F124, 'F125, 'F126, 'F127,
//                 'F130, 'F131, 'F132, 'F133
// Tool chain:     Keil, SDCC
// Command Line:   None
//
// Release 1.0
//    -Ported from 'F360 DEFS rev 1.1 by BW
//    -8 DEC 2006
//

//-----------------------------------------------------------------------------
// Header File Preprocessor Directive
//-----------------------------------------------------------------------------

#ifndef C8051F120_DEFS_H
#define C8051F120_DEFS_H

#include "si_toolchain.h"

//-----------------------------------------------------------------------------
// Byte Registers
//-----------------------------------------------------------------------------

SI_SFR(P0, 0x80);                        // Port 0 Latch
SI_SFR(SP, 0x81);                        // Stack Pointer
SI_SFR(DPL, 0x82);                       // Data Pointer Low
SI_SFR(DPH, 0x83);                       // Data Pointer High
SI_SFR(SFRPAGE, 0x84);                   // SFR Page Select
SI_SFR(SFRNEXT, 0x85);                   // SFR stack next page
SI_SFR(SFRLAST, 0x86);                   // SFR stack last page
SI_SFR(PCON, 0x87);                      // Power Control
SI_SFR(TCON, 0x88);                      // Timer/Counter Control
SI_SFR(CPT0CN, 0x88);                    // Comparator1 Control
SI_SFR(CPT1CN, 0x88);                    // Comparator0 Control
SI_SFR(FLSTAT, 0x88);                    // Flash Status
SI_SFR(TMOD, 0x89);                      // Timer/Counter Mode
SI_SFR(CPT0MD, 0x89);                    // Comparator0 Mode
SI_SFR(CPT1MD, 0x89);                    // Comparator1 Mode
SI_SFR(PLL0CN, 0x89);                    // PLL0 Control
SI_SFR(TL0, 0x8A);                       // Timer/Counter 0 Low
SI_SFR(OSCICN, 0x8A);                    // Internal Oscillator Control
SI_SFR(TL1, 0x8B);                       // Timer/Counter 1 Low
SI_SFR(OSCICL, 0x8B);                    // Internal Oscillator Calibration
SI_SFR(TH0, 0x8C);                       // Timer/Counter 0 High
SI_SFR(OSCXCN, 0x8C);                    // External Oscillator Control
SI_SFR(TH1, 0x8D);                       // Timer/Counter 1 High
SI_SFR(PLL0DIV, 0x8D);                   // PLL0 Divider
SI_SFR(CKCON, 0x8E);                     // Clock Control
SI_SFR(PLL0MUL, 0x8E);                   // PLL0 Multiplier
SI_SFR(PSCTL, 0x8F);                     // Program Store R/W Control
SI_SFR(PLL0FLT, 0x8F);                   // PLL0 Filter
SI_SFR(P1, 0x90);                        // Port 1 Latch
SI_SFR(SSTA0, 0x91);                     // UART0 Status
SI_SFR(MAC0BL, 0x91);                    // MAC0 B Low Byte
SI_SFR(MAC0BH, 0x92);                    // MAC0 B High Byte
SI_SFR(MAC0ACC0, 0x93);                  // MAC0 Accumulator Byte 0
SI_SFR(MAC0ACC1, 0x94);                  // MAC0 Accumulator Byte 1
SI_SFR(MAC0ACC2, 0x95);                  // MAC0 Accumulator Byte 2
SI_SFR(MAC0ACC3, 0x96);                  // MAC0 Accumulator Byte 3
SI_SFR(SFRPGCN, 0x96);                   // SFR Page Control
SI_SFR(CLKSEL, 0x97);                    // System clock select
SI_SFR(MAC0OVR, 0x97);                   // MAC0 Accumulator Overflow
SI_SFR(SCON0, 0x98);                     // UART0 Control
SI_SFR(SCON, 0x98);                      // UART0 Control
SI_SFR(SCON1, 0x98);                     // UART1 Control
SI_SFR(SBUF0, 0x99);                     // UART0 Data Buffer
SI_SFR(SBUF, 0x99);                      // UART0 Data Buffer
SI_SFR(SBUF1, 0x99);                     // UART1 Data Buffer
SI_SFR(SPI0CFG, 0x9A);                   // SPI0 Configuration
SI_SFR(CCH0MA, 0x9A);                    // Cache Miss Accumulator
SI_SFR(SPI0DAT, 0x9B);                   // SPI0 Data
SI_SFR(P4MDOUT, 0x9C);                   // Port 4 Output Mode
SI_SFR(SPI0CKR, 0x9D);                   // SPI0 Clock rate control
SI_SFR(P5MDOUT, 0x9D);                   // Port 5 Output Mode
SI_SFR(P6MDOUT, 0x9E);                   // Port 6 Output Mode
SI_SFR(P7MDOUT, 0x9F);                   // Port 7 Output Mode
SI_SFR(P2, 0xA0);                        // Port 2 Latch
SI_SFR(EMI0TC, 0xA1);                    // EMIF Timing control
SI_SFR(CCH0CN, 0xA1);                    // Cache control
SI_SFR(EMI0CN, 0xA2);                    // EMIF control
SI_SFR(CCH0TN, 0xA2);                    // Cache tuning
SI_SFR(EMI0CF, 0xA3);                    // EMIF configuration
SI_SFR(CCH0LC, 0xA3);                    // Cache lock
SI_SFR(P0MDOUT, 0xA4);                   // Port 0 Output Mode
SI_SFR(P1MDOUT, 0xA5);                   // Port 1 Output Mode
SI_SFR(P2MDOUT, 0xA6);                   // Port 2 Output Mode
SI_SFR(P3MDOUT, 0xA7);                   // Port 3 Output Mode
SI_SFR(IE, 0xA8);                        // Interrupt Enable
SI_SFR(SADDR0, 0xA9);                    // UART0 Slave address
SI_SFR(P1MDIN, 0xAD);                    // Port 1 Analog Input Mode
SI_SFR(P3, 0xB0);                        // Port 3 Latch
SI_SFR(PSBANK, 0xB1);                    // Flash bank select
SI_SFR(FLSCL, 0xB7);                     // Flash scale
SI_SFR(FLACL, 0xB7);                     // Flash access limit
SI_SFR(IP, 0xB8);                        // Interrupt Priority
SI_SFR(SADEN0, 0xB9);                    // UART0 Slave address mask
SI_SFR(AMX0CF, 0xBA);                    // AMUX0 Channel configuration
SI_SFR(AMX2CF, 0xBA);                    // AMUX2 Channel configuration
SI_SFR(AMX0SL, 0xBB);                    // AMUX0 Channel select
SI_SFR(AMX2SL, 0xBB);                    // AMUX2 Channel select
SI_SFR(ADC0CF, 0xBC);                    // ADC0 Configuration
SI_SFR(ADC2CF, 0xBC);                    // ADC2 Configuration
SI_SFR(ADC0L, 0xBE);                     // ADC0 Data Low
SI_SFR(ADC2, 0xBE);                      // ADC2 Data
SI_SFR(ADC0H, 0xBF);                     // ADC0 Data High
SI_SFR(SMB0CN, 0xC0);                    // SMBus0 Control
SI_SFR(MAC0STA, 0xC0);                   // MAC0 Status
SI_SFR(SMB0STA, 0xC1);                   // SMBus0 Status
SI_SFR(MAC0AL, 0xC1);                    // MAC0 A Low Byte
SI_SFR(SMB0DAT, 0xC2);                   // SMBus0 Data
SI_SFR(MAC0AH, 0xC2);                    // MAC0 A High Byte
SI_SFR(SMB0ADR, 0xC3);                   // SMBus0 Slave address
SI_SFR(MAC0CF, 0xC3);                    // MAC0 Configuration
SI_SFR(ADC0GTL, 0xC4);                   // ADC0 Greater-Than Compare Low
SI_SFR(ADC2GT, 0xC4);                    // ADC2 Greater-Than Compare
SI_SFR(ADC0GTH, 0xC5);                   // ADC0 Greater-Than Compare High
SI_SFR(ADC0LTL, 0xC6);                   // ADC0 Less-Than Compare Word Low
SI_SFR(ADC2LT, 0xC6);                    // ADC2 Less-Than Compare Word
SI_SFR(ADC0LTH, 0xC7);                   // ADC0 Less-Than Compare Word High
SI_SFR(TMR2CN, 0xC8);                    // Timer/Counter 2 Control
SI_SFR(TMR3CN, 0xC8);                    // Timer/Counter 3 Control
SI_SFR(TMR4CN, 0xC8);                    // Timer/Counter 4 Control
SI_SFR(P4, 0xC8);                        // Port 4 Latch
SI_SFR(TMR2CF, 0xC9);                    // Timer/Counter 2 Configuration
SI_SFR(TMR3CF, 0xC9);                    // Timer/Counter 3 Configuration
SI_SFR(TMR4CF, 0xC9);                    // Timer/Counter 4 Configuration
SI_SFR(RCAP2L, 0xCA);                    // Timer/Counter 2 Reload Low
SI_SFR(RCAP3L, 0xCA);                    // Timer/Counter 3 Reload Low
SI_SFR(RCAP4L, 0xCA);                    // Timer/Counter 4 Reload Low
SI_SFR(RCAP2H, 0xCB);                    // Timer/Counter 2 Reload High
SI_SFR(RCAP3H, 0xCB);                    // Timer/Counter 3 Reload High
SI_SFR(RCAP4H, 0xCB);                    // Timer/Counter 4 Reload High
SI_SFR(TMR2L, 0xCC);                     // Timer/Counter 2 Low
SI_SFR(TMR3L, 0xCC);                     // Timer/Counter 3 Low
SI_SFR(TMR4L, 0xCC);                     // Timer/Counter 4 Low
SI_SFR(TMR2H, 0xCD);                     // Timer/Counter 2 High
SI_SFR(TMR3H, 0xCD);                     // Timer/Counter 3 High
SI_SFR(TMR4H, 0xCD);                     // Timer/Counter 4 High
SI_SFR(MAC0RNDL, 0xCE);                  // MAC0 Rounding Register Low Byte
SI_SFR(SMB0CR, 0xCF);                    // SMBus0 Clock Rate
SI_SFR(MAC0RNDH, 0xCF);                  // MAC0 Rounding Register High Byte
SI_SFR(PSW, 0xD0);                       // Program Status Word
SI_SFR(REF0CN, 0xD1);                    // Voltage Reference Control
SI_SFR(DAC0L, 0xD2);                     // DAC0 Data Low
SI_SFR(DAC1L, 0xD2);                     // DAC1 Data Low
SI_SFR(DAC0H, 0xD3);                     // DAC0 Data High
SI_SFR(DAC1H, 0xD3);                     // DAC1 Data High
SI_SFR(DAC0CN, 0xD4);                    // DAC0 Control
SI_SFR(DAC1CN, 0xD4);                    // DAC2 Control
SI_SFR(PCA0CN, 0xD8);                    // PCA0 Control
SI_SFR(P5, 0xD8);                        // Port 5 Latch
SI_SFR(PCA0MD, 0xD9);                    // PCA0 Mode
SI_SFR(PCA0CPM0, 0xDA);                  // PCA0 Module 0 Mode Register
SI_SFR(PCA0CPM1, 0xDB);                  // PCA0 Module 1 Mode Register
SI_SFR(PCA0CPM2, 0xDC);                  // PCA0 Module 2 Mode Register
SI_SFR(PCA0CPM3, 0xDD);                  // PCA0 Module 3 Mode Register
SI_SFR(PCA0CPM4, 0xDE);                  // PCA0 Module 4 Mode Register
SI_SFR(PCA0CPM5, 0xDF);                  // PCA0 Module 5 Mode Register
SI_SFR(ACC, 0xE0);                       // Accumulator
SI_SFR(PCA0CPL5, 0xE1);                  // PCA0 Module 5 Capture/Compare Low
SI_SFR(XBR0, 0xE1);                      // Port I/O Crossbar Control 0
SI_SFR(PCA0CPH5, 0xE2);                  // PCA0 Module 5 Capture/Compare High
SI_SFR(XBR1, 0xE2);                      // Port I/O Crossbar Control 1
SI_SFR(XBR2, 0xE3);                      // Port I/O Crossbar Control 2
SI_SFR(EIE1, 0xE6);                      // Extended Interrupt Enable 1
SI_SFR(EIE2, 0xE7);                      // Extended Interrupt Enable 2
SI_SFR(ADC0CN, 0xE8);                    // ADC0 Control
SI_SFR(ADC2CN, 0xE8);                    // ADC2 Control
SI_SFR(P6, 0xE8);                        // Port 6 Latch
SI_SFR(PCA0CPL2, 0xE9);                  // PCA0 Capture 2 Low
SI_SFR(PCA0CPH2, 0xEA);                  // PCA0 Capture 2 High
SI_SFR(PCA0CPL3, 0xEB);                  // PCA0 Capture 3 Low
SI_SFR(PCA0CPH3, 0xEC);                  // PCA0 Capture 3 High
SI_SFR(PCA0CPL4, 0xED);                  // PCA0 Capture 4 Low
SI_SFR(PCA0CPH4, 0xEE);                  // PCA0 Capture 4 High
SI_SFR(RSTSRC, 0xEF);                    // Reset Source Configuration/Status
SI_SFR(B, 0xF0);                         // B Register
SI_SFR(EIP1, 0xF6);                      // External Interrupt Priority 1
SI_SFR(EIP2, 0xF7);                      // External Interrupt Priority 2
SI_SFR(SPI0CN, 0xF8);                    // SPI0 Control
SI_SFR(P7, 0xF8);                        // Port 7 Latch
SI_SFR(PCA0L, 0xF9);                     // PCA0 Counter Low
SI_SFR(PCA0H, 0xFA);                     // PCA0 Counter High
SI_SFR(PCA0CPL0, 0xFB);                  // PCA0 Capture 0 Low
SI_SFR(PCA0CPH0, 0xFC);                  // PCA0 Capture 0 High
SI_SFR(PCA0CPL1, 0xFD);                  // PCA0 Capture 1 Low
SI_SFR(PCA0CPH1, 0xFE);                  // PCA0 Capture 1 High
SI_SFR(WDTCN, 0xFF);                     // Watchdog Timer Control

//-----------------------------------------------------------------------------
// 16-bit Register Definitions (might not be supported by all compilers)
//-----------------------------------------------------------------------------

SI_SFR16(DP, 0x82);                      // Data Pointer
SI_SFR16(MAC0B, 0x91);                   // MAC0B data register
SI_SFR16(MAC0ACCL, 0x93);                // MAC0ACC low registers
SI_SFR16(MAC0ACCH, 0x95);                // MAC0ACC high registers
SI_SFR16(ADC0, 0xbe);                    // ADC0 data
SI_SFR16(MAC0A, 0xc1);                   // MAC0A data register
SI_SFR16(ADC0GT, 0xc4);                  // ADC0 greater than window
SI_SFR16(ADC0LT, 0xc6);                  // ADC0 less than window
SI_SFR16(RCAP2, 0xca);                   // Timer2 capture/reload
SI_SFR16(RCAP3, 0xca);                   // Timer3 capture/reload
SI_SFR16(RCAP4, 0xca);                   // Timer4 capture/reload
SI_SFR16(TMR2, 0xcc);                    // Timer2
SI_SFR16(TMR3, 0xcc);                    // Timer3
SI_SFR16(TMR4, 0xcc);                    // Timer4
SI_SFR16(MAC0RND, 0xce);                 // MAC0RND registers
SI_SFR16(DAC0, 0xd2);                    // DAC0 data
SI_SFR16(DAC1, 0xd2);                    // DAC1 data
SI_SFR16(PCA0CP5, 0xe1);                 // PCA0 Module 5 capture
SI_SFR16(PCA0CP2, 0xe9);                 // PCA0 Module 2 capture
SI_SFR16(PCA0CP3, 0xeb);                 // PCA0 Module 3 capture
SI_SFR16(PCA0CP4, 0xed);                 // PCA0 Module 4 capture
SI_SFR16(PCA0, 0xf9);                    // PCA0 counter
SI_SFR16(PCA0CP0, 0xfb);                 // PCA0 Module 0 capture
SI_SFR16(PCA0CP1, 0xfd);                 // PCA0 Module 1 capture

//-----------------------------------------------------------------------------
// Address Definitions for Bit-addressable Registers
//-----------------------------------------------------------------------------

#define SFR_P0       0x80
#define SFR_TCON     0x88
#define SFR_CPT0CN   0x88
#define SFR_CPT1CN   0x88
#define SFR_FLSTAT   0x88
#define SFR_P1       0x90
#define SFR_SCON0    0x98
#define SFR_SCON     0x98
#define SFR_SCON1    0x98
#define SFR_P2       0xA0
#define SFR_IE       0xA8
#define SFR_P3       0xB0
#define SFR_IP       0xB8
#define SFR_SMB0CN   0xC0
#define SFR_TMR2CN   0xC8
#define SFR_TMR3CN   0xC8
#define SFR_TMR4CN   0xC8
#define SFR_P4       0xC8
#define SFR_PSW      0xD0
#define SFR_PCA0CN   0xD8
#define SFR_P5       0xD8
#define SFR_ACC      0xE0
#define SFR_ADC0CN   0xE8
#define SFR_ADC2CN   0xE8
#define SFR_B        0xF0
#define SFR_SPI0CN   0xF8
#define SFR_P7       0xF8

//-----------------------------------------------------------------------------
// Bit Definitions
//-----------------------------------------------------------------------------

// TCON 0x88
SI_SBIT(TF1, SFR_TCON, 7);               // Timer 1 Overflow Flag
SI_SBIT(TR1, SFR_TCON, 6);               // Timer 1 On/Off Control
SI_SBIT(TF0, SFR_TCON, 5);               // Timer 0 Overflow Flag
SI_SBIT(TR0, SFR_TCON, 4);               // Timer 0 On/Off Control
SI_SBIT(IE1, SFR_TCON, 3);               // Ext. Interrupt 1 Edge Flag
SI_SBIT(IT1, SFR_TCON, 2);               // Ext. Interrupt 1 Type
SI_SBIT(IE0, SFR_TCON, 1);               // Ext. Interrupt 0 Edge Flag
SI_SBIT(IT0, SFR_TCON, 0);               // Ext. Interrupt 0 Type

// CPT0CN  0x88

SI_SBIT(CP0EN, SFR_CPT0CN, 7);           // COMPARATOR 0 ENABLE
SI_SBIT(CP0OUT, SFR_CPT0CN, 6);          // COMPARATOR 0 OUTPUT
SI_SBIT(CP0RIF, SFR_CPT0CN, 5);          // COMPARATOR 0 RISING EDGE INTERRUPT
SI_SBIT(CP0FIF, SFR_CPT0CN, 4);          // COMPARATOR 0 FALLING EDGE INTERRUPT
SI_SBIT(CP0HYP1, SFR_CPT0CN, 3);         // COMPARATOR 0 POSITIVE HYSTERESIS 1
SI_SBIT(CP0HYP0, SFR_CPT0CN, 2);         // COMPARATOR 0 POSITIVE HYSTERESIS 0
SI_SBIT(CP0HYN1, SFR_CPT0CN, 1);         // COMPARATOR 0 NEGATIVE HYSTERESIS 1
SI_SBIT(CP0HYN0, SFR_CPT0CN, 0);         // COMPARATOR 0 NEGATIVE HYSTERESIS 0

// CPT1CN  0x88

SI_SBIT(CP1EN, SFR_CPT1CN, 7);           // COMPARATOR 1 ENABLE
SI_SBIT(CP1OUT, SFR_CPT1CN, 6);          // COMPARATOR 1 OUTPUT
SI_SBIT(CP1RIF, SFR_CPT1CN, 5);          // COMPARATOR 1 RISING EDGE INTERRUPT
SI_SBIT(CP1FIF, SFR_CPT1CN, 4);          // COMPARATOR 1 FALLING EDGE INTERRUPT
SI_SBIT(CP1HYP1, SFR_CPT1CN, 3);         // COMPARATOR 1 POSITIVE HYSTERESIS 1
SI_SBIT(CP1HYP0, SFR_CPT1CN, 2);         // COMPARATOR 1 POSITIVE HYSTERESIS 0
SI_SBIT(CP1HYN1, SFR_CPT1CN, 1);         // COMPARATOR 1 NEGATIVE HYSTERESIS 1
SI_SBIT(CP1HYN0, SFR_CPT1CN, 0);         // COMPARATOR 1 NEGATIVE HYSTERESIS 0

// FLSTAT  0x88
SI_SBIT(FLBUSY, SFR_FLSTAT, 0);          // FLASH Busy

// SCON0 0x98
SI_SBIT(SM00, SFR_SCON0, 7);             // UART0 Mode 0
SI_SBIT(SM10, SFR_SCON0, 6);             // UART0 Mode 1
SI_SBIT(SM20, SFR_SCON0, 5);             // UART0 Multiprocessor enable
SI_SBIT(REN0, SFR_SCON0, 4);             // UART0 RX Enable
SI_SBIT(REN, SFR_SCON0, 4);              // UART0 RX Enable
SI_SBIT(TB80, SFR_SCON0, 3);             // UART0 TX Bit 8
SI_SBIT(TB8, SFR_SCON0, 3);              // UART0 TX Bit 8
SI_SBIT(RB80, SFR_SCON0, 2);             // UART0 RX Bit 8
SI_SBIT(RB8, SFR_SCON0, 2);              // UART0 RX Bit 8
SI_SBIT(TI0, SFR_SCON0, 1);              // UART0 TX Interrupt Flag
SI_SBIT(TI, SFR_SCON0, 1);               // UART0 TX Interrupt Flag
SI_SBIT(RI0, SFR_SCON0, 0);              // UART0 RX Interrupt Flag
SI_SBIT(RI, SFR_SCON0, 0);               // UART0 RX Interrupt Flag

// SCON1 0x98
SI_SBIT(S1MODE, SFR_SCON1, 7);           // UART1 Mode
                                       // Bit6 UNUSED
SI_SBIT(MCE1, SFR_SCON1, 5);             // UART1 MCE
SI_SBIT(REN1, SFR_SCON1, 4);             // UART1 RX Enable
SI_SBIT(TB81, SFR_SCON1, 3);             // UART1 TX Bit 8
SI_SBIT(RB81, SFR_SCON1, 2);             // UART1 RX Bit 8
SI_SBIT(TI1, SFR_SCON1, 1);              // UART1 TX Interrupt Flag
SI_SBIT(RI1, SFR_SCON1, 0);              // UART1 RX Interrupt Flag


// IE 0xA8
SI_SBIT(EA, SFR_IE, 7);                  // Global Interrupt Enable
                                       // Bit 6 unused
SI_SBIT(ET2, SFR_IE, 5);                 // Timer 2 Interrupt Enable
SI_SBIT(ES0, SFR_IE, 4);                 // UART0 Interrupt Enable
SI_SBIT(ET1, SFR_IE, 3);                 // Timer 1 Interrupt Enable
SI_SBIT(EX1, SFR_IE, 2);                 // External Interrupt 1 Enable
SI_SBIT(ET0, SFR_IE, 1);                 // Timer 0 Interrupt Enable
SI_SBIT(EX0, SFR_IE, 0);                 // External Interrupt 0 Enable

// IP 0xB8
                                       // Bit 7 unused
                                       // Bit 6 unused
SI_SBIT(PT2, SFR_IP, 5);                 // Timer 2 Priority
SI_SBIT(PS0, SFR_IP, 4);                 // UART0 Priority
SI_SBIT(PS, SFR_IP, 4);                  // UART0 Priority
SI_SBIT(PT1, SFR_IP, 3);                 // Timer 1 Priority
SI_SBIT(PX1, SFR_IP, 2);                 // External Interrupt 1 Priority
SI_SBIT(PT0, SFR_IP, 1);                 // Timer 0 Priority
SI_SBIT(PX0, SFR_IP, 0);                 // External Interrupt 0 Priority

// SMB0CN 0xC0
SI_SBIT(BUSY, SFR_SMB0CN, 7);            // SMBus0 Busy
SI_SBIT(ENSMB, SFR_SMB0CN, 6);           // SMBus0 Enable
SI_SBIT(STA, SFR_SMB0CN, 5);             // SMBus0 Start Flag
SI_SBIT(STO, SFR_SMB0CN, 4);             // SMBus0 Stop Flag
SI_SBIT(SI, SFR_SMB0CN, 3);              // SMBus0 Interrupt pending
SI_SBIT(AA, SFR_SMB0CN, 2);              // SMBus0 Assert/Ack Flag
SI_SBIT(SMBFTE, SFR_SMB0CN, 1);          // SMBus0 Bus free timeout enable
SI_SBIT(SMBTOE, SFR_SMB0CN, 0);          // SMBus0 SCL low timeout enable

// TMR2CN 0xC8
SI_SBIT(TF2, SFR_TMR2CN, 7);             // Timer2 Overflow
SI_SBIT(EXF2, SFR_TMR2CN, 6);            // Timer2 External
                                       // Bit 5 unused
                                       // Bit 4 unused
SI_SBIT(EXEN2, SFR_TMR2CN, 3);           // Timer2 External Enable
SI_SBIT(TR2, SFR_TMR2CN, 2);             // Timer2 Run Enable
SI_SBIT(CT2, SFR_TMR2CN, 1);             // Timer2 Counter select
SI_SBIT(CPRL2, SFR_TMR2CN, 0);           // Timer2 Capture select

// TMR3CN 0xC8
SI_SBIT(TF3, SFR_TMR3CN, 7);             // Timer3 Overflow
SI_SBIT(EXF3, SFR_TMR3CN, 6);            // Timer3 External
                                       // Bit 5 unused
                                       // Bit 4 unused
SI_SBIT(EXEN3, SFR_TMR3CN, 3);           // Timer3 External Enable
SI_SBIT(TR3, SFR_TMR3CN, 2);             // Timer3 Run Enable
SI_SBIT(CT3, SFR_TMR3CN, 1);             // Timer3 Counter select
SI_SBIT(CPRL3, SFR_TMR3CN, 0);           // Timer3 Capture select

// TMR4CN 0xC8
SI_SBIT(TF4, SFR_TMR4CN, 7);             // Timer4 Overflow
SI_SBIT(EXF4, SFR_TMR4CN, 6);            // Timer4 External
                                       // Bit 5 unused
                                       // Bit 4 unused
SI_SBIT(EXEN4, SFR_TMR4CN, 3);           // Timer4 External Enable
SI_SBIT(TR4, SFR_TMR4CN, 2);             // Timer4 Run Enable
SI_SBIT(CT4, SFR_TMR4CN, 1);             // Timer4 Counter select
SI_SBIT(CPRL4, SFR_TMR4CN, 0);           // Timer4 Capture select

// PSW 0xD0
SI_SBIT(CY, SFR_PSW, 7);                 // Carry Flag
SI_SBIT(AC, SFR_PSW, 6);                 // Auxiliary Carry Flag
SI_SBIT(F0, SFR_PSW, 5);                 // User Flag 0
SI_SBIT(RS1, SFR_PSW, 4);                // Register Bank Select 1
SI_SBIT(RS0, SFR_PSW, 3);                // Register Bank Select 0
SI_SBIT(OV, SFR_PSW, 2);                 // Overflow Flag
SI_SBIT(F1, SFR_PSW, 1);                 // User Flag 1
SI_SBIT(P, SFR_PSW, 0);                  // Accumulator Parity Flag

// PCA0CN 0xD8
SI_SBIT(CF, SFR_PCA0CN, 7);              // PCA0 Counter Overflow Flag
SI_SBIT(CR, SFR_PCA0CN, 6);              // PCA0 Counter Run Control Bit
SI_SBIT(CCF5, SFR_PCA0CN, 5);            // PCA0 Module 5 Interrupt Flag
SI_SBIT(CCF4, SFR_PCA0CN, 4);            // PCA0 Module 4 Interrupt Flag
SI_SBIT(CCF3, SFR_PCA0CN, 3);            // PCA0 Module 3 Interrupt Flag
SI_SBIT(CCF2, SFR_PCA0CN, 2);            // PCA0 Module 2 Interrupt Flag
SI_SBIT(CCF1, SFR_PCA0CN, 1);            // PCA0 Module 1 Interrupt Flag
SI_SBIT(CCF0, SFR_PCA0CN, 0);            // PCA0 Module 0 Interrupt Flag

// ADC0CN 0xE8
SI_SBIT(AD0EN, SFR_ADC0CN, 7);           // ADC0 Enable
SI_SBIT(AD0TM, SFR_ADC0CN, 6);           // ADC0 Track Mode
SI_SBIT(AD0INT, SFR_ADC0CN, 5);          // ADC0 EOC Interrupt Flag
SI_SBIT(AD0BUSY, SFR_ADC0CN, 4);         // ADC0 Busy Flag
SI_SBIT(AD0CM1, SFR_ADC0CN, 3);          // ADC0 Convert Start Mode Bit 1
SI_SBIT(AD0CM0, SFR_ADC0CN, 2);          // ADC0 Convert Start Mode Bit 0
SI_SBIT(AD0WINT, SFR_ADC0CN, 1);         // ADC0 Window Interrupt Flag
SI_SBIT(AD0LJST, SFR_ADC0CN, 0);         // ADC0 Left Justify

// ADC2CN 0xE8
SI_SBIT(AD2EN, SFR_ADC2CN, 7);           // ADC2 Enable
SI_SBIT(AD2TM, SFR_ADC2CN, 6);           // ADC2 Track Mode
SI_SBIT(AD2INT, SFR_ADC2CN, 5);          // ADC2 EOC Interrupt Flag
SI_SBIT(AD2BUSY, SFR_ADC2CN, 4);         // ADC2 Busy Flag
SI_SBIT(AD2CM2, SFR_ADC2CN, 3);          // ADC2 Convert Start Mode Bit 2
SI_SBIT(AD2CM1, SFR_ADC2CN, 2);          // ADC2 Convert Start Mode Bit 1
SI_SBIT(AD2CM0, SFR_ADC2CN, 1);          // ADC2 Convert Start Mode Bit 0
SI_SBIT(AD2WINT, SFR_ADC2CN, 0);         // ADC2 Window Interrupt Flag

// SPI0CN 0xF8
SI_SBIT(SPIF, SFR_SPI0CN, 7);            // SPI0 Interrupt Flag
SI_SBIT(WCOL, SFR_SPI0CN, 6);            // SPI0 Write Collision Flag
SI_SBIT(MODF, SFR_SPI0CN, 5);            // SPI0 Mode Fault Flag
SI_SBIT(RXOVRN, SFR_SPI0CN, 4);          // SPI0 RX Overrun Flag
SI_SBIT(NSSMD1, SFR_SPI0CN, 3);          // SPI0 Slave Select Mode 1
SI_SBIT(NSSMD0, SFR_SPI0CN, 2);          // SPI0 Slave Select Mode 0
SI_SBIT(TXBMT, SFR_SPI0CN, 1);           // SPI0 TX Buffer Empty Flag
SI_SBIT(SPIEN, SFR_SPI0CN, 0);           // SPI0 Enable

//-----------------------------------------------------------------------------
// Interrupt Priorities
//-----------------------------------------------------------------------------

#define INTERRUPT_INT0             0   // External Interrupt 0
#define INTERRUPT_TIMER0           1   // Timer0 Overflow
#define INTERRUPT_INT1             2   // External Interrupt 1
#define INTERRUPT_TIMER1           3   // Timer1 Overflow
#define INTERRUPT_UART0            4   // UART0
#define INTERRUPT_TIMER2           5   // Timer2 Overflow
#define INTERRUPT_SPI0             6   // SPI0
#define INTERRUPT_SMBUS0           7   // SMBus0 Interface
#define INTERRUPT_ADC0_WINDOW      8   // ADC0 Window Comparison
#define INTERRUPT_PCA0             9   // PCA0 Peripheral
#define INTERRUPT_COMPARATOR0F     10  // Comparator0 Falling
#define INTERRUPT_COMPARATOR0R     11  // Comparator0 Rising
#define INTERRUPT_COMPARATOR1F     12  // Comparator1 Falling
#define INTERRUPT_COMPARATOR1R     13  // Comparator1 Rising
#define INTERRUPT_TIMER3           14  // Timer3 Overflow
#define INTERRUPT_ADC0_EOC         15  // ADC0 End Of Conversion
#define INTERRUPT_TIMER4           16  // Timer4 Overflow
#define INTERRUPT_ADC2_WINDOW      17  // ADC2 Window Comparison
#define INTERRUPT_ADC2_EOC         18  // ADC2 End Of Conversion
                                       // 19 - reserved
#define INTERRUPT_UART1            20  // UART1

//-----------------------------------------------------------------------------
// SFR Page Definitions
//-----------------------------------------------------------------------------

#define  CONFIG_PAGE       0x0F        // SYSTEM AND PORT CONFIGURATION PAGE
#define  LEGACY_PAGE       0x00        // LEGACY SFR PAGE
#define  TIMER01_PAGE      0x00        // TIMER0 AND TIMER1
#define  CPT0_PAGE         0x01        // COMPARATOR0
#define  CPT1_PAGE         0x02        // COMPARATOR1
#define  UART0_PAGE        0x00        // UART0
#define  UART1_PAGE        0x01        // UART1
#define  SPI0_PAGE         0x00        // SPI0
#define  EMI0_PAGE         0x00        // EMIF
#define  ADC0_PAGE         0x00        // ADC0
#define  ADC2_PAGE         0x02        // ADC2
#define  MAC0_PAGE         0x03        // MAC
#define  SMB0_PAGE         0x00        // SMBUS0
#define  TMR2_PAGE         0x00        // TIMER2
#define  TMR3_PAGE         0x01        // TIMER3
#define  TMR4_PAGE         0x02        // TIMER4
#define  DAC0_PAGE         0x00        // DAC0
#define  DAC1_PAGE         0x01        // DAC1
#define  PCA0_PAGE         0x00        // PCA0
#define  REF0_PAGE         0x00        // VREF0
#define  PLL0_PAGE         0x0F        // PLL0

//-----------------------------------------------------------------------------
// Header File PreProcessor Directive
//-----------------------------------------------------------------------------

#endif                                 // #define C8051F120_DEFS_H

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------