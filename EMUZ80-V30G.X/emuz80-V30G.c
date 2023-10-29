/*
 * Copyright (c) 2023 @Gazelle8087
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*  Modified by  Gazelle https://twitter.com/Gazelle8087
 *  1st release 2023.10.29
 *
 *  Target: PIC18F47Q84 (not verified for 18F47Q43 at 1st release)
 *  IDE: MPLAB X v6.0
 *  Compiler: XC8 v2.36
 * 
 * References:
 * EMUZ80-V20 https://github.com/satoshiokue/EMUZ80-V20
 * 8086Basic https://github.com/satoshiokue/8086_NASCOM_BASIC
 * V20BASIC http://www.amy.hi-ho.ne.jp/officetetsu/storage/sbcv20_datapack.zip
 * Universal monitor https://electrelic.com/electrelic/node/1317
 */

 /*!
 * PIC18F47Q84/PIC18F47Q83 ROM RAM and UART emulation firmware
 * This single source file contains all code
 *
 * Target: EMUZ80 with V30
 * Compiler: MPLAB XC8 v2.41
 *
 * Modified by Satoshi Okue https://twitter.com/S_Okue
 * Version 0.1 2023/10/14
 *
 *  PIC18F47Q84 ROM RAM and UART emulation firmware
 *  This single source file contains all code
 *
 *  Original source Written by Tetsuya Suzuki
 *  https://github.com/vintagechips/emuz80
 *  Target: EMUZ80 - The computer with only Z80 and PIC18F47Q43
 *  Compiler: MPLAB XC8 v2.36
 *
 *  Modified by  Gazelle https://twitter.com/Gazelle8087
 *  1st release 2022.7.17
 *
 *  Target: PIC18F47Q84
 *  IDE: MPLAB X v6.0
 *  Compiler: XC8 v2.36
 *
 *  References: source code written by yyhayami
 *  for the CLC configuration and memory mapping/access procedure.
 *  https://github.com/yyhayami/emuz80_hayami/tree/main/emuz80_clc.X
*/

// CONFIG1
#pragma config FEXTOSC = OFF	// External Oscillator Selection (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_64MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG2
#pragma config CLKOUTEN = OFF	// Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = ON		// PRLOCKED One-Way Set Enable bit (PRLOCKED bit can be cleared and set only once)
#pragma config CSWEN = ON		// Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON		// Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#ifndef _18F47Q43
#pragma config JTAGEN = OFF
#pragma config FCMENP = OFF
#pragma config FCMENS = OFF
#endif

// CONFIG3
#pragma config MCLRE = EXTMCLR	// MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = ON		// Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
#pragma config IVT1WAY = ON		// IVTLOCK bit One-way set enable bit (IVTLOCKED bit can be cleared and set only once)
#pragma config LPBOREN = OFF	// Low Power BOR Enable bit (Low-Power BOR disabled)
#pragma config BOREN = SBORDIS	// Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG4
#pragma config BORV = VBOR_1P9	// Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 1.9V)
#pragma config ZCD = OFF		// ZCD Disable bit (ZCD module is disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = OFF	// PPSLOCK bit One-Way Set Enable bit (PPSLOCKED bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON		// Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON			// Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)
#pragma config XINST = OFF		// Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG5
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF		// WDT operating mode (WDT Disabled; SWDTEN is ignored)

// CONFIG6
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC		// WDT input clock selector (Software Control)

// CONFIG7
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF		// Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF		// Storage Area Flash enable bit (SAF disabled)
#ifdef _18F47Q43
#pragma config DEBUG = OFF		// Background Debugger (Background Debugger disabled)
#endif

// CONFIG8
#pragma config WRTB = OFF		// Boot Block Write Protection bit (Boot Block not Write protected)
#pragma config WRTC = OFF		// Configuration Register Write Protection bit (Configuration registers not Write protected)
#pragma config WRTD = OFF		// Data EEPROM Write Protection bit (Data EEPROM not Write protected)
#pragma config WRTSAF = OFF		// SAF Write protection bit (SAF not Write Protected)
#pragma config WRTAPP = OFF		// Application Block write protection bit (Application Block not write protected)

// CONFIG10
#pragma config CP = OFF		 	// PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

#ifndef _18F47Q43
// CONFIG9
#pragma config BOOTPINSEL = RC5	// CRC on boot output pin selection (CRC on boot output pin is RC5)
#pragma config BPEN = OFF		// CRC on boot output pin enable bit (CRC on boot output pin disabled)
#pragma config ODCON = OFF		// CRC on boot output pin open drain bit (Pin drives both high-going and low-going signals)

// CONFIG11
#pragma config BOOTSCEN = OFF	// CRC on boot scan enable for boot area (CRC on boot will not include the boot area of program memory in its calculation)
#pragma config BOOTCOE = HALT	// CRC on boot Continue on Error for boot areas bit (CRC on boot will stop device if error is detected in boot areas)
#pragma config APPSCEN = OFF	// CRC on boot application code scan enable (CRC on boot will not include the application area of program memory in its calculation)
#pragma config SAFSCEN = OFF	// CRC on boot SAF area scan enable (CRC on boot will not include the SAF area of program memory in its calculation)
#pragma config DATASCEN = OFF	// CRC on boot Data EEPROM scan enable (CRC on boot will not include data EEPROM in its calculation)
#pragma config CFGSCEN = OFF	// CRC on boot Config fuses scan enable (CRC on boot will not include the configuration fuses in its calculation)
#pragma config COE = HALT		// CRC on boot Continue on Error for non-boot areas bit (CRC on boot will stop device if error is detected in non-boot areas)
#pragma config BOOTPOR = OFF	// Boot on CRC Enable bit (CRC on boot will not run)

// CONFIG12
#pragma config BCRCPOLT = 0xFF	// Boot Sector Polynomial for CRC on boot bits 31-24 (Bits 31:24 of BCRCPOL are 0xFF)

// CONFIG13
#pragma config BCRCPOLU = 0xFF	// Boot Sector Polynomial for CRC on boot bits 23-16 (Bits 23:16 of BCRCPOL are 0xFF)

// CONFIG14
#pragma config BCRCPOLH = 0xFF	// Boot Sector Polynomial for CRC on boot bits 15-8 (Bits 15:8 of BCRCPOL are 0xFF)

// CONFIG15
#pragma config BCRCPOLL = 0xFF	// Boot Sector Polynomial for CRC on boot bits 7-0 (Bits 7:0 of BCRCPOL are 0xFF)

// CONFIG16
#pragma config BCRCSEEDT = 0xFF	// Boot Sector Seed for CRC on boot bits 31-24 (Bits 31:24 of BCRCSEED are 0xFF)

// CONFIG17
#pragma config BCRCSEEDU = 0xFF	// Boot Sector Seed for CRC on boot bits 23-16 (Bits 23:16 of BCRCSEED are 0xFF)

// CONFIG18
#pragma config BCRCSEEDH = 0xFF	// Boot Sector Seed for CRC on boot bits 15-8 (Bits 15:8 of BCRCSEED are 0xFF)

// CONFIG19
#pragma config BCRCSEEDL = 0xFF	// Boot Sector Seed for CRC on boot bits 7-0 (Bits 7:0 of BCRCSEED are 0xFF)

// CONFIG20
#pragma config BCRCEREST = 0xFF	// Boot Sector Expected Result for CRC on boot bits 31-24 (Bits 31:24 of BCRCERES are 0xFF)

// CONFIG21
#pragma config BCRCERESU = 0xFF	// Boot Sector Expected Result for CRC on boot bits 23-16 (Bits 23:16 of BCRCERES are 0xFF)

// CONFIG22
#pragma config BCRCERESH = 0xFF	// Boot Sector Expected Result for CRC on boot bits 15-8 (Bits 15:8 of BCRCERES are 0xFF)

// CONFIG23
#pragma config BCRCERESL = 0xFF	// Boot Sector Expected Result for CRC on boot bits 7-0 (Bits 7:0 of BCRCERES are 0xFF)

// CONFIG24
#pragma config CRCPOLT = 0xFF	// Non-Boot Sector Polynomial for CRC on boot bits 31-24 (Bits 31:24 of CRCPOL are 0xFF)

// CONFIG25
#pragma config CRCPOLU = 0xFF	// Non-Boot Sector Polynomial for CRC on boot bits 23-16 (Bits 23:16 of CRCPOL are 0xFF)

// CONFIG26
#pragma config CRCPOLH = 0xFF	// Non-Boot Sector Polynomial for CRC on boot bits 15-8 (Bits 15:8 of CRCPOL are 0xFF)

// CONFIG27
#pragma config CRCPOLL = 0xFF	// Non-Boot Sector Polynomial for CRC on boot bits 7-0 (Bits 7:0 of CRCPOL are 0xFF)

// CONFIG28
#pragma config CRCSEEDT = 0xFF	// Non-Boot Sector Seed for CRC on boot bits 31-24 (Bits 31:24 of CRCSEED are 0xFF)

// CONFIG29
#pragma config CRCSEEDU = 0xFF	// Non-Boot Sector Seed for CRC on boot bits 23-16 (Bits 23:16 of CRCSEED are 0xFF)

// CONFIG30
#pragma config CRCSEEDH = 0xFF	// Non-Boot Sector Seed for CRC on boot bits 15-8 (Bits 15:8 of CRCSEED are 0xFF)

// CONFIG31
#pragma config CRCSEEDL = 0xFF	// Non-Boot Sector Seed for CRC on boot bits 7-0 (Bits 7:0 of CRCSEED are 0xFF)

// CONFIG32
#pragma config CRCEREST = 0xFF	// Non-Boot Sector Expected Result for CRC on boot bits 31-24 (Bits 31:24 of CRCERES are 0xFF)

// CONFIG33
#pragma config CRCERESU = 0xFF	// Non-Boot Sector Expected Result for CRC on boot bits 23-16 (Bits 23:16 of CRCERES are 0xFF)

// CONFIG34
#pragma config CRCERESH = 0xFF	// Non-Boot Sector Expected Result for CRC on boot bits 15-8 (Bits 15:8 of CRCERES are 0xFF)

// CONFIG35
#pragma config CRCERESL = 0xFF	// Non-Boot Sector Expected Result for CRC on boot bits 7-0 (Bits 7:0 of CRCERES are 0xFF)
#endif
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>

#define RAM_TOP 0x0000		//RAM top address
#ifdef _18F47Q43
#define RAM_SIZE 0x1000 //4K bytes
#else
#define RAM_SIZE 0x2000 //8K bytes
#endif

//#define emulation8080			// for V30
#define UART_DREG 0x0000		// Data REG
#define UART_CREG 0x0001		// Control REG

#define _XTAL_FREQ 64000000UL

// V30 RAM equivalent
volatile unsigned char ram[RAM_SIZE] __at(0x0600);

// V30 ROM equivalent, see end of this file
extern const unsigned char rom[];	// 0x0000-0xFFFF

//Address Bus
union {
	unsigned int w;			//16 bits Address
	struct {
		unsigned char l;	//Address low
		unsigned char h;	//Address high
	};
} ab;

// UART3 Transmit
void putch(char c) {
	while(!U3TXIF);	// Wait or Tx interrupt flag set
	U3TXB = c;		// Write data
}

/*
// UART3 Recive
char getch(void) {
	while(!U3RXIF);	// Wait for Rx interrupt flag set
	return U3RXB;	// Read data
}
*/

void __interrupt(irq(U3RX),base(8)) UART_receive(){
	LATE2 = U3RXIF;
//	U3RXIF = 0;
}
// Never called, logically
void __interrupt(irq(default),base(8)) Default_ISR(){}

// main routine
void main(void) {

	int i;

	// System initialize
	OSCFRQ = 0x08;	// 64MHz internal OSC

//========== CLC pin assign ===========    
    CLCIN0PPS = 0x02;   // RA2 <- ALE
	CLCIN2PPS = 0x08;	// RB0 <- M/IO
	CLCIN3PPS = 0x0A;	// RB2 <- /RD
	CLCIN6PPS = 0x09;	// RB1 <- /WR

//========== CLC1 latch ALE ==========
    CLCSELECT = 0;      // select CLC1  

//	CLCnSEL0 = 127;		// NC
	CLCnSEL0 = 0;		// CLCIN0PPS <- ALE
	CLCnSEL1 = 127;		// NC 
	CLCnSEL2 = 127;		// NC
	CLCnSEL3 = 127;		// NC

	CLCnGLS0 = 0x02;    // C <- ALE no invert
	CLCnGLS1 = 0x04;    // D <- 1(0 invert)
	CLCnGLS2 = 0;		// R <- 0 (controlled by firm ware)
	CLCnGLS3 = 0;		// S <- 0

	CLCnPOL = 0x80;     // Q invert CLK no invert D no invert
	CLCnCON = 0x84;     // D-FF, no interrupt

//========== CLC2 momory read decode ==========
	CLCSELECT = 1;      // select CLC2 

	CLCnSEL0 = 2;       // CLCIN2PPS <- <M/IO
	CLCnSEL1 = 3;		// CLCIN3PPS <- /RD
	CLCnSEL2 = 127;     // NC
	CLCnSEL3 = 127;     // NC

	CLCnGLS0 = 0x02;    // M/IO no invert
	CLCnGLS1 = 0x04;    // /RD invert
	CLCnGLS2 = 0x10;    // 1(0 inverted) for AND gate
	CLCnGLS3 = 0x40;    // 1(0 inverted) for AND gate

	CLCnPOL = 0x00;     // Not inverted
	CLCnCON = 0x82;     // 4 input AND, no interrupt

//========== CLC3 momory write decode ==========
	CLCSELECT = 2;      // select CLC3 

	CLCnSEL0 = 2;       // CLCIN2PPS <- <M/IO
	CLCnSEL1 = 6;		// CLCIN6PPS <- /WR
	CLCnSEL2 = 127;     // NC
	CLCnSEL3 = 127;     // NC

	CLCnGLS0 = 0x02;    // M/IO no invert
	CLCnGLS1 = 0x04;    // /WR invert
	CLCnGLS2 = 0x10;    // 1(0 inverted) for AND gate
	CLCnGLS3 = 0x40;    // 1(0 inverted) for AND gate

	CLCnPOL = 0x00;     // Not inverted
	CLCnCON = 0x82;     // 4 input AND, no interrupt

//========== CLC8 NCO clock gating ==========
	CLCSELECT = 7;      // select CLC8  

	CLCnSEL0 = 8;       // Fosc
	CLCnSEL1 = 51;		// CLC1 
	CLCnSEL2 = 127;     // NC
	CLCnSEL3 = 127;     // NC

	CLCnGLS0 = 0x02;    // Fosc no invert
	CLCnGLS1 = 0x08;    // CLC1 no invert
	CLCnGLS2 = 0x10;    // 1(0 inverted) for AND gate
	CLCnGLS3 = 0x40;    // 1(0 inverted) for AND gate

	CLCnPOL = 0x00;     // Not inverted
	CLCnCON = 0x82;     // 4 input AND, no interrupt

//=====================================================

	// V30 clock(RA1) by NCO FDC mode
	RA1PPS = 0x3f;		// RA3 assign NCO1
	ANSELA1 = 0;		// Disable analog function
	TRISA1 = 0;			// NCO output pin
//	NCO1INC = (unsigned int)(CLK_V30 / 30.5175781);
	NCO1INC = 0x40000;	//8.0MHz

	NCO1CLK = 0x1A;     // Clock source CLC8
//	NCO1CLK = 0x00;     // Clock source Fosc

	NCO1PFM = 0;		// FDC mode
	NCO1OUT = 1;		// NCO output enable
	NCO1EN = 1;			// NCO enable
//---------------------------------------------------end of clock setup
	// RESET (RA5) output pin
	ANSELA5 = 0;	// Disable analog function
	LATA5 = 1;		// Reset=High
	TRISA5 = 0;		// Set as output

	// Address bus AD15-AD8 pin
	ANSELD = 0x00;	// Disable analog function
	WPUD = 0xff;	// Week pull up
	TRISD = 0xff;	// Set as input

	// Address bus AD7-AD0 pin
	ANSELC = 0x00;	// Disable analog function
	WPUC = 0xff;	// Week pull up
	TRISC = 0xff;	// Set as input

	// /RD (RB2) input pin
	ANSELB2 = 0;	// Disable analog function
	WPUB2 = 1;		// Week pull up
	TRISB2 = 1;		// Set as intput

	// /WR (RB1) input pin
	ANSELB1 = 0;	// Disable analog function
	WPUB1 = 1;		// Week pull up
	TRISB1 = 1;		// Set as intput

	// IO#/M(RB0) input pin
	ANSELB0 = 0;	// Disable analog function
	WPUB0 = 1;		// Week pull up
	TRISB0 = 1;		// Set as intput

	// ASTB (RA2) input pin
	ANSELA2 = 0;	// Disable analog function
	WPUA2 = 1;		// Week pull up
	TRISA2 = 1;		// Set as intput

	// READY (RA4) output pin
	ANSELA4 = 0;	// Disable analog function
	LATA4 = 1;
	TRISA4 = 0;		// Set as output

	// INT (RE2) output pin
	ANSELE2 = 0;	// Disable analog function
	LATE2 = 0;
	TRISE2 = 0;		// Set as output

	// NMI (RA0) output pin
	ANSELA0 = 0;	// Disable analog function
	LATA0 = 0;
	TRISA0 = 0;		// Set as output

	// /UBE (RB4) input pin
	ANSELB4 = 0;	// Disable analog function
	WPUB4 = 1;		// Week pull up
	TRISB4 = 1;		// Set as intput

	// /POLL (RE0) output pin
	ANSELE0 = 0;	// Disable analog function
	LATE0 = 0;
	TRISE0 = 0;		// Set as output

	// HLDRQ (RE1) output pin
	ANSELE1 = 0;	// Disable analog function
	LATE1 = 0;
	TRISE1 = 0;		// Set as output

	// S/LG (RB3) output pin
	ANSELB3 = 0;	// Disable analog function
	LATB3 = 1;
	TRISB3 = 0;		// Set as output

	// UART3 initialize
	U3BRG = 416;	// 9600bps @ 64MHz
	U3RXEN = 1;		// Receiver enable
	U3TXEN = 1;		// Transmitter enable

	// UART3 Receiver
	ANSELA7 = 0;	// Disable analog function
	TRISA7 = 1;		// RX set as input
	U3RXPPS = 0x07;	// RA7->UART3:RX3;

	// UART3 Transmitter
	ANSELA6 = 0;	// Disable analog function
	LATA6 = 1;		// Default level
	TRISA6 = 0;		// TX set as output
	RA6PPS = 0x26;	// RA6->UART3:TX3;

	U3ON = 1;		// Serial port enable

	printf("\r\nMEZV30 %2.3fMHz\r\n",NCO1INC * 30.5175781 / 1000000);

	// RAM clear
	for(i = 0; i < RAM_SIZE; i++) {
		ram[i] = 0;
	}

	// Unlock IVT
	IVTLOCK = 0x55;
	IVTLOCK = 0xAA;
	IVTLOCKbits.IVTLOCKED = 0x00;

	// Default IVT base address
	IVTBASE = 0x000008;

	// Lock IVT
	IVTLOCK = 0x55;
	IVTLOCK = 0xAA;
	IVTLOCKbits.IVTLOCKED = 0x01;

	U3RXIF = 0;
	GIE = 1;		// Global interrupt enable
//	U3RXIE = 1;
	BSR = 0;
	CLCSELECT= 0;
	TBLPTRU = 1;
	LATA5 = 0;		// Release reset
//---------------------------------------------------------------
	asm(						// V30 start
	"bra	ALE_poll		\n" // Skip post process
	"after_RD:				\n"	// Return point after RD cycle<---------
	"btfss	PORTB,2,c		\n"	// Wait for /RD = H
	"bra	after_RD		\n" // loop
	"after_RD2:				\n"	// /RD polling cut trial
	"setf	TRISC,c			\n"	// Set data bus(L) as input
	"setf	TRISD,c			\n"	// Set data bus(H) as input
	"after_WR:				\n"	// Return point aftter WR cycle<--------
	"btfsc	PIR9,0,c		\n"	// U3RXIF = H?
	"bsf	LATE,2,c		\n"	// then INT = H
	);
//-----------------------------------------------------------------
	asm(
	"ALE_poll:				\n" // T1 cycle
	"btfsc	CLCDATA,0,b		\n"	// Wait for CLC1(ALE latch) = H
	"bra	ALE_poll		\n"	// loop
	"movff  PORTC,TBLPTRL   \n"	// TBLPTRL = PORTC(lower address)
	"movff  PORTD,TBLPTRH   \n"	// TBLPTRH = PORTD(upper address)
	"bsf    CLCnPOL,2,b     \n"	// NCO clock restart (G3POL = 1;)
	"bcf	LATA,4,c		\n" // READY = L // LATA4 = 0;
	"bcf    CLCnPOL,2,b     \n" // G3POL = 0;
	);
//		while(RA2);				// Wait for ASTB = L //no need
//----------------------------------------------------------------
	asm(							// memory read
	"btfss	CLCDATA,1,b		\n"		// CLC2 = H?
	"bra	mem_read_exit	\n"		// otherwise next
	"clrf	TRISC,c			\n"		// Set data bus(L) as output
	"clrf	TRISD,c			\n"		// Set data bus(H) as output
	);
	if((TBLPTRH & 0x80) == 0x80) {	// A15 = H?
		asm(						// ROM read
		"bcf	TBLPTRL,0,c		\n"	// TBLPTRL &= 0xfffe;
		"tblrd  *+				\n" // TABLAT = ROM(TBLPTR),TBLPTR++
		"movff  TABLAT,LATC		\n" // PORTB = TABLAT
		"tblrd  *				\n" // TABLAT = ROM(TBLPTR)
		"bsf	LATA,4,c		\n" // READY = H
		"movff  TABLAT,LATD		\n" // PORTB = TABLAT
//		"bsf	LATA,4,c		\n" // READY = H
		"bra	after_RD		\n"	// jmp to post process
//		"bra	after_RD2		\n"	// jmp to quick post process(trial)
		);
	} else if(TBLPTRH <  RAM_SIZE / 256 ){	// within RAMSIZE?
		asm(						// RAM read
		"movff  TBLPTRL,FSR0L	\n" // FSR0L = TBLPTRL(lower address)
		"bcf	FSR0L,0,c		\n"	// FSR0L &= 0xfffe;
		"movf   TBLPTRH,w		\n" // W = TBLPTRH(upper address)
		"addlw  high _ram		\n" // W =+ RAM locate address
		"movwf  FSR0H,c			\n" // FSR0H = W
		"movff  indf0,LATC		\n" // PORTD = RAM(FSR0H,FSR0L)
		"bsf	FSR0L,0,c		\n"	// FSR0L |= 0x0001;
		"bsf	LATA,4,c		\n" // READY = H
		"movff  indf0,LATD		\n" // PORTD = RAM(FSR0H,FSR0L)
//		"bsf	LATA,4,c		\n" // READY = H
		"bra	after_RD		\n"	// jmp to post process
//		"bra	after_RD2		\n"	// jmp to quick post process(trial)
		);
	}
	asm(
	"mem_read_exit:	\n"
	);
//-------------------------------------------------------------------
	asm(							// Memory write
	"btfss	CLCDATA,2,b		\n"		// CLC3 = H?
	"bra	mem_write_exit	\n"		// otherwise next
	);
		if(TBLPTRH <  RAM_SIZE / 256 ){
		asm(
		"movff  TBLPTRL,FSR0L	\n" // FSR0L = TBLPTRL(lower address)
		"movf   TBLPTRH,w		\n" // W = TBLPTRH(High address)
		"addlw  high _ram		\n" // W =+ RAM locate address
		"movwf  FSR0H,c			\n" // FSR0H = W
		);
		if((TBLPTRL & 0x01) == 0){
			asm(					// write even
			"bcf	FSR0L,0,c	\n"	// FSR0L &= 0xfffe
			"movff  PORTC,indf0	\n" // RAM(FSR0) = PORTC
			);
		}
		if(RB4 == 0){
			asm(					// write odd
			"bsf	FSR0L,0,c	\n"	// FSR0L |= 0x0001
			"movff  PORTD,indf0	\n" // RAM(FSR0) = PORTD
			);
		}
	}
	asm(
	"bsf	LATA,4,c	\n"			// READY = High
	"bra	after_WR	\n"			// jump to post process
	"mem_write_exit:	\n"
	);
//---------------------------------------------------------------------
	if(!RB2) {								// IO read
		asm(
		"clrf	TRISC,c	\n"					// Set data bus(L) as output
		"clrf	TRISD,c	\n"					// Set data bus(U) as output
		);
		if(TBLPTRL == UART_CREG) {			// PIR9
			LATD = (U3RXIF + U3RXIF + U3TXIF);
		} else if(TBLPTRL == UART_DREG) {	// U3RXB
			LATC = U3RXB;					// Out U3RXB
			LATE2 = 0;						// INTR = L after receive
		}
		asm(
		"bsf	LATA,4,c	\n"				// READY = High
		"bra	after_RD	\n"				// jmp to post process
		);
	}
//-------------------------------------------------------------
	if(!RB1) {							// IO write
		if(TBLPTRL == UART_DREG) {
			U3TXB = PORTC;				// Write into U3TXB
		}
	}
//-------------------------------------------------------------
	asm(								// INTA cycle
	"bsf	LATA,4,c	\n"				// READY = High
	"bra	after_WR	\n"				// jmp to post process
	);
//-------------------------------------------------------------
}
const unsigned char rom[] __at(0x10000) = {
};

const unsigned char rom1[] __at(0x18000) = {
// Nascom Basic
#ifdef emulation8080
	#include "V20BASIC_8000_0000_814b.c"		// 0000:814B
#else
	#include "8088basic_8000_0000_8081.c"		// 0000:8081
#endif
};

const unsigned char rom2[] __at(0x1c000) = {
// Universal Monitor
	#include "unimon_MEZV30_c000_f000_c000.c"	// F000:C000
};

const unsigned char reset[] __at(0x1fff0) = {
	0xea, 0x00, 0xc0, 0x00, 0xf0				// jmp F000:C000
};
