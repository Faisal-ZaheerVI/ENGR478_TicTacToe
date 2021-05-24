/*
		TIC TAC TOE GAME
		By Faisal Zaheer and Zamar Bravo
		ENGR 478 - 01
		Final Term Project (Spring 2021)
*/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "Nokia5110.h"

// Use SSI0 to send an 8-bit code to the Nokia5110 48x84
// pixel LCD to display text, images, or other information.
// This file has been modified to work with TExaS, which
// sets the PLL to 80 MHz, where earlier versions ran at
// 50 MHz or less.

// Font table, initialization, and other functions based
// off of Nokia_5110_Example from Spark Fun:
// 7-17-2011
// Spark Fun Electronics 2011
// Nathan Seidle
// http://dlnmh9ip6v2uc.cloudfront.net/datasheets/LCD/Monochrome/Nokia_5110_Example.pde

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// Red SparkFun Nokia 5110 (LCD-10168)
// -----------------------------------
// Signal        (Nokia 5110) LaunchPad pin
// 3.3V          (VCC, pin 1) power
// Ground        (GND, pin 2) ground
// SSI0Fss       (SCE, pin 3) connected to PA3
// Reset         (RST, pin 4) connected to PA7
// Data/Command  (D/C, pin 5) connected to PA6
// SSI0Tx        (DN,  pin 6) connected to PA5
// SSI0Clk       (SCLK, pin 7) connected to PA2
// back light    (LED, pin 8) not connected, consists of 4 white LEDs which draw ~80mA total

// Maximum dimensions of the LCD, although the pixels are
// numbered from zero to (MAX-1).  Address may automatically
// be incremented after each transmission.

#define MAX_X                   84
#define MAX_Y                   48

#define DC                      (*((volatile unsigned long *)0x40004100))
#define DC_COMMAND              0
#define DC_DATA                 0x40
#define RESET                   (*((volatile unsigned long *)0x40004200))
#define RESET_LOW               0
#define RESET_HIGH              0x80
#define GPIO_PORTA_DIR_R        (*((volatile unsigned long *)0x40004400))
#define GPIO_PORTA_AFSEL_R      (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_DEN_R        (*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_AMSEL_R      (*((volatile unsigned long *)0x40004528))
#define GPIO_PORTA_PCTL_R       (*((volatile unsigned long *)0x4000452C))
#define SSI0_CR0_R              (*((volatile unsigned long *)0x40008000))
#define SSI0_CR1_R              (*((volatile unsigned long *)0x40008004))
#define SSI0_DR_R               (*((volatile unsigned long *)0x40008008))
#define SSI0_SR_R               (*((volatile unsigned long *)0x4000800C))
#define SSI0_CPSR_R             (*((volatile unsigned long *)0x40008010))
#define SSI0_CC_R               (*((volatile unsigned long *)0x40008FC8))
#define SSI_CR0_SCR_M           0x0000FF00  // SSI Serial Clock Rate
#define SSI_CR0_SPH             0x00000080  // SSI Serial Clock Phase
#define SSI_CR0_SPO             0x00000040  // SSI Serial Clock Polarity
#define SSI_CR0_FRF_M           0x00000030  // SSI Frame Format Select
#define SSI_CR0_FRF_MOTO        0x00000000  // Freescale SPI Frame Format
#define SSI_CR0_DSS_M           0x0000000F  // SSI Data Size Select
#define SSI_CR0_DSS_8           0x00000007  // 8-bit data
#define SSI_CR1_MS              0x00000004  // SSI Master/Slave Select
#define SSI_CR1_SSE             0x00000002  // SSI Synchronous Serial Port
                                            // Enable
#define SSI_SR_BSY              0x00000010  // SSI Busy Bit
#define SSI_SR_TNF              0x00000002  // SSI Transmit FIFO Not Full
#define SSI_CPSR_CPSDVSR_M      0x000000FF  // SSI Clock Prescale Divisor
#define SSI_CC_CS_M             0x0000000F  // SSI Baud Clock Source
#define SSI_CC_CS_SYSPLL        0x00000000  // Either the system clock (if the
                                            // PLL bypass is in effect) or the
                                            // PLL output (default)
#define SYSCTL_RCGC1_R          (*((volatile unsigned long *)0x400FE104))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC1_SSI0       0x00000010  // SSI0 Clock Gating Control
#define SYSCTL_RCGC2_GPIOA      0x00000001  // port A Clock Gating Control


// This table contains the hex values that represent pixels
// for a font that is 5 pixels wide and 8 pixels high
static const char ASCII[][5] = {
  {0x00, 0x00, 0x00, 0x00, 0x00} // 20
  ,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
  ,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
  ,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
  ,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
  ,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
  ,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
  ,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
  ,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
  ,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
  ,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
  ,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
  ,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
  ,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
  ,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
  ,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
  ,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
  ,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
  ,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
  ,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
  ,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
  ,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
  ,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
  ,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
  ,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
  ,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
  ,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
  ,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
  ,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
  ,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
  ,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
  ,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
  ,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
  ,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
  ,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
  ,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
  ,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
  ,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
  ,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
  ,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
  ,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
  ,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
  ,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
  ,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
  ,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
  ,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
  ,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
  ,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
  ,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
  ,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
  ,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
  ,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
  ,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
  ,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
  ,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
  ,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
  ,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
  ,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
  ,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
  ,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
  ,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c '\'
  ,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
  ,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
  ,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
  ,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
  ,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
  ,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
  ,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
  ,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
  ,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
  ,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
  ,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
  ,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
  ,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
  ,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
  ,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
  ,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
  ,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
  ,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
  ,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
  ,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
  ,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
  ,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
  ,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
  ,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
  ,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
  ,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
  ,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
  ,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
  ,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
  ,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
  ,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
  ,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
  ,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
  ,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ~
//  ,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f DEL
  ,{0x1f, 0x24, 0x7c, 0x24, 0x1f} // 7f UT sign
};

enum typeOfWrite{
  COMMAND,                              // the transmission is an LCD command
  DATA                                  // the transmission is data
};
// The Data/Command pin must be valid when the eighth bit is
// sent.  The SSI module has hardware input and output FIFOs
// that are 8 locations deep.  Based on the observation that
// the LCD interface tends to send a few commands and then a
// lot of data, the FIFOs are not used when writing
// commands, and they are used when writing data.  This
// ensures that the Data/Command pin status matches the byte
// that is actually being transmitted.
// The write command operation waits until all data has been
// sent, configures the Data/Command pin for commands, sends
// the command, and then waits for the transmission to
// finish.
// The write data operation waits until there is room in the
// transmit FIFO, configures the Data/Command pin for data,
// and then adds the data to the transmit FIFO.

/* Joystick pins
VCC - +3.3V
GND - Ground
X-axis: PE1
Y-axis: PE2
K-axis: PB5
*/

uint32_t ui32ADC0Value[4]; 	// Reading of X-Axis
uint32_t ui32ADC1Value[4]; 	// Reading of Y-Axis
volatile uint32_t ui32_Xpin;
volatile uint32_t ui32_Ypin;

char gameBoard [5][5] = {
{' ', '|', ' ', '|', ' '}, 
{'-', '+', '-', '+', '-'}, 
{' ', '|', ' ', '|', ' '}, 
{'-', '+', '-', '+', '-'},
{' ', '|', ' ', '|', ' '} 
};

// Global variables.
int choice = 0;
char playerSign = 'X';
char computerSign = 'O';
float x_volt;
float y_volt;
bool isPlayerTurn = false;
bool isComputerTurn = false;
bool choiceMade = false;
bool playerWin = false;
bool computerWin = false;
bool endGame = false;

// Port Init
void PortFunctionInit(void)
{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);	// using ADC0 and ADC1
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);	// Using PE1 - X, PE2 - Y
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // For Switches/LEDs (Mainly SW2 switch at first)
	
		GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2); // Configures PE1 and PE2 as ADC analog input pins.

    //First open the lock and select the bits we want to modify in the GPIO commit register.
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;
		//Now modify the configuration of the pins that we unlocked.
		GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0); // Enable pin PF0 for GPIOInput
		GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4); // Enable pin PF4 for GPIOInput
	
		//Enable pull-up on PF4 and PF0 (2^4 = 0x10 and 2^0 = 0x01 respectively)
		GPIO_PORTF_PUR_R |= 0x11; 
}


void
Interrupt_Init(void)
{
  IntEnable(INT_GPIOF);  							// enable interrupt 30 in NVIC (GPIOF)
	IntPrioritySet(INT_GPIOF, 0x00); 		// configure GPIOF interrupt priority as 0
	GPIO_PORTF_IM_R |= 0x11;   		// arm interrupt on PF0 and PF4
	GPIO_PORTF_IS_R &= ~0x11;     // PF0 and PF4 are edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;   	// PF0 and PF4 not both edges trigger 
  //GPIO_PORTF_IEV_R &= ~0x11;  	// PF0 and PF4 falling edge event
	IntMasterEnable();       		// globally enable interrupt
}


//ADC0 initializaiton
void ADC0_Init(void)
{
		SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); // configures the system clock to be 40MHz
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);	//activate the clock of ADC0
		SysCtlDelay(6);	//insert a few cycles after enabling the peripheral to allow the clock to be fully activated.

		ADCSequenceDisable(ADC0_BASE, 2); //disables ADC0 before the conf. is complete
		ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0); // will use ADC0, SS1, processor-trigger, priority 0
		
		ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH2);
		ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH2);
		ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_CH2);
		ADCSequenceStepConfigure(ADC0_BASE, 2, 3, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END); // Ch. 2 = PE1
		IntPrioritySet(INT_ADC0SS2, 0x00);  	 // configure ADC0 SS2 interrupt priority as 0
		IntEnable(INT_ADC0SS2);    				// enable interrupt 31 in NVIC (ADC0 SS2)
		ADCIntEnableEx(ADC0_BASE, ADC_INT_SS2);      // arm interrupt of ADC0 SS2
	
		ADCSequenceEnable(ADC0_BASE, 2); //enable ADC0
}

// ADC1 initializaiton
void ADC1_Init(void)
{
		SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); // configures the system clock to be 40MHz
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);	//activate the clock of ADC1
		SysCtlDelay(6);	//insert a few cycles after enabling the peripheral to allow the clock to be fully activated.

		ADCSequenceDisable(ADC1_BASE, 2); //disables ADC1 before the conf. is complete
		ADCSequenceConfigure(ADC1_BASE, 2, ADC_TRIGGER_PROCESSOR, 0); // will use ADC1, SS2, processor-trigger, priority 0
		ADCSequenceStepConfigure(ADC1_BASE, 2, 0, ADC_CTL_CH1);
		ADCSequenceStepConfigure(ADC1_BASE, 2, 1, ADC_CTL_CH1);
		ADCSequenceStepConfigure(ADC1_BASE, 2, 2, ADC_CTL_CH1);
		ADCSequenceStepConfigure(ADC1_BASE, 2, 3, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END); // Ch. 1 = PE2 (Y axis.)
		IntPrioritySet(INT_ADC1SS2, 0x00);  	 // configure ADC1 SS2 interrupt priority as 0
		IntEnable(INT_ADC1SS2); // enable interrupt 31 in NVIC (ADC1 SS2)
		ADCIntEnableEx(ADC1_BASE, ADC_INT_SS2); // arm interrupt of ADC1 SS2
	
		ADCSequenceEnable(ADC1_BASE, 2); //enable ADC1
}

// Prints the gameBoard array onto the LCD screen.
// (From left to right, top to down)
void drawBoard() {
	for (int i = 0; i < 5; i++) {
		Nokia5110_SetCursor(0, i);
		for (int j = 0; j < 5; j++) {
			Nokia5110_OutChar(gameBoard[i][j]);
		}
	}
}

bool checkOpenPosition(int position) {
	// Returns false is selected position is unavailable.
	bool openSpace = false;
	// If selected position is empty (there is a space character in
	// the specific position in the gameBoard array), return true,
	// meaning that the selected position is an unoccupied position.
	switch (position) {
		case 1: if (gameBoard[0][0] == ' ')
			openSpace = true;
			break;
		case 2: if (gameBoard[0][2] == ' ')
			openSpace = true;
			break;
		case 3: if (gameBoard[0][4] == ' ')
			openSpace = true;
			break;
		case 4: if (gameBoard[2][0] == ' ')
			openSpace = true;
			break;
		case 5: if (gameBoard[2][2] == ' ')
			openSpace = true;
			break;
		case 6: if (gameBoard[2][4] == ' ')
			openSpace = true;
			break;
		case 7: if (gameBoard[4][0] == ' ')
			openSpace = true;
			break;
		case 8: if (gameBoard[4][2] == ' ')
			openSpace = true;
			break;
		case 9: if (gameBoard[4][4] == ' ')
			openSpace = true;
			break;
	}
	return openSpace;
}

// Clears the Tic Tac Toe game board of any signs. ('X' and 'O')
void clearBoard() {
	Nokia5110_SetCursor(0, 0);
	Nokia5110_OutString(" ");
	Nokia5110_SetCursor(2, 0);
	Nokia5110_OutString(" ");
	Nokia5110_SetCursor(4, 0);
	Nokia5110_OutString(" ");
	
	Nokia5110_SetCursor(0, 2);
	Nokia5110_OutString(" ");
	Nokia5110_SetCursor(2, 2);
	Nokia5110_OutString(" ");
	Nokia5110_SetCursor(4, 2);
	Nokia5110_OutString(" ");
	
	Nokia5110_SetCursor(0, 4);
	Nokia5110_OutString(" ");
	Nokia5110_SetCursor(2, 4);
	Nokia5110_OutString(" ");
	Nokia5110_SetCursor(4, 4);
	Nokia5110_OutString(" ");
}

void updatePos(int position) {
	// Based on selected position # (1-9),
	// the gameBoard array will update with
	// the player's sign ('X').
	switch (position) {
		case 1: gameBoard[0][0] = playerSign; Nokia5110_SetCursor(0, 0); Nokia5110_OutChar(playerSign); break;
		case 2: gameBoard[0][2] = playerSign; Nokia5110_SetCursor(2, 0); Nokia5110_OutChar(playerSign); break;
		case 3: gameBoard[0][4] = playerSign; Nokia5110_SetCursor(4, 0); Nokia5110_OutChar(playerSign); break;
		case 4: gameBoard[2][0] = playerSign; Nokia5110_SetCursor(0, 2); Nokia5110_OutChar(playerSign); break;
		case 5: gameBoard[2][2] = playerSign; Nokia5110_SetCursor(2, 2); Nokia5110_OutChar(playerSign); break;
		case 6: gameBoard[2][4] = playerSign; Nokia5110_SetCursor(4, 2); Nokia5110_OutChar(playerSign); break;
		case 7: gameBoard[4][0] = playerSign; Nokia5110_SetCursor(0, 4); Nokia5110_OutChar(playerSign); break;
		case 8: gameBoard[4][2] = playerSign; Nokia5110_SetCursor(2, 4); Nokia5110_OutChar(playerSign); break;
		case 9: gameBoard[4][4] = playerSign; Nokia5110_SetCursor(4, 4); Nokia5110_OutChar(playerSign); break;
	}
}

void updateComputerPos(int position) {
	// Based on the randomly selected position # (1-9),
	// the gameBoard array will update with the 
	// computer's sign ('O').
	switch (position) {
		case 1: gameBoard[0][0] = computerSign; Nokia5110_SetCursor(0, 0); Nokia5110_OutChar(computerSign); break;
		case 2: gameBoard[0][2] = computerSign; Nokia5110_SetCursor(2, 0); Nokia5110_OutChar(computerSign); break;
		case 3: gameBoard[0][4] = computerSign; Nokia5110_SetCursor(4, 0); Nokia5110_OutChar(computerSign); break;
		case 4: gameBoard[2][0] = computerSign; Nokia5110_SetCursor(0, 2); Nokia5110_OutChar(computerSign); break;
		case 5: gameBoard[2][2] = computerSign; Nokia5110_SetCursor(2, 2); Nokia5110_OutChar(computerSign); break;
		case 6: gameBoard[2][4] = computerSign; Nokia5110_SetCursor(4, 2); Nokia5110_OutChar(computerSign); break;
		case 7: gameBoard[4][0] = computerSign; Nokia5110_SetCursor(0, 4); Nokia5110_OutChar(computerSign); break;
		case 8: gameBoard[4][2] = computerSign; Nokia5110_SetCursor(2, 4); Nokia5110_OutChar(computerSign); break;
		case 9: gameBoard[4][4] = computerSign; Nokia5110_SetCursor(4, 4); Nokia5110_OutChar(computerSign); break;
	}
}

/* --- CHECK WIN/TIE CONDITIONS --- */
bool checkWin() {
	// Checks if the player has won the game.
	if ((gameBoard[0][0] == playerSign && gameBoard[2][2] == playerSign && gameBoard[4][4] == playerSign)
		|| (gameBoard[0][4] == playerSign && gameBoard[2][2] == playerSign && gameBoard[4][0] == playerSign)
		|| (gameBoard[0][0] == playerSign && gameBoard[0][2] == playerSign && gameBoard[0][4] == playerSign)
		|| (gameBoard[2][0] == playerSign && gameBoard[2][2] == playerSign && gameBoard[2][4] == playerSign)
		|| (gameBoard[4][0] == playerSign && gameBoard[4][2] == playerSign && gameBoard[4][4] == playerSign)
		|| (gameBoard[0][0] == playerSign && gameBoard[2][0] == playerSign && gameBoard[4][0] == playerSign)
		|| (gameBoard[0][2] == playerSign && gameBoard[2][2] == playerSign && gameBoard[4][2] == playerSign)
		|| (gameBoard[0][4] == playerSign && gameBoard[2][4] == playerSign && gameBoard[4][4] == playerSign)) 
	{
			playerWin = true;
			return true;
	}
	
	// Checks if the computer has won the game.
	else if ((gameBoard[0][0] == computerSign && gameBoard[2][2] == computerSign && gameBoard[4][4] == computerSign)
		|| (gameBoard[0][4] == computerSign && gameBoard[2][2] == computerSign && gameBoard[4][0] == computerSign)
		|| (gameBoard[0][0] == computerSign && gameBoard[0][2] == computerSign && gameBoard[0][4] == computerSign)
		|| (gameBoard[2][0] == computerSign && gameBoard[2][2] == computerSign && gameBoard[2][4] == computerSign)
		|| (gameBoard[4][0] == computerSign && gameBoard[4][2] == computerSign && gameBoard[4][4] == computerSign)
		|| (gameBoard[0][0] == computerSign && gameBoard[2][0] == computerSign && gameBoard[4][0] == computerSign)
		|| (gameBoard[0][2] == computerSign && gameBoard[2][2] == computerSign && gameBoard[4][2] == computerSign)
		|| (gameBoard[0][4] == computerSign && gameBoard[2][4] == computerSign && gameBoard[4][4] == computerSign)) 
	{
			computerWin = true;
			return true;
	}
	// Checks for no more open positions, and neither player or computer has won, aka a tie game condition.
	else if (gameBoard[0][0] != ' ' && gameBoard[0][2] != ' ' && gameBoard[0][4] != ' '
				&& gameBoard[2][0] != ' ' && gameBoard[2][2] != ' ' && gameBoard[2][4] != ' '
				&& gameBoard[4][0] != ' ' && gameBoard[4][2] != ' ' && gameBoard[4][4] != ' ') 
	{
			return true;
	}
		// Return false if neither side has won.
		return false;
}

/* --- CHECK END GAME CONDITIONS --- */
void checkEndGame() {
	if (checkWin()) {
		if (playerWin) {
			Nokia5110_SetCursor(6, 2);
			Nokia5110_OutString("You");
			Nokia5110_SetCursor(6, 3);
			Nokia5110_OutString("Win!");
		}
		else if (computerWin) {
			Nokia5110_SetCursor(6, 2);
			Nokia5110_OutString("You");
			Nokia5110_SetCursor(6, 3);
			Nokia5110_OutString("Lose.");
		}
		endGame = checkWin();
	}
}

void computerTurn() {	
	int rand();
	int compInput = rand() % 9 + 1;
	
	bool posOpen = checkOpenPosition(compInput);
	while (!posOpen) {
		compInput = rand() % 9 + 1;
		posOpen = checkOpenPosition(compInput);
	}
	updateComputerPos(compInput);
	char computerInput = compInput+'0';
	
	//SysCtlDelay(SysCtlClockGet() / (1 * 3)); //delay ~1000 msec = 1 second
	
	choiceMade = true;
	isComputerTurn = false;
	
	if (!isComputerTurn && choiceMade) {
		checkEndGame();
		drawBoard();
		choiceMade = false;
		isPlayerTurn = true;
	}
}

void GPIOPortF_Handler(void) 
{
		//IntDisable(INT_GPIOF);
		NVIC_EN0_R &= ~0x40000000; 
		SysCtlDelay(160000);
		//IntEnable(INT_GPIOF);
		NVIC_EN0_R |= 0x40000000; 
		
		//SW2 (PF0) has action
		//if((GPIO_PORTF_DATA_R & 0x01) == 0x01)
		if(GPIO_PORTF_RIS_R&0x01)			
		{
			if (!isPlayerTurn && choiceMade) {
				checkEndGame();
				drawBoard();
				choiceMade = false;
				isComputerTurn = true;
				computerTurn();
			}
		}
}


// ADC interrupt handler
void ADC0_Handler(void)
{
	if (isPlayerTurn) {
		ADCIntClear(ADC0_BASE, 2);
		ADCProcessorTrigger(ADC0_BASE, 2);
	
		ADCIntClear(ADC1_BASE, 2);
		ADCProcessorTrigger(ADC1_BASE, 2);

		// ADC Values
		ADCSequenceDataGet(ADC0_BASE, 2, ui32ADC0Value);	// gives X-axis reading
		ADCSequenceDataGet(ADC1_BASE, 2, ui32ADC1Value);	// gives Y-axis reading
	
		// Grabs X&Y readings -> integers
		ui32_Xpin = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3]) / 4;	
		ui32_Ypin = (ui32ADC1Value[0] + ui32ADC1Value[1] + ui32ADC1Value[2] + ui32ADC1Value[3]) / 4;

		// Converts readings to Voltage values (~0V - 3.3V)	
		x_volt = ( (ui32_Xpin * 3.3) / 4095 );
		y_volt = ( (ui32_Ypin * 3.3) / 4095 );  
	
		/* --- Diagonal Cases --- */
		// Top Left
		if (x_volt >= 2.8 && y_volt <= 0.5) {
			choice = 3;
			bool posOpen = checkOpenPosition(choice);
			if (posOpen) {
				clearBoard();
				
				Nokia5110_SetCursor(0, 0);
				Nokia5110_OutChar(playerSign);
				
				//SW2 is pressed
				if(GPIO_PORTF_RIS_R&0x01 && !choiceMade) {
						choiceMade = true;
						updatePos(1);
						isPlayerTurn = false;
				}
			}
		}
		
		// Top Right
		else if (x_volt <= 0.5 && y_volt <= 0.5) {
			choice = 3;
			bool posOpen = checkOpenPosition(choice);
			if (posOpen) {
				clearBoard();
				
				Nokia5110_SetCursor(4, 0);
				Nokia5110_OutChar(playerSign);
				
				//SW2 is pressed
				if(GPIO_PORTF_RIS_R&0x01 && !choiceMade) {
						choiceMade = true;
						updatePos(3);
						isPlayerTurn = false;
				}
			}
		}
		
		// Bottom Left
		else if (x_volt >= 2.8 && y_volt >= 2.8) {
			choice = 7;
			bool posOpen = checkOpenPosition(choice);
			if (posOpen) {
				clearBoard();
				
				Nokia5110_SetCursor(0, 4);
				Nokia5110_OutChar(playerSign);
				
				//SW2 is pressed
				if(GPIO_PORTF_RIS_R&0x01 && !choiceMade) {
						choiceMade = true;
						updatePos(7);
						isPlayerTurn = false;
				}
			}
		}
		
		// Bottom Right
		else if (x_volt <= 0.5 && y_volt >= 2.8) {
			choice = 9;
			bool posOpen = checkOpenPosition(choice);
			if (posOpen) {
				clearBoard();
				
				Nokia5110_SetCursor(4, 4);
				Nokia5110_OutChar(playerSign);
				
				//SW2 is pressed
				if(GPIO_PORTF_RIS_R&0x01 && !choiceMade) {
						choiceMade = true;
						updatePos(9);
						isPlayerTurn = false;
				}
			}
		}
	
		// Default (In Middle)
		else if (y_volt <= 2.3 && y_volt >= 1 && x_volt <= 2.3 && x_volt >= 1) {
			choice = 5;
			bool posOpen = checkOpenPosition(choice);
			if (posOpen) {
				Nokia5110_SetCursor(0, 0);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(2, 0);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(4, 0);
				Nokia5110_OutString(" ");
				
				Nokia5110_SetCursor(0, 2);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(4, 2);
				Nokia5110_OutString(" ");
				
				Nokia5110_SetCursor(0, 4);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(2, 4);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(4, 4);
				Nokia5110_OutString(" ");
				
				Nokia5110_SetCursor(2, 2);
				Nokia5110_OutChar(playerSign);
				
				//SW2 is pressed
				if(GPIO_PORTF_RIS_R&0x01 && !choiceMade) {
						choiceMade = true;
						updatePos(5);
						isPlayerTurn = false;
				}
			}
		}
		
		// Goes down
		else if (y_volt > 2.3) {
			choice = 8;
			bool posOpen = checkOpenPosition(choice);
			if (posOpen) {
				Nokia5110_SetCursor(0, 0);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(2, 0);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(4, 0);
				Nokia5110_OutString(" ");
				
				Nokia5110_SetCursor(0, 2);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(2, 2);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(4, 2);
				Nokia5110_OutString(" ");
				
				Nokia5110_SetCursor(0, 4);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(4, 4);
				Nokia5110_OutString(" ");
				
				Nokia5110_SetCursor(2, 4);
				Nokia5110_OutChar(playerSign);
				
				//SW2 is pressed
				if(GPIO_PORTF_RIS_R&0x01 && !choiceMade) {
					choiceMade = true;
					updatePos(8);
					isPlayerTurn = false;
				}
			}
		}
		// Goes up
		else if (y_volt < 1) {
			choice = 2;
			bool posOpen = checkOpenPosition(choice);
			if (posOpen) {
				Nokia5110_SetCursor(0, 0);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(4, 0);
				Nokia5110_OutString(" ");
				
				Nokia5110_SetCursor(0, 2);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(2, 2);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(4, 2);
				Nokia5110_OutString(" ");
				
				Nokia5110_SetCursor(0, 4);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(2, 4);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(4, 4);
				Nokia5110_OutString(" ");
				
				Nokia5110_SetCursor(2, 0);
				Nokia5110_OutChar(playerSign);
				
				//SW2 is pressed
				if(GPIO_PORTF_RIS_R&0x01) {
					choiceMade = true;
					updatePos(2);
					isPlayerTurn = false;
				}
			}
		}
		
		// Goes Left
		else if (x_volt > 2.3) {
			choice = 4;
			bool posOpen = checkOpenPosition(choice);
			if (posOpen) {
				Nokia5110_SetCursor(0, 0);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(2, 0);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(4, 0);
				Nokia5110_OutString(" ");
				
				Nokia5110_SetCursor(2, 2);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(4, 2);
				Nokia5110_OutString(" ");
				
				Nokia5110_SetCursor(0, 4);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(2, 4);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(4, 4);
				Nokia5110_OutString(" ");
				
				Nokia5110_SetCursor(0, 2);
				Nokia5110_OutChar(playerSign);
				
				//SW2 is pressed
				if(GPIO_PORTF_RIS_R&0x01 && !choiceMade) {
					choiceMade = true;
					updatePos(4);
					isPlayerTurn = false;
				}
			}
		}
		
		// Goes Right
		else if (x_volt < 1) {
			choice = 6;
			bool posOpen = checkOpenPosition(choice);
			if (posOpen) {
				Nokia5110_SetCursor(0, 0);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(2, 0);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(4, 0);
				Nokia5110_OutString(" ");
				
				Nokia5110_SetCursor(0, 2);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(2, 2);
				Nokia5110_OutString(" ");
				
				Nokia5110_SetCursor(0, 4);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(2, 4);
				Nokia5110_OutString(" ");
				Nokia5110_SetCursor(4, 4);
				Nokia5110_OutString(" ");
				
				Nokia5110_SetCursor(4, 2);
				Nokia5110_OutChar(playerSign);
				
				//SW2 is pressed
				if(GPIO_PORTF_RIS_R&0x01 && !choiceMade) {
					choiceMade = true;
					updatePos(6);
					isPlayerTurn = false;
				}
			}
		}
		else {
			choice = 0;
			Nokia5110_Clear();
			Nokia5110_SetCursor(6, 0);
			Nokia5110_OutString("ERROR");
		}
	}
}

// This is a helper function that sends an 8-bit message to the LCD.
// inputs: type     COMMAND or DATA
//         message  8-bit code to transmit
// outputs: none
// assumes: SSI0 and port A have already been initialized and enabled
void static lcdwrite(enum typeOfWrite type, char message){
  if(type == COMMAND){
                                        // wait until SSI0 not busy/transmit FIFO empty
    while((SSI0_SR_R&SSI_SR_BSY)==SSI_SR_BSY){};
    DC = DC_COMMAND;
    SSI0_DR_R = message;                // command out
                                        // wait until SSI0 not busy/transmit FIFO empty
    while((SSI0_SR_R&SSI_SR_BSY)==SSI_SR_BSY){};
  } else{
    while((SSI0_SR_R&SSI_SR_TNF)==0){}; // wait until transmit FIFO not full
    DC = DC_DATA;
    SSI0_DR_R = message;                // data out
  }
}

// Initializes Nokia 5110 48x84 LCD by sending the proper
// commands to the PCD8544 driver.  One new feature of the
// LM4F120 is that its SSIs can get their baud clock from
// either the system clock or from the 16 MHz precision
// internal oscillator.
// assumes: system clock rate of 80 MHz
void Nokia5110_Init(void){
  volatile unsigned long delay;
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_SSI0;  // activate SSI0
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // activate port A
  delay = SYSCTL_RCGC2_R;               // allow time to finish activating
  GPIO_PORTA_DIR_R |= 0xC0;             // make PA6,7 out
  GPIO_PORTA_AFSEL_R |= 0x2C;           // enable alt funct on PA2,3,5
  GPIO_PORTA_AFSEL_R &= ~0xC0;          // disable alt funct on PA6,7
  GPIO_PORTA_DEN_R |= 0xEC;             // enable digital I/O on PA2,3,5,6,7
                                        // configure PA2,3,5 as SSI
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFF0F00FF)+0x00202200;
                                        // configure PA6,7 as GPIO
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0x00FFFFFF)+0x00000000;
  GPIO_PORTA_AMSEL_R &= ~0xEC;          // disable analog functionality on PA2,3,5,6,7
  SSI0_CR1_R &= ~SSI_CR1_SSE;           // disable SSI
  SSI0_CR1_R &= ~SSI_CR1_MS;            // master mode
                                        // configure for system clock/PLL baud clock source
  SSI0_CC_R = (SSI0_CC_R&~SSI_CC_CS_M)+SSI_CC_CS_SYSPLL;
                                        // clock divider for 3.33 MHz SSIClk (80 MHz PLL/24)
                                        // SysClk/(CPSDVSR*(1+SCR))
                                        // 80/(24*(1+0)) = 3.33 MHz (slower than 4 MHz)
  SSI0_CPSR_R = (SSI0_CPSR_R&~SSI_CPSR_CPSDVSR_M)+24; // must be even number
  SSI0_CR0_R &= ~(SSI_CR0_SCR_M |       // SCR = 0 (3.33 Mbps data rate)
                  SSI_CR0_SPH |         // SPH = 0
                  SSI_CR0_SPO);         // SPO = 0
                                        // FRF = Freescale format
  SSI0_CR0_R = (SSI0_CR0_R&~SSI_CR0_FRF_M)+SSI_CR0_FRF_MOTO;
                                        // DSS = 8-bit data
  SSI0_CR0_R = (SSI0_CR0_R&~SSI_CR0_DSS_M)+SSI_CR0_DSS_8;
  SSI0_CR1_R |= SSI_CR1_SSE;            // enable SSI

  RESET = RESET_LOW;                    // reset the LCD to a known state
  for(delay=0; delay<10; delay=delay+1);// delay minimum 100 ns
  RESET = RESET_HIGH;                   // negative logic

  lcdwrite(COMMAND, 0x21);              // chip active; horizontal addressing mode (V = 0); use extended instruction set (H = 1)
                                        // set LCD Vop (contrast), which may require some tweaking:
  lcdwrite(COMMAND, CONTRAST);          // try 0xB1 (for 3.3V red SparkFun), 0xB8 (for 3.3V blue SparkFun), 0xBF if your display is too dark, or 0x80 to 0xFF if experimenting
  lcdwrite(COMMAND, 0x04);              // set temp coefficient
  lcdwrite(COMMAND, 0x14);              // LCD bias mode 1:48: try 0x13 or 0x14

  lcdwrite(COMMAND, 0x20);              // we must send 0x20 before modifying the display control mode
  lcdwrite(COMMAND, 0x0C);              // set display control to normal mode: 0x0D for inverse
}

// Print a character to the Nokia 5110 48x84 LCD.  The
// character will be printed at the current cursor position,
// the cursor will automatically be updated, and it will
// wrap to the next row or back to the top if necessary.
// One blank column of pixels will be printed on either side
// of the character for readability.  Since characters are 8
// pixels tall and 5 pixels wide, 12 characters fit per row,
// and there are six rows.
// inputs: data  character to print
// outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_OutChar(unsigned char data){
  lcdwrite(DATA, 0x00);                 // blank vertical line padding
  for(int i=0; i<5; i++){
    lcdwrite(DATA, ASCII[data - 0x20][i]);
  }
  lcdwrite(DATA, 0x00);                 // blank vertical line padding
}

// Print a string of characters to the Nokia 5110 48x84 LCD.
// The string will automatically wrap, so padding spaces may
// be needed to make the output look optimal.
// inputs: ptr  pointer to NULL-terminated ASCII string
// outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_OutString(unsigned char *ptr){
  while(*ptr){
    Nokia5110_OutChar((unsigned char)*ptr);
    ptr = ptr + 1;
  }
}

// Output a 16-bit number in unsigned decimal format with a
// fixed size of five right-justified digits of output.
// Inputs: n  16-bit unsigned number
// Outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_OutUDec(unsigned short n){
  if(n < 10){
    Nokia5110_OutString("    ");
    Nokia5110_OutChar(n+'0'); /* n is between 0 and 9 */
  } else if(n<100){
    Nokia5110_OutString("   ");
    Nokia5110_OutChar(n/10+'0'); /* tens digit */
    Nokia5110_OutChar(n%10+'0'); /* ones digit */
  } else if(n<1000){
    Nokia5110_OutString("  ");
    Nokia5110_OutChar(n/100+'0'); /* hundreds digit */
    n = n%100;
    Nokia5110_OutChar(n/10+'0'); /* tens digit */
    Nokia5110_OutChar(n%10+'0'); /* ones digit */
  }
  else if(n<10000){
    Nokia5110_OutChar(' ');
    Nokia5110_OutChar(n/1000+'0'); /* thousands digit */
    n = n%1000;
    Nokia5110_OutChar(n/100+'0'); /* hundreds digit */
    n = n%100;
    Nokia5110_OutChar(n/10+'0'); /* tens digit */
    Nokia5110_OutChar(n%10+'0'); /* ones digit */
  }
  else {
    Nokia5110_OutChar(n/10000+'0'); /* ten-thousands digit */
    n = n%10000;
    Nokia5110_OutChar(n/1000+'0'); /* thousands digit */
    n = n%1000;
    Nokia5110_OutChar(n/100+'0'); /* hundreds digit */
    n = n%100;
    Nokia5110_OutChar(n/10+'0'); /* tens digit */
    Nokia5110_OutChar(n%10+'0'); /* ones digit */
  }
}

// Move the cursor to the desired X- and Y-position.  The
// next character will be printed here.  X=0 is the leftmost
// column.  Y=0 is the top row.
// inputs: newX  new X-position of the cursor (0<=newX<=11)
//         newY  new Y-position of the cursor (0<=newY<=5)
// outputs: none
void Nokia5110_SetCursor(unsigned char newX, unsigned char newY){
  if((newX > 11) || (newY > 5)){        // bad input
    return;                             // do nothing
  }
  // multiply newX by 7 because each character is 7 columns wide
  lcdwrite(COMMAND, 0x80|(newX*7));     // setting bit 7 updates X-position
  lcdwrite(COMMAND, 0x40|newY);         // setting bit 6 updates Y-position
}

// Clears the LCD by writing zeros to the entire screen and
// reset the cursor to (0,0) (top left corner of screen).
void Nokia5110_Clear(void){
  for (int i = 0; i < (MAX_X*MAX_Y/8); i++){
    lcdwrite(DATA, 0x00);
  }
  Nokia5110_SetCursor(0, 0);
}

// Fill the whole screen by drawing a 48x84 bitmap image.
// inputs: ptr  pointer to 504 byte bitmap
// outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_DrawFullImage(const char *ptr){
  Nokia5110_SetCursor(0, 0);
  for(int i=0; i<(MAX_X*MAX_Y/8); i++){
    lcdwrite(DATA, ptr[i]);
  }
}

int main(void) 
{
		// --- GAME INITIALIZATION --- //
		Nokia5110_Init(); // Initializes LCD screen.
		Nokia5110_Clear(); // Clears the screen of any pixels.
		drawBoard(); // Draws the initial, empty game board.
	
		PortFunctionInit(); // Initializes the ports on the microcontroller.
		Interrupt_Init(); // Initializes general interrupts. (For the GPIOPortF_Handler)
		// Initializes both ADC0 and ADC1
		ADC0_Init();
		ADC1_Init();
		SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ);
		IntMasterEnable(); // enable processor interrupts
		ADCProcessorTrigger(ADC0_BASE, 2);
		ADCProcessorTrigger(ADC1_BASE, 2);
		
		isPlayerTurn = true; // Starts out with the player selecting a position.
		
    while (!endGame) // Loop while there is no endGame condition triggered.
    {
			if (!isPlayerTurn && choiceMade) {
				checkEndGame();
				drawBoard();
				isComputerTurn = true;
				choiceMade = false;
				computerTurn();
			}
			else if (!isComputerTurn && choiceMade) {
				checkEndGame();
				drawBoard();
				isPlayerTurn = true;
				choiceMade = false;
			}	
			// END
    }
}
