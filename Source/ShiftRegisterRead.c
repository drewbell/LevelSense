/****************************************************************************
 Module
   ShiftRegisterWrite.c

 Revision
   0.0.1

 Description
   This module acts as the low level interface to a write only shift register.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 10/24/16 19:55 afb    first rodeo
 
****************************************************************************/
// the common headers for C99 types 
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

// the headers to access the GPIO subsystem
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"

// the headers to access the TivaWare Library
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "BITDEFS.H"

// readability defines
#define DATA GPIO_PIN_0

#define SCLK GPIO_PIN_1
#define SCLK_HI BIT1HI
#define SCLK_LO BIT1LO

#define RCLK GPIO_PIN_2
#define RCLK_LO BIT2LO
#define RCLK_HI BIT2HI

#define ALL_BITS (0xff<<2)
#define GET_MSB_IN_LSB(x) ((x & 0x80)>>7)

// an image of the last 8 bits written to the shift register
static uint8_t LocalRegisterImage=0;

// Create your own function header comment
void SR_Init(void){

// set up port B by enabling the peripheral clock and setting the direction of PB1 & PB2 to output
// also set up port E by enabling peripheral clock and setting the direction of PE2 to output 

// SET BIT 1 TO ENABLE PORT B Clock and wait until peripheral reports that its clock is ready
HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1; 
while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R1) != SYSCTL_PRGPIO_R1) 
	;
			
//Write to the digital enable register to connect pins 1,2 to digital I/O ports
HWREG(GPIO_PORTB_BASE+GPIO_O_DEN) |= (GPIO_PIN_1 | GPIO_PIN_2);
	
//Set PB0, PB1, PB2 to be outputs
HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) |= (GPIO_PIN_1 | GPIO_PIN_2);

// also set up port E by enabling peripheral clock and setting the direction of PE2 to output 

// SET BIT 4 TO ENABLE PORT E Clock and wait until peripheral reports that its clock is ready
HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R4; 
while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R4) != SYSCTL_PRGPIO_R4) 
	;
	
//Write to the digital enable register to connect pin 2 to digital I/O ports
HWREG(GPIO_PORTE_BASE+GPIO_O_DEN) |= (GPIO_PIN_2);
	
//Set PE2 to be outputs
HWREG(GPIO_PORTE_BASE+GPIO_O_DIR) |= (GPIO_PIN_2);
	
// JEC start with the data & sclk lines low and the RCLK line high
HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS)) &= BIT3LO;		//data line PE2
HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= SCLK_LO;		//SCLK PB1
HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= RCLK_HI;		//RCLK PB2

}

// Returns local register image
uint8_t SR_GetCurrentRegister(void){
  return LocalRegisterImage;
}

// Create your own function header comment
void SR_Write(uint8_t NewValue){
	uint8_t BitValue = 0;
  uint8_t BitCounter = 0;
	
	LocalRegisterImage = NewValue; // save a local copy

	// lower the register clock
	HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= RCLK_LO;
	
	// shift out the data while pulsing the serial clock
	for (BitCounter=0; BitCounter<8; BitCounter++)  {
			
		//Isolate the MSB of NewValue, put it into the LSB position and output		
		//GET_MSB Macro function:
		// Isolate the MSB of NewValue, put it into the LSB position and output
		// Shift bit in question over to the LSB
					
		BitValue = GET_MSB_IN_LSB((NewValue<<BitCounter));
		if(BitValue)
			HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS)) |= BIT2HI;
		else HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS)) &= BIT2LO; 	
						
	//print values of BitValue and NewValue for each point in the loop
	//printf("\nDataOnPin = BitValue = %d, NewValue = %d\r\n", BitValue, NewValue); 
					
	//Push the value out of the shift register output 
	// raise SCLK
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= SCLK_HI;
	// lower SCLK
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= SCLK_LO;
	}
	 
		// raise the register clock to latch the new data
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= RCLK_HI;  
	
}

