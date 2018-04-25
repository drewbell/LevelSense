/****************************************************************************
 Module
   ShiftRegisterRead.c

 Revision
   0.0.1

 Description
   This module acts as the low level interface to a read only shift register.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 04/24/18 19:55 afb    Converting over from shiftRegisterWrite
 
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
#define DATA_IN GPIO_PIN_0

#define SH_LD GPIO_PIN_1
#define SH_LD_HI BIT1HI
#define SH_LD_LO BIT1LO

#define RCLK GPIO_PIN_2
#define RCLK_LO BIT2LO
#define RCLK_HI BIT2HI

#define NUM_BITS 8

#define ALL_BITS (0xff<<2)
#define GET_MSB_IN_LSB(x) ((x & 0x80)>>7)

// an image of the last 8 bits written to the shift register
static uint8_t localRegisterImage=0;

// Create your own function header comment
void SR_Read_Init(void){

// set up port B by enabling the peripheral clock
// and setting the direction of PB1 & PB2 to output, PB0 as input

  // SET BIT 1 TO ENABLE PORT B Clock and wait until peripheral reports that its clock is ready
HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1; 
while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R1) != SYSCTL_PRGPIO_R1) 
	;
			
//Write to the digital enable register to connect pins 1,2 to digital I/O ports
HWREG(GPIO_PORTB_BASE+GPIO_O_DEN) |= (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
	
//Set PB0, PB1, PB2 to be outputs
HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) |= (GPIO_PIN_1 | GPIO_PIN_2);
HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) &= ~GPIO_PIN_0;

	
// start with the SH_LD high and RCLK low 

HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= SH_LD_HI;		//SH_LD PB1
HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= RCLK_LO;		//RCLK PB2

}

// Returns local register image
uint8_t SR_Read_GetCurrReg(void){
  return localRegisterImage;
}

/* Function: SR_Read
 * Designed to read from an 8-bit parallel-input/serial-output such as the
 * MC74HC165 
 */
uint8_t SR_Read(void){
	uint8_t valueRead = 0;
    
  // latch the data from the pins by lowering and raising the SH_LD pin
	HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= SH_LD_LO;
  HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= SH_LD_HI;
	
  
	
	// get data out over shift while pulsing the serial clock
	for (uint8_t bit = 0; bit < NUM_BITS; bit++)  {
    // raise the register clock -> give extra time to set up
    HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= RCLK_HI;
    volatile int wait = 0;
    // lower the register clock
    HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= RCLK_LO;
		// read the value off the 
    uint8_t rawVal = HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) & DATA_IN;
    if(rawVal) printf("Bit%d hi\n\r", bit);
    else printf("Bit%d lo\n\r", bit);
    
    valueRead |= (rawVal << (7 - bit));     // assemble byte to return
  }

    localRegisterImage = valueRead;
    return valueRead;
}

