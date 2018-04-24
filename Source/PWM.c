/****************************************************************************
 Module
   PWM.c

 Revision
   1.0.1

 Description
   This is a PWM module with public functions to set the PWM duty cycle to a desired value between 0 and 100

 Notes
 

 
 To do: clean up commments about 6 and 7

 History
 When           Who     What/Why
 -------------- ---     --------
 01/23/17 20:58 afb      starting PWM module
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "PWM.h"

//relevant includes
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "bitdefs.h"

/*----------------------------- Module Defines ----------------------------*/

// 40,000 ticks per mS assumes a 40Mhz clock, we will use SysClk/32 for PWM
#define PWMTicksPerMS 40000/32
// 40,000 ticks per mS assumes a 40Mhz clock, we will use SysClk/32 for PWM
#define PWMTicksPerUS 40/32
// set 200 Hz frequency so 5mS period
#define PeriodInMS 4
#define MOTOR12TAU 441
#define TEST_PER_IN_US (MOTOR12TAU*125)/100
// define 1ms to be 1000uS
#define USinMS 1000
#define PeriodInUS (100*125)/400
// define a miniumum PWM period in uS
#define MinPWMPeriod 10

// program generator A to go to 1 at rising comare A, 0 on falling compare A  
#define GenA_Normal (PWM_0_GENA_ACTCMPAU_ONE | PWM_0_GENA_ACTCMPAD_ZERO )
// program generator B to go to 1 at rising comare B, 0 on falling compare B  
#define GenB_Normal (PWM_0_GENB_ACTCMPBU_ONE | PWM_0_GENB_ACTCMPBD_ZERO )

#define BitsPerNibble 4
#define PrintInterval 7

//PWM Pin mapping
#define M0PWM0_VAL 4  //PWM Module 0 PWM 0 mux write
#define M0PWM0_S 6  //PWM Module 0 PWM 0 register shift
#define M0PWM1_VAL 4 	//PWM Module 0 PWM 1 mux write
#define M0PWM1_S 7  //PWM Module 0 PWM 1 register shift
#define REG67MASK 0x00ffffff  //PWM pin map register mask for registers 6 and 7 

//printing defines
//#define printPWMinPWM
//#define printCMPA

#define PART1_1

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

void Set100DC ( void );					//set the PWM duty cycle to 100
void Set0DC ( void );						//set the PWM duty cycle to 0 
void RestoreDC (void);					//restores the PWM duty cycle settings to normal after Set100DC or Set0DC calls

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
#ifdef printPWMinPWM
static uint16_t PrintCount = 0;
#endif


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitPWM

 Parameters
     none

 Returns
     none

 Description
     Does initializations required for PWM function
 Notes

 Author
     Drew Bell, 01/23/17, 22:00
****************************************************************************/
void InitPWM ( void )
{
	
	printf("\n\rInit PWM...");
	
  // start by enabling the clock to the PWM Module (PWM0)
  HWREG(SYSCTL_RCGCPWM) |= SYSCTL_RCGCPWM_R0;

// enable the clock to Port B  
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1;

// Select the PWM clock as System Clock/32
  HWREG(SYSCTL_RCC) = (HWREG(SYSCTL_RCC) & ~SYSCTL_RCC_PWMDIV_M) |
    (SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_32);
  
// make sure that the PWM module clock has gotten going
	while ((HWREG(SYSCTL_PRPWM) & SYSCTL_PRPWM_R0) != SYSCTL_PRPWM_R0)
    ;

// disable the PWM while initializing
  HWREG( PWM0_BASE+PWM_O_0_CTL ) = 0;

// program generators to go to 1 at rising compare A/B, 0 on falling compare A/B  
// GenA_Normal = (PWM_0_GENA_ACTCMPAU_ONE | PWM_0_GENA_ACTCMPAD_ZERO )
  HWREG( PWM0_BASE+PWM_O_0_GENA) = GenA_Normal;
// GenB_Normal = (PWM_0_GENB_ACTCMPBU_ONE | PWM_0_GENB_ACTCMPBD_ZERO )
  HWREG( PWM0_BASE+PWM_O_0_GENB) = GenB_Normal;

// Set the PWM period. Since we are counting both up & down, we initialize
// the load register to 1/2 the desired total period. We will also program
// the match compare registers to 1/2 the desired high time  
	
// for Part 1.1: to find electrical time constant, set Period manually to 10-100Hz, 
// enough time for the signal to fully rise	
	#ifdef PART1_1
	HWREG( PWM0_BASE+PWM_O_0_LOAD) = ((TEST_PER_IN_US * PWMTicksPerUS))>>1;
	#endif
	
	//otherwise, if Part1_1 not defined, use standard period
	#ifndef PART1_1
	HWREG( PWM0_BASE+PWM_O_0_LOAD) = ((PeriodInMS * PWMTicksPerMS))>>1;
	#endif
  
// Set the initial Duty cycle on A to 50% by programming the compare value
// to 1/2 the period to count up (or down). Technically, the value to program
// should be Period/2 - DesiredHighTime/2, but since the desired high time is 1/2 
// the period, we can skip the subtract 
  HWREG( PWM0_BASE+PWM_O_0_CMPA) = HWREG( PWM0_BASE+PWM_O_0_LOAD)>>1;

// Set the initial Duty cycle on B to 25% by programming the compare value
// to Period/2 - Period/8  (75% of the period)
  HWREG( PWM0_BASE+PWM_O_0_CMPB) = (HWREG( PWM0_BASE+PWM_O_0_LOAD)) - (((PeriodInMS * PWMTicksPerMS))>>3);

// enable the PWM outputs
	HWREG( PWM0_BASE+PWM_O_ENABLE) |= (PWM_ENABLE_PWM1EN | PWM_ENABLE_PWM0EN);

// now configure the Port B pins to be PWM outputs
// start by selecting the alternate function for PB6 & 7
	HWREG(GPIO_PORTB_BASE+GPIO_O_AFSEL) |= (BIT7HI | BIT6HI);

// now choose to map PWM to those pins, this is a mux value of 4 that we
// want to use for specifying the function on bits 6 & 7
  HWREG(GPIO_PORTB_BASE+GPIO_O_PCTL) = 
		(HWREG(GPIO_PORTB_BASE+GPIO_O_PCTL) & REG67MASK) + (M0PWM1_VAL<<(M0PWM1_S*BitsPerNibble)) + (M0PWM0_VAL<<(M0PWM0_S*BitsPerNibble));


// Enable pins 6 & 7 on Port B for digital I/O
  HWREG(GPIO_PORTB_BASE+GPIO_O_DEN) |= (BIT7HI | BIT6HI);
		
// make pins 6 & 7 on Port B into outputs
  HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) |= (BIT7HI |BIT6HI);

// set the up/down count mode, enable the PWM generator and make
// both generator updates locally synchronized to zero count
  HWREG(PWM0_BASE+ PWM_O_0_CTL) = (PWM_0_CTL_MODE | PWM_0_CTL_ENABLE | PWM_0_CTL_GENAUPD_LS | PWM_0_CTL_GENBUPD_LS);
	
// Programming B to start off high for forward drive
  HWREG( PWM0_BASE+PWM_O_0_GENB) = PWM_0_GENB_ACTZERO_ZERO;
	

printf("Init PWM Done!\n\r");

}

/****************************************************************************
 Function
     SetDC_A

 Parameters
     unit

 Returns
     none

 Description
     This function makes the change needed to set the duty cycle to 100
 Notes

 Author
     Drew Bell, 1/23/17, 19:25
****************************************************************************/
void SetDC_A( uint8_t PWMDuty_A)
{
//Set local NewPWM to PWMDuty
	uint8_t NewPWMDuty_A = PWMDuty_A;

//If NewPWM is less than or equal to zero
	if (NewPWMDuty_A <= 1) {	//NewPWM should be >= as unsigned int, but using defensive coding
		//Call Set0DC
		Set0DC();
	}
//Else if NewPWM is greater than or equal to 100
	else if(NewPWMDuty_A > 99) {
//	Call Set100DC
		Set100DC();
	}
//Else
	else {
		//Call RestoreDC to make sure we’re all enabled
		RestoreDC();
		//Set the duty cycle by programming compare value to be NewPWM (+ some math)
			// Set the initial Duty cycle on A to 50% by programming the compare value
			// to 1/2 the period to count up (or down). Technically, the value to program
			// should be Period/2 - DesiredHighTime/2, but since the desired high time is 1/2 
			// the period, we can skip the subtract 
			HWREG( PWM0_BASE+PWM_O_0_CMPA) = (HWREG( PWM0_BASE+PWM_O_0_LOAD)) - ((HWREG( PWM0_BASE+PWM_O_0_LOAD)*NewPWMDuty_A)/100);
	}
	
	#ifdef printPWMinPWM
	
	if(PrintCount < PrintInterval) {
			PrintCount = PrintCount + 1;
	}
	else if(PrintCount >= PrintInterval){
		printf("\n\rPWMDuty in PWM module is %d\n\r", NewPWMDuty_A);
		PrintCount = 0;
	}
	
	#endif
	
	#ifdef printCMPA
	printf("\n\rLoad = %d, CPMA = %d\n\r", (HWREG( PWM0_BASE+PWM_O_0_LOAD)), HWREG( PWM0_BASE+PWM_O_0_CMPA));
	#endif
	
//Return nothing
}

/****************************************************************************
 Function
     SetPWMPeriod

 Parameters
     uint16_t : the desired period in uSeconds

 Returns
     none

 Description
     This function changes to PWM frequency by updating the PWM period
 Notes

 Author
     Drew Bell, 1/23/17, 19:25
****************************************************************************/
void SetPWMPeriod( uint16_t NewPWMPeriod)
{
//Set local NewFreq to PWMFreq 
	uint16_t NewPeriod = NewPWMPeriod;
	
	uint16_t NewPeriodInMS; 
	uint16_t NewPeriodInUS;
	
	//make sure period is large enough, so if number is smaller than 10us
		if(NewPWMPeriod < 10){
		//print "PWM Period too small, enter a larger one" to terminal
		printf("\n\rPWM Period too small, enter a larger one. Setting to 10uS...\n\r");
		//set to 10us
		NewPWMPeriod = MinPWMPeriod;
		}

	
	//write new period to to load register
		//if value is creater than 1000 uS, use the more accurate clock conversion
		if (NewPeriod >= USinMS) {
			//Assign NewPeriodInMS the value of NewPeriod divided by 1000, the number of 
			NewPeriodInMS = NewPeriod / USinMS;
			//Set the PWM period. With using Up/Down mode, set load register
			// with 1/2 the desired total period
			HWREG( PWM0_BASE+PWM_O_0_LOAD) = ((NewPeriodInMS * PWMTicksPerMS))>>1;
		}
		//else use a less accurate clock divider (less accurate due to integer math roundoff)
		else {
			//For clarity, assign NewPeriodInUS the value of NewPeriod
			NewPeriodInUS = NewPeriod;
			//Set the PWM period. With using Up/Down mode, set load register
			// with 1/2 the desired total period
			HWREG( PWM0_BASE+PWM_O_0_LOAD) = ((NewPeriodInUS * PWMTicksPerUS))>>1;
		}	
		
return;	
}

/*************PRIVATE FUNCTIONS*********************************************/
/***************************************************************************/


/****************************************************************************
 Function
     Set100DC

 Parameters
     none

 Returns
     none

 Description
     This function makes the change needed to set the duty cycle to 100
 Notes

 Author
     Drew Bell, 1/23/17, 19:25
****************************************************************************/
void Set100DC( void )
{
	// To program 100% DC, simply set the action on Zero to set the output to one
  HWREG( PWM0_BASE+PWM_O_0_GENA) = PWM_0_GENA_ACTZERO_ONE;
    
// don't forget to restore the proper actions when the DC drops below 100%
// or rises above 0% 
}


/****************************************************************************
 Function
     Set0DC

 Parameters
     none

 Returns
     none

 Description
     This function makes the change needed to set the duty cycle to 0
 Notes

 Author
     Drew Bell, 1/23/17, 19:25
****************************************************************************/
void Set0DC( void )
{
 // To program 0% DC, simply set the action on Zero to set the output to zero
  HWREG( PWM0_BASE+PWM_O_0_GENA) = PWM_0_GENA_ACTZERO_ZERO;
   
// don't forget to restore the proper actions when the DC drops rises above 0% 
}

/****************************************************************************
 Function
     RestoreDC

 Parameters
     none

 Returns
     none

 Description
     This function restores the PWM function after a Set0DC or Set100DC call
 Notes

 Author
     Drew Bell, 1/23/17, 19:25
****************************************************************************/
void RestoreDC( void )
{

//To restore the previos DC, simply set the action back to the normal actions
	HWREG( PWM0_BASE+PWM_O_0_GENA) = GenA_Normal;
  //* HWREG( PWM0_BASE+PWM_O_0_GENB) = GenB_Normal; 

}



/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

