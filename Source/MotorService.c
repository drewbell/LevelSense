/****************************************************************************
 Module
   MotorService.c

 Revision
   1.0.1

 Description
   This is a simple service to run an DC motor by tying together a AD input and PWM output.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/23/17 09:58 afb     starting module code
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/

//this module include
#include "MotorService.h"

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

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BITDEFS.H"

//application includes
#include "PWM.h"
#include "ButtonDebounce.h"
#include "ADService.h"
#include "ShiftRegisterWrite.h"



/*----------------------------- Module Defines ----------------------------*/

#define Enable12_ON BIT0HI
#define Enable12_OFF BIT0LO
#define ON 1
#define OFF 0
#define TOGGLE -1
#define AD2PWMSCALE 41
#define SHIFTED_AD2RPM 595
#define AD_SHIFT_S 3

#define ALL_BITS (0xff<<2)

#define	AD_MIN	80
#define AD2RPM 73

#define	RPM_MIN	0
#define RPM_MAX 55

//printf defines
//#define printTargetRPM

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/



/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static uint8_t SpeedFetchPeriod = 100;   // time between [ms] fetches of the analog value for PWM speed setting
static uint8_t DriveMode;								// DriveMode, currently 1 = ON, 0 = OFF
static uint8_t DriveDir;								// Direction of spinning, 1 = FWD, -1 = REVERSE
static uint16_t TargetRPM;							// Target RPM calculated from AD value


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitializeMotorService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does initializes pins and modules needed
     for the function of this module.

 Notes

 Author
     Drew Bell, 01/25/17, 10:00
****************************************************************************/
bool InitializeMotorService ( uint8_t Priority )
{
  ES_Event ThisEvent;

  MyPriority = Priority;
	
	printf("\n\rInit MotorService...");
	
  // Initialize DriveMode to be 0 (on = 1, off = 0)
  	DriveMode = OFF;
  // Initialize DriveDir to be 1 (forwards = 1, backwards  -1)
  	DriveDir = ON; 
  // Call InitPWM to set up PWM
  	InitPWM();

  //initialize PB0 as GPIO outputs for the 12Enable (like lab 5)

  // SET BIT 1 TO ENABLE PORT B Clock and wait until peripheral reports that its clock is ready
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1; 
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R1) != SYSCTL_PRGPIO_R1) 
    ;

  //Write to the digital enable register to connect pin 0 to digital I/O port
  HWREG(GPIO_PORTB_BASE+GPIO_O_DEN) |= (GPIO_PIN_0);
  
  //Set PB0 to be output
  HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) |= (GPIO_PIN_0);

  // Enable EN12 line
  HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= (Enable12_ON);
	
  //set first step timer using initialization of DutyFetchPeriod
  ES_Timer_InitTimer(TargetSpeedUpdateTimer, SpeedFetchPeriod);
	
	printf("Init MotorService Done!");
	
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService( MyPriority, ThisEvent) == true)
  {
      return true;
  }else
  {
      return false;
  }
}

/****************************************************************************
 Function
     PostMotorService

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     Drew Bell, 01/25/17, 19:25
****************************************************************************/
bool PostMotorService( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunMotorService

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   implements logic to control drive inputs and timer between steps
 Notes
   
 Author
   Drew Bell, 01/25/12, 15:23
****************************************************************************/
ES_Event RunMotorService( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
	uint16_t NewADValue;       //declare temporary local var
	  
//if ThisEvent is DBButtonDown_Mode
if(ThisEvent.EventType == ES_DBButtonDown_Mode) {
	//increment DriveMode to next mode (0-1 right now, ON and OFF. Can easily add more modes)
	//if drivemode is ON
	if(DriveMode == ON) {
		//change DriveMode from ON to OFF
		DriveMode = OFF;
		//disable H-bridge by lowering EN12 line for L293
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= (Enable12_OFF);
	}
	//else if drivemode is OFF
	else if(DriveMode == OFF){
		//change DriveMode from OFF to ON 
		DriveMode = ON;
		//Enable EN12 line for L293
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= (Enable12_ON);
	}
}

//if ThisEvent is DBButtonDown_Dir
if(ThisEvent.EventType == ES_DBButtonDown_Dir) {
	//Toggle DriveDir direction (1 = forward, -1 = reverse)

	DriveDir = TOGGLE*DriveDir;
		
	/***
	Feature pipeline: add commands to change drive direction by switching to driving the other PWM pin to get reverse current through the motor
	***/
	}
	
  //if ThisEvent was ES_TIMEOUT and EventParam was DutyUpdateTimer
	if((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == TargetSpeedUpdateTimer)){
		//call GetADReading from ADService to get ADValue
		NewADValue = GetADReading();
		//calculate Target RPM value based on reading. Taking AD value (0-4095) times 4 and divided by modified_AD2RPM const to get RPM 0 to 55
		//TargetRPM = ((NewADValue<<AD_SHIFT_S)/SHIFTED_AD2RPM);	
		TargetRPM = (NewADValue-AD_MIN)/AD2RPM;			
		
	}
	
	#ifdef printTargetRPM
	printf("\n\rTargetRPM: %d\n\r", TargetRPM);
	#endif

//Set DutyUpdateTimer to DutyFetchPeriod and start timer
ES_Timer_InitTimer(TargetSpeedUpdateTimer, SpeedFetchPeriod);


  return ReturnEvent;
}



/****************************************************************************
 Function
    GetTargetRPM

 Parameters
   nothing

 Returns
   uint16_t Target speed

 Description
		simple public getter function to allow for reading in Target RPM

 Notes
   
 Author
   Drew Bell, 01/25/12, 15:23
****************************************************************************/

uint16_t GetTargetRPM ( void ) {
	return TargetRPM;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/****************************************************************************
 Function
    int_clamp

 Parameters
   value val, lower threshold ClampLow, upper threshold ClampHigh 

 Returns
   clamped value

 Description
   A simple utility function to clamp value between to limits
 Notes
   
 Author
   Drew Bell, 01/31/17, 15:23
****************************************************************************/

uint8_t int_clamp( uint8_t val, uint8_t ClampLow, uint8_t ClampHigh) {

if(val > ClampHigh)    //if val is too high
	return ClampHigh;     //return ClampHigh
if(val < ClampLow)
	return ClampLow;      //if val is too low
return val;             //otherwise return val because val is just right

}



/****************************************************************************
 Function
    int16_clamp

 Parameters
   value val, lower threshold ClampLow, upper threshold ClampHigh 

 Returns
   clamped value

 Description
   A simple utility function to clamp value between to limits for uint16_t
 Notes
   
 Author
   Drew Bell, 01/31/17, 15:23
****************************************************************************/

uint16_t int16_clamp( uint16_t val, uint16_t ClampLow, uint16_t ClampHigh) {

if(val > ClampHigh)    //if val is too high
	return ClampHigh;     //return ClampHigh
if(val < ClampLow)
	return ClampLow;      //if val is too low
return val;             //otherwise return val because val is just right

}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

