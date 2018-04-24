/****************************************************************************
 Module
   ButtonDebounce.c

 Revision
   1.0.1 

 Description
   This module handles the button event checking and debouncing

 Notes
   Note the use of static variables in sample event checker to detect
   ONLY transitions.
   
 History
 When           Who     What/Why
 -------------- ---     --------
 01/015/17 13:36 afb     initial entry of pseudocode and coding

****************************************************************************/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/

#include "ButtonDebounce.h"

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
#include "ES_Types.h"
#include "ES_Timers.h"
#include "MotorService.h"

// readability defines

#define SCLK_HI BIT1HI

//#define TEST_HARNESS
//#define test
//#define testDB

// this will pull in the symbolic definitions for events, which we will want
// to post in response to detecting events
#include "ES_Configure.h"
// this will get us the structure definition for events, which we will need
// in order to post events in response to detecting events
#include "ES_Events.h"
// if you want to use distribution lists then you need those function 
// definitions too.
#include "ES_PostList.h"
// This include will pull in all of the headers from the service modules
// providing the prototypes for all of the post functions
#include "ES_ServiceHeaders.h"
// this test harness for the framework references the serial routines that
// are defined in ES_Port.c
#include "ES_Port.h"
// include our own prototypes to insure consistency between header & 
// actual functionsdefinition
#include "EventCheckers.h"

#include "ES_Framework.h"
#include "ES_DeferRecall.h"

/*----------------------------- Module Defines ----------------------------*/
#define DATA GPIO_PIN_0

#define SCLK GPIO_PIN_1
#define SCLK_HI BIT1HI
#define SCLK_LO BIT1LO

#define RCLK GPIO_PIN_2
#define RCLK_LO BIT2LO
#define RCLK_HI BIT2HI

#define ModeState 0
#define DirState 1
#define BUTTON_UP 1
#define BUTTON_DOWN 0

#define ALL_BITS (0xff<<2)



#define DBDelay 60	//60ms debounce delay

//print defines
#define testDB


/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/


/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static ButtonState_t CurrentState;
static uint8_t LastButtonState[2] = {1,1};

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     InitializeButtonDebounce

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any 
     other required initialization for this service
 Notes

 Author
     Drew Bell, 01/15/17, 10:00
****************************************************************************/
bool InitializeButtonDebounce ( uint8_t Priority )
{
	printf("\n\rInit ButtonDB...");
	
  ES_Event ThisEvent;
  //Initialize the MyPriority variable with the passed in parameter.
  MyPriority = Priority;

  // SET BIT 1 TO ENABLE PORT F Clock and wait until peripheral reports that its clock is ready
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R5; 
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R5) != SYSCTL_PRGPIO_R5) 
    ;

  // Initialize the port line to monitor both button on PF4
      //Write to the digital enable register to connect pins 0 & 4 to digital I/O ports
      HWREG(GPIO_PORTF_BASE+GPIO_O_DEN) |= (GPIO_PIN_0 | GPIO_PIN_4);

      //Set PF0 and PF4 to be input
      HWREG(GPIO_PORTF_BASE+GPIO_O_DIR) &= ~(GPIO_PIN_0 | GPIO_PIN_4);
			
			//unlock PF0 pull-up by writing 0x4C4F434B to te lock register and 1 to the 0 bit in commit register
			HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) =0x4C4F434B;
			HWREG(GPIO_PORTF_BASE+GPIO_O_CR) |= BIT0HI;

      //Write to enable pullup resistors 
      HWREG(GPIO_PORTF_BASE+GPIO_O_PUR) |= (BIT0HI | BIT4HI);

  // Sample the button port pins and use it to initialize LastButtonState, using bit 0 as to monitor mode and bit 4 to monitor direction
  LastButtonState[ModeState] = HWREG(GPIO_PORTF_BASE+(GPIO_O_DATA + ALL_BITS)) & (GPIO_PIN_0);
	LastButtonState[DirState] = HWREG(GPIO_PORTF_BASE+(GPIO_O_DATA + ALL_BITS)) & (GPIO_PIN_4);
	//printf("\n\rLast Button State = %d\n\r",LastButtonState);

  // Set CurrentState to be DEBOUNCING_INIT
  CurrentState = DEBOUNCING_INIT;

  // Start debounce timer (timer posts to ButtonDebounceSM)
  ES_Timer_InitTimer(ButtonDebounceTimer, DBDelay);
	
	printf("InitButtonDB Done");

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
     PostButtonDebounce

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machines queue
 Notes

 Author
     Drew Bell, 1/15/17, 19:25
****************************************************************************/
bool PostButtonDebounce( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}


/****************************************************************************
 Function
    RunButtonDebounceSM

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error, ES_ERROR otherwise

 Description
   implements a 5-state state machine for debounce timing
 Notes
   
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event RunButtonDebounceSM( ES_Event ThisEvent )
{
ES_Event ReturnEvent;
ReturnEvent.EventType = ES_NO_EVENT; // assume no errors


// Switch between various states:
switch (CurrentState) {
  // State is DEBOUNCING_INIT
  case DEBOUNCING_INIT :
    // If ThisEvent is ES_TIMEOUT and EventType is Init Timer
    if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == ButtonDebounceTimer)) {
      // Set CurrentState to Ready2Sample
      CurrentState = Ready2Sample;
    }
    break;

  // State is Ready2Sample
  case Ready2Sample :
    // If EventType is ES_ButtonUp_Mode
    if(ThisEvent.EventType == ES_ButtonUp_Mode) {
      // Set CurrentState to DEBOUNCING_MODE
      CurrentState = DEBOUNCING_MODE;
      // Start Mode Debounce Timer
      ES_Timer_InitTimer(ModeDebounceTimer, DBDelay);
      // Post ES_DBButtonUp_Mode to MotorService
      ES_Event NewEvent;
      NewEvent.EventType = (ES_DBButtonUp_Mode);
			PostMotorService(NewEvent);
			#ifdef testDB
			printf("\n\rReady2Sample: Mode Up\n\r");
			#endif
      
    }
   // Else if EventType is ES_ButtonDown_Mode
    else if(ThisEvent.EventType == ES_ButtonDown_Mode) {
      // Set CurrentState to DEBOUNCING_MODE
      CurrentState = DEBOUNCING_MODE;
      // Start Mode Debounce Timer
      ES_Timer_InitTimer(ModeDebounceTimer, DBDelay);
      // Post ES_DBButtonDown_Mode to MotorService 
      ES_Event NewEvent;
      NewEvent.EventType = (ES_DBButtonDown_Mode);
			PostMotorService(NewEvent);
			#ifdef testDB
			printf("\n\rReady2Sample: Mode Down\n\r");
			#endif
    }
    // Else if EventType is ES_ButtonUp_Dir
      else if(ThisEvent.EventType == ES_ButtonUp_Dir) {
      // Set CurrentState to DEBOUNCING_DIR
      CurrentState = DEBOUNCING_DIR;
      // Start Dir Debounce Timer
      ES_Timer_InitTimer(DirDebounceTimer, DBDelay);
				
      // (need to?) Post ES_DBButtonUp_Dir to MotorService
      //ES_Event NewEvent;
      //NewEvent.EventType = (ES_DBButtonUp_Dir);
      //PostMotorService(NewEvent);
				
			#ifdef testDB
			printf("\n\rReady2Sample: Dir Up\n\r");
			#endif
    }
    // Else if EventType is ES_ButtonDown_Dir
    else if(ThisEvent.EventType == ES_ButtonDown_Dir) {
      // Set CurrentState to DEBOUNCING_DIR
      CurrentState = DEBOUNCING_DIR;
      // Start Dir Debounce Timer
      ES_Timer_InitTimer(DirDebounceTimer, DBDelay);
      // Post ES_DBButtonDown_Dir to MotorService
      ES_Event NewEvent;
      NewEvent.EventType = (ES_DBButtonDown_Dir);
      PostMotorService(NewEvent);
			#ifdef testDB
			printf("\n\rReady2Sample: Dir Down\n\r");
			#endif
    }
    break;

  // State is DEBOUNCING_DIR
  case DEBOUNCING_DIR :
    // If EventType is ES_TIMEOUT & parameter is Dir Debounce Timer
    if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DirDebounceTimer)) {
      // Set CurrentState to Ready2Sample
      CurrentState = Ready2Sample;
    }
    // Else if EventType is ES_ButtonUp_Mode
    else if(ThisEvent.EventType == ES_ButtonUp_Mode) {
      // Set CurrentState to DEBOUNCING_BOTH
      CurrentState = DEBOUNCING_BOTH;
      // Start Mode Debounce Timer
      ES_Timer_InitTimer(ModeDebounceTimer, DBDelay);
			
      //(Need to Post?) Post ES_DBButtonUp_Mode to MotorService
      //ES_Event NewEvent;
      //NewEvent.EventType = (ES_DBButtonUp_Mode);
      //PostMotorService(NewEvent);
			
			#ifdef testDB
			printf("\n\rDebouncingDir: Mode Up\n\r");
			#endif
    }
    // Else if EventType is ES_ButtonDown_Mode
    else if(ThisEvent.EventType == ES_ButtonDown_Mode) {
      // Set CurrentState DEBOUNCING_BOTH
      CurrentState = DEBOUNCING_BOTH;
      // Start Mode Debounce Timer
      ES_Timer_InitTimer(ModeDebounceTimer, DBDelay);
      // Post ES_DBButtonDown_Mode to MotorService
      ES_Event NewEvent;
      NewEvent.EventType = (ES_DBButtonDown_Mode);
			PostMotorService(NewEvent);
			#ifdef testDB
			printf("\n\rDebouncingDir: Mode Down\n\r");
			#endif
    }
    break;

  // State is DEBOUNCING_MODE
  case DEBOUNCING_MODE :
    // If EventType is ES_TIMEOUT & parameter is Mode Debounce Timer
    if((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == ModeDebounceTimer)) {
      // Set CurrentState to Ready2Sample
      CurrentState = Ready2Sample;
    }
    // Else if EventType is ES_ButtonUp_Dir
    else if(ThisEvent.EventType == ES_ButtonUp_Dir) {
      // Set CurrentState to DEBOUNCING_BOTH
      CurrentState = DEBOUNCING_BOTH;
      // Start Dir Debounce Timer
      ES_Timer_InitTimer(DirDebounceTimer, DBDelay);
      // (Need to Post?) Post ES_DBButtonUp_Dir to MotorService
      ES_Event NewEvent;
      NewEvent.EventType = (ES_DBButtonUp_Dir);
      PostMotorService(NewEvent);
			#ifdef testDB
			printf("\n\rDebouncingMode: Dir Up\n\r");
			#endif
    }
    // Else if EventType is ES_ButtonDown_Dir
    else if(ThisEvent.EventType == ES_ButtonDown_Dir) {
      // Set CurrentState to DEBOUNCING_BOTH
      CurrentState = DEBOUNCING_BOTH;
      // Start Dir Debounce Timer
      ES_Timer_InitTimer(DirDebounceTimer, DBDelay);
      // Post ES_DBButtonDown_Dir to MotorService
      ES_Event NewEvent;
      NewEvent.EventType = (ES_DBButtonDown_Dir);
      PostMotorService(NewEvent);
			#ifdef testDB
			printf("\n\rDebouncingDiMode: Dir Down\n\r");
			#endif
    }
    break;

  // State is DEBOUNCING_BOTH
  case DEBOUNCING_BOTH :
    // If EventType is ES_TIMEOUT & parameter is Dir Debounce Timer
    if((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DirDebounceTimer)) {
      // Set CurrentState to DEBOUNCING_MODE
      CurrentState = DEBOUNCING_MODE;
    }
    // Else if EventType is ES_TIMEOUT & parameter is Mode Debounce Timer
    else if((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == ModeDebounceTimer)) {
       // Set CurrentState to DEBOUNCING_DIR
      CurrentState = DEBOUNCING_DIR;
    }
    break;
  }

  //Return ES_NO_EVENT 
  //End of RunButtonDebounceSM
  return ReturnEvent;
}



/****************************************************************************
 Function
   CheckButtonEvents
 Parameters
   None
 Returns
   bool: true if a new key was detected & posted
 Description
   checks to see if a new button event occurs from PF0 and PF4 and, if so, 
   retrieves posts an ES_NewKey event to itself
 Notes

 Author
   Drew Bell, 01/15/17, 13:48
****************************************************************************/

bool CheckButtonEvents(void)
{
uint8_t ReturnVal = false;
uint8_t CurrentButtonState[2] = {0,0}; 

// Set CurrentButtonState to states read from port pins
CurrentButtonState[ModeState] = HWREG(GPIO_PORTF_BASE+(GPIO_O_DATA + ALL_BITS)) & (GPIO_PIN_0);
CurrentButtonState[DirState] = HWREG(GPIO_PORTF_BASE+(GPIO_O_DATA + ALL_BITS)) & (GPIO_PIN_4);

// If bit0 of CurrentButtonState has changed from bit0 of LastButtonState
if (CurrentButtonState[ModeState] != LastButtonState[ModeState]) {
  //set return value to be true
  ReturnVal = true;
  // If bit0 of CurrentButtonState is high (button up)
  if (CurrentButtonState[ModeState]) {
    // PostEvent ButtonUp_Mode to ButtonDebounce queue
    ES_Event ThisEvent;
    ThisEvent.EventType = ES_ButtonUp_Mode;
    PostButtonDebounce(ThisEvent);
    // Set bit0 of LastButtonState to the CurrentButtonState
    LastButtonState[ModeState] = CurrentButtonState[ModeState];
    
    #ifdef test
    printf("Mode Released");
    #endif
  }
  // Else (Bit0 of CurrentButtonState is low) (button down)
  else if ((CurrentButtonState[ModeState]) == BUTTON_DOWN) {
    // PostEvent ButtonDown_Mode to ButtonDebounce queue
    ES_Event ThisEvent;
    ThisEvent.EventType = ES_ButtonDown_Mode;
    PostButtonDebounce(ThisEvent);
    // Set bit0 of LastButtonState to the CurrentButtonState
    LastButtonState[ModeState] = CurrentButtonState[ModeState];

    #ifdef test
    printf("Mode Pressed");
    #endif
  }
}

// If bit4 of CurrentButtonState has changed from bit4 of LastButtonState
  if (CurrentButtonState[DirState] != LastButtonState[DirState]) {
    //set return value to be true
    ReturnVal = true;
    // If bit4 of CurrentButtonState is high (button up)
    if (CurrentButtonState[DirState] ) {
      // PostEvent ButtonUp_Dir to ButtonDebounce queue
      ES_Event ThisEvent;
      ThisEvent.EventType = ES_ButtonUp_Dir;
      PostButtonDebounce(ThisEvent);  
      // Set bit0 of LastButtonState to the CurrentButtonState
      LastButtonState[DirState] = CurrentButtonState[DirState];

      #ifdef test
      printf("Dir Released");
      #endif
    }
  // Else (Bit4 of CurrentButtonState is low) (button down)
    else if(CurrentButtonState[DirState] == BUTTON_DOWN) {
      // PostEvent ButtonDown_Dir to ButtonDebounce queue
      ES_Event ThisEvent;
      ThisEvent.EventType = ES_ButtonDown_Dir;
      PostButtonDebounce(ThisEvent);
      LastButtonState[DirState] = CurrentButtonState[DirState];

      #ifdef test
      printf("Dir Pressed");
      #endif
    }
  }

  //Return ReturnVal
  return ReturnVal;
  //End of CheckButtonEvents
}
  

/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
