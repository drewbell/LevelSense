/****************************************************************************
 Module
   ADService.c

 Revision
   1.0.1

 Description
   This is Gen2 Events and Services Framework module to read an analog
   signal and saves it to be accessed by another service.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/15/17 14:58 afb      Add in pseudocode and starting to code
 01/23/17	20:26 afb			 Adapting code for lab 6

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ADService.h"
#include "ADMulti.h"


/*----------------------------- Module Defines ----------------------------*/
#define OneChannel  1 
#define PE0_Reading 0
#define PrintInverval 100
#define ADTIMER_VAL 100

//#define test1print
//#define printADReading
//#define fastPrint


/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static uint16_t ADReading;
static uint32_t results[OneChannel];

#ifdef printADReading
static uint16_t PrintInc = 0;
#endif

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitializeADService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any 
     other required initialization for this service
 Notes

 Author
     Drew Bell, 01/23/17, 10:00
****************************************************************************/
bool InitializeADService ( uint8_t Priority )
{
	printf("\n\rInit ADService...");
		
  // Create ES_Event called ThisEvent;
  ES_Event ThisEvent;
  // Initialize the MyPriority variable with the passed in parameter.
  MyPriority = Priority;

  // Initialize the A/D system by calling ADC_MultiInit and pass 1 to enable PE0 to be A/D pin
  ADC_MultiInit(OneChannel);

  // Start ADTimer at 100ms to trigger speed updates at 10Hz
  ES_Timer_InitTimer(ADTimer, ADTIMER_VAL);
	
	printf("Init ADService Done!");

// Return True to end init

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
     PostADService

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this run function's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostADService( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunADService

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   When the ADTimer expires, read the PortE0 value and save it to a module-level variable.
 Notes
   
 Author
   Drew Bell, 01/12/17, 15:23
****************************************************************************/
ES_Event RunADService( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
// if EventType of ThisEvent is ES_TIMEOUT and Event Parameter is ADTimer
if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == ADTimer){
  // Use ADC read to get A/D reading from PE0
  ADC_MultiRead(results);
	
	ADReading = results[PE0_Reading];
	
	/*if fastPrint is defined, print every cycle*/
	#ifdef fastPrint
	printf("\n\rADReading = %d\n\r", ADReading);
	#endif
	
	/*if printADReading is defined, print every 100 cycles*/
	
	#ifdef printADReading
	if(PrintInc < PrintInverval) {
			PrintInc++;
	}
	else if(PrintInc >= PrintInverval){
		printf("\n\rADReading = %d\n\r", ADReading);
		PrintInc = 0;
	}
	#endif

  // Start ADTimer at 100ms to trigger speed updates at 10Hz
  ES_Timer_InitTimer(ADTimer, ADTIMER_VAL);

	}
  return ReturnEvent;
}

/****************************************************************************
 Function
     GetADReading

 Parameters
     none

 Returns
     uint16_t: 12 bit AD reading from port

 Description
     This is a public access funtion to allow for reading AD Reading

 Notes

 Author
     Drew Bell, 1/23/17, 19:25
****************************************************************************/
uint16_t GetADReading( void )
{
  return ADReading;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

