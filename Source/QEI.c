/****************************************************************************
 Module
   QEI.c

 Revision
   1.0.1

 Description
   This is service sets up and checks a the state of a motor using the TivaWare
   Quadrature Encoder Interface (QEI) module. 

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 12/04/17 09:58 dfb     started implementation of module
 
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "QEI.h"
#include <stdint.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/qei.h"
#include "constants.h"

/*----------------------------- Module Defines ----------------------------*/

 //#define printQEI0
 //#define printQEI1


/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitQEI

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any 
     other required initialization for this service
 Notes

 Author
     Drew Bell, 12/04/17, 10:00
****************************************************************************/
bool InitQEI ( uint8_t Priority )
{
  ES_Event ThisEvent;

  MyPriority = Priority;
    
    printf("\n\rInit QEI...\n\n\r");
    
    // print steering configuration
    printf("One motor rotation = %d ticks\n\r", E0_TICKS_PER_REV_OUT);
    printf("Steering Range (full left to full right) in turns: %.1f\n\r", 
        STEERING_RANGE);
    printf("   Ticks: MIN = %d,  MIDDLE = %d,  MAX = %d\n\r", 0, ENCODER_0_CENTER, ENCODER_0_RANGE);
    
    // print throttle configuration
    printf("Throttle ticks: \n\r   MIN %i \n\r   MIDDLE %i\n\r   MAX %i \n\r   RANGE %i \n\r", 
        THROTTLE_MIN_TICK, THROTTLE_MIDDLE, THROTTLE_MAX_TICK, THROTTLE_TICK_RANGE);
    
    
    
    // Initialize QEI0 on PD6:7 and QEI1 on PC5:6
    
    // Enable GPIO PortD and wait til it is ready
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
	{
	}
    
    // Enable GPIO PortC and wait til it is ready
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
	{
	}
    
    // Enable the QEI0 peripheral and wait for the QEI0 module to be ready.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0))
    {
    }
    
    // Enable the QEI1 peripheral and wait for the QEI1 module to be ready.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI1))
    {
    }
    
    //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; 
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
    
    // configure the alternate function of GPIO pins
    GPIOPinConfigure(GPIO_PD3_IDX0);    // PD3 is index for QEI0
    GPIOPinConfigure(GPIO_PD6_PHA0);    // PD6 is PhA0 for QEI0
    GPIOPinConfigure(GPIO_PD7_PHB0);    // PD7 is PhB0 for QEI0
    
    GPIOPinConfigure(GPIO_PC4_IDX1);    // PC4 is index for QEI1
    GPIOPinConfigure(GPIO_PC5_PHA1);    // PC5 is PhA1 for QEI1
    GPIOPinConfigure(GPIO_PC6_PHB1);    // PC6 is PhB1 for QEI1

    // configures pins to be used by QEI0 peripheral
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7);
    
    // configures pins to be used by QEI1 peripheral
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);

    //
    // Configure the quadrature encoder to capture edges on both signals and
    // maintain an absolute position by resetting on index pulses. Using a
    // 12 line encoder at four edges per line, there are 48 pulses per
    // revolution of the motor, which is geared up 9.68:1; 
    // therefore set the maximum position to 454 as the count
    // is zero based.
    //
    QEIDisable(QEI0_BASE);

    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_RESET_IDX |
        QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), ENCODER_0_RANGE);
        
        
    // configure QE1
        
    QEIDisable(QEI1_BASE);

    QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    
    QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_RESET_IDX |
        QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), ENCODER_1_RANGE);
    
    //
    // Enable the quadrature encoder.
    QEIEnable(QEI0_BASE);
    QEIEnable(QEI1_BASE);
    
    //Set position to a middle value so we can see if things are working
	QEIPositionSet(QEI0_BASE, ENCODER_0_RANGE/2);
    QEIPositionSet(QEI1_BASE, ENCODER_1_RANGE/2);

    printf("\n\rInit QEI Done.\n\r");
    
    ES_Timer_InitTimer(QEI_TIMER, ONE_SEC/2);
    
  
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
     PostQEI

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostQEI( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunQEI

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event RunQEI( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
    
    switch (ThisEvent.EventType){
        case ES_INIT :
            // do nothing
            break;
        
        case ES_TIMEOUT :  
            // if its the QEI timer, print out the current position and direction of chosen encoder, reset timer
            if(ThisEvent.EventParam == QEI_TIMER){
                
                #ifdef printQEI0
                printf("\n\rSteer: Pos0: %d, Dir0: %d", QEIPositionGet(QEI0_BASE), QEIDirectionGet(QEI0_BASE));
                #endif
                
                #ifdef printQEI1
                printf("\n\rThrottle: Pos1: %d, Dir1: %d", QEIPositionGet(QEI1_BASE), QEIDirectionGet(QEI1_BASE));
                #endif
                
                ES_Timer_InitTimer(QEI_TIMER, ONE_SEC/2);
            }
        
          break;
        
        case ES_NEW_KEY :  
            // if the event is an 'i' key, reset the index
            if(ThisEvent.EventParam == 'i'){
                // todo implememnt
            }       
            // else if event is 'p', print the position
            else if(ThisEvent.EventParam == 'p'){
                printf("\n\rPos: %d, Dir: %d", QEIPositionGet(QEI0_BASE), QEIDirectionGet(QEI0_BASE));
            }            
            break;
    }
    
  return ReturnEvent;
}

/****************************************************************************
 Function
    getEncoderPosition

 Parameters
   uint8_t module

 Returns
   uint16_t position

 Description
   Returns the encoder position in ticks
 Notes
   
 Author
   Drew Bell, 12/06/17, 15:23
****************************************************************************/
uint16_t getEncoderPosition(uint8_t qeiModule){
    if(qeiModule == 0){
        return QEIPositionGet(QEI0_BASE);
    }
    else if(qeiModule == 1){
        return QEIPositionGet(QEI1_BASE);
    }
    // else 
    return 0;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

