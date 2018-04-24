/*==========================================================================
 Name: MainService.c
 Author: Michal Rittikaidachar
 Version:1.0
 Date Created: 112/8/17
 Description: Main Service to control User input nodes
 Notes: 
 

  When           Who     What/Why
 -------------- ---     --------
 12/8/17				MR			Started Skeleton
 ==========================================================================*/
/*----------------------------- Include Files -----------------------------*/


// Headers FOR ES Framework & Services
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "SPIService.h"
#include "ES_ShortTimer.h"
#include "SteeringFeedback.h"

// Headers for TIVA's GPIO subsystem
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ssi.h"
#include "inc/hw_nvic.h"

// Headers to access the TivaWare Library
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"   // Define PART_TM4C123GH6PM in project
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "BITDEFS.H"
/*----------------------------- Module Defines ----------------------------*/
// these times assume a 1.000mS/tick timing
#define ONE_SEC 976
#define HALF_SEC (ONE_SEC/2)
#define TWO_SEC (ONE_SEC*2)
#define FIVE_SEC (ONE_SEC*5)
#define DAC_CONFIG_MASK 0x7000
#define DAC_DELAY 5

// Port D pin definitions
#define CLOCK_SSI GPIO_PIN_0
#define SLAVE_SELECT_0 GPIO_PIN_1
#define RX_SSI GPIO_PIN_2


//Port F pin defenions
#define SLAVE_SELECT_1 GPIO_PIN_3
#define TX_SSI GPIO_PIN_1

// SPI definitions
#define CPSDVSR 200
#define SCR 0x13
#define BitsPerNibble 4
#define SLAVE_0 0
#define SLAVE_1 1
#define ThrottleLine SLAVE_0
#define SteeringLine SLAVE_1
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
void WriteData( uint16_t Data , uint8_t Line);
uint8_t ReadData(void);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
// add a deferral queue for up to 3 pending deferrals +1 to allow for ovehead
static ES_Event DeferralQueue[3+1];
static InputServiceState_t CurrentState = InitInput; 
static uint16_t ThrottleVal;
static uint16_t SteeringVal;
static uint16_t DataOut=0;

/*------------------------------ Module Code ------------------------------*/


/*===========================================================================
 Function: initInputMainService
 Parameters: uint8_t : the priorty of this service
 Returns: bool, false if error in initialization, true otherwise
 Description:  Saves away the priority, and does any other required 
							 initialization for this service
 Author: Michal Rittikaidachar 10/25/17
 Notes: Code adapted from and based off of skeleton files by Ed Carryer
===========================================================================*/
bool InitSPIService ( uint8_t Priority )
{
  ES_Event ThisEvent;
  
  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
	// initialize deferral queue for testing Deferal function
  ES_InitDeferralQueueWith( DeferralQueue, ARRAY_SIZE(DeferralQueue) );
	// initialize LED drive for testing/debug output
	
  // initialize the Short timer system for channel A
  ES_ShortTimerInit(MyPriority, SHORT_TIMER_UNUSED);

  ES_Timer_InitTimer(DAC_TIMER, DAC_DELAY);

  
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

/*
 ============================================================
 Function: PostInputMainService
 Parameters: EF_Event ThisEvent ,the event to post to the queue
 Returns: bool false if the Enqueue operation failed, true otherwise
 Description: Posts an event to this state machine's queue
 Author: Michal Rittikaidachar 10/25/17
 Notes: Code adapted from and based off of skeleton files by Ed Carryer
 ============================================================
*/
bool PostSPIService( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/*===========================================================================
 Function: RunInputMainService
 Parameters: ES_Event : the event to process
 Returns: ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise
 Description: Runs State Machine for Controller Main Service.  
 Author: Michal Rittikaidachar 10/25/17
 Notes: Code adapted from and based off of skeleton files by Ed Carryer
===========================================================================*/
ES_Event RunSPIService( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
	
  
  switch (CurrentState){
      
    case InitInput :
        if ( ThisEvent.EventType == ES_INIT ){
            #ifdef PRINTING_CONTROLLER_MAIN
                    printf("\rCM: Initializing ControllerMainService State Machine\n");
            #endif
        
            //TODO Do any initialization here
        
            // switch states to selectingVehicle
            CurrentState =  Waiting;	
            
            #ifdef PRINTING_SPI
                printf("\rCM: Switched State to Waiting\n\r");
            #endif
	
		}

    break;
    
    case Waiting :  
        if ( ThisEvent.EventType == ES_TIMEOUT ){
             #ifdef PRINTING_SPI
                    printf("\rCM: Received Begin Xmit Event\n");
            #endif
            ThrottleVal=get12BitThrottleCmd();
            SteeringVal=get12BitSteeringCmd();
					  
					  #ifdef PRINT_SPI_STEER
					  printf("\n\r%i", SteeringVal);
					  #endif
					
            DataOut= (DAC_CONFIG_MASK | ThrottleVal);
            
            // Write Throttle Value Out
            WriteData(DataOut, ThrottleLine);
           
            // switch states to Xmiting
            CurrentState =  XmittingThrottle;	
        
            #ifdef PRINTING_SPI
                printf("\rCM: Switched State to Xmiting\n\r");
            #endif
        }            
        else if ( ThisEvent.EventType == ES_BEGIN_XMITTING ){
            #ifdef PRINTING_SPI
                    printf("\rCM: Received Begin Xmit Event\n");
            #endif
            
            ThrottleVal=get12BitThrottleCmd();
            SteeringVal=get12BitSteeringCmd();
            DataOut= (DAC_CONFIG_MASK | ThrottleVal);
            
            // Write Throttle Value Out
            WriteData(DataOut, ThrottleLine);
            
            // switch states to Xmiting
            CurrentState =  XmittingThrottle;	
        
            #ifdef PRINTING_SPI
                printf("\rCM: Switched State to Xmiting\n\r");
            #endif
	
		}
        
    break;
        
    case XmittingThrottle:  
        if ( ThisEvent.EventType == ES_EOT ){
            #ifdef PRINTING_SPI
                    printf("\rCM: RECEIVED EOT While transmitting throttle\n");
            #endif
        
            DataOut= (DAC_CONFIG_MASK | SteeringVal);
            WriteData(DataOut, SteeringLine);
            // switch states to xmittingsteering
            CurrentState =  XmittingSteering;	
        
            #ifdef PRINTING_SPI
                printf("\rCM: Switched State to Xmitting Steering\n\r");
            #endif
	
		}	
    break;
        
    case XmittingSteering:  
        if ( ThisEvent.EventType == ES_EOT ){
            #ifdef PRINTING_SPI
                    printf("\rCM: RECEIVED EOT while Xmitting Steering \n");
            #endif
        
            // Restart timer
             ES_Timer_InitTimer(DAC_TIMER, DAC_DELAY);
            
            // switch states to Waitning
            CurrentState =  Waiting;	
        
            #ifdef PRINTING_SPI
                printf("\rCM: Switched State to Waiting\n\r");
            #endif
	
		}	
    break;
    default :
        
    break;
  }
  return ReturnEvent;
}

/*===========================================================================
 Function: EOT_ISR
 Parameters: None
 Returns: None
 Description: Interrupt Response for SPI  
 Author: Michal Rittikaidachar 12/9/17
 Notes: Code adapted from and based off of skeleton files by Ed Carryer
===========================================================================*/
void EOT_ISR( void ) {
	// Mask the EOT interrupt
	HWREG(SSI1_BASE+SSI_O_IM) &=  ~SSI_IM_TXIM;
	HWREG(GPIO_PORTD_BASE+GPIO_O_DEN) &= ~SLAVE_SELECT_0;
    HWREG(GPIO_PORTF_BASE+GPIO_O_DEN) &= ~SLAVE_SELECT_1;
	// Post EOT event to InputMainService
	ES_Event ThisEvent;
	ThisEvent.EventType = ES_EOT;
	PostSPIService(ThisEvent);
}



/***************************************************************************
 private functions
 ***************************************************************************/

/***************************************************************************
 private functions
 ***************************************************************************/
void InitSPI( void ) {
    #ifdef PRINTING_SPI
        printf("\rSPI.C: Initilizing SPI \n\r");
    #endif    
    
	    // Enable clock to GPIO Port D and F
	HWREG(SYSCTL_RCGCGPIO) |= (SYSCTL_RCGCGPIO_R3 | SYSCTL_RCGCGPIO_R5);
    
	// Enable the clock to the SSI module 1
	HWREG(SYSCTL_RCGCSSI) |= SYSCTL_RCGCSSI_R1;
	
	// Wait for the SSI module to be ready
	while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R5) != SYSCTL_PRGPIO_R5);

	// Program the GPIO Port D to use alternate functions for SSI pins (PD:0/1/2)
    HWREG(GPIO_PORTD_BASE+GPIO_O_AFSEL) |= (CLOCK_SSI | SLAVE_SELECT_0 | RX_SSI );

    // Program the GPIO Port F to use alternate functions for SSI pins (PF3)
    HWREG(GPIO_PORTF_BASE+GPIO_O_AFSEL) |= (SLAVE_SELECT_1 | TX_SSI);
    
	// Map SSI to those pins. This is a mux value of 2 that we
	//want to use for specifying the function on bits 2, 3, 4, & 5
    //Port D
    HWREG(GPIO_PORTD_BASE+GPIO_O_PCTL) = 
    (HWREG(GPIO_PORTD_BASE+GPIO_O_PCTL) & 0xff0000ff) + (2<<(0*BitsPerNibble)) +
      (2<<(1*BitsPerNibble)) + (2<<(2*BitsPerNibble)) + (2<<(3*BitsPerNibble));
    HWREG(GPIO_PORTF_BASE+GPIO_O_PCTL) = 
    (HWREG(GPIO_PORTF_BASE+GPIO_O_PCTL) & 0xff0000ff) + (2<<(1*BitsPerNibble)) +
    (2<<(3*BitsPerNibble));
      
	// Enable PD:0,2,3 and PF3 as digital pins
	HWREG(GPIO_PORTD_BASE+GPIO_O_DEN) |= (CLOCK_SSI |  RX_SSI );
    HWREG(GPIO_PORTD_BASE+GPIO_O_DEN) |= SLAVE_SELECT_0;
    HWREG(GPIO_PORTF_BASE+GPIO_O_DEN) |= (SLAVE_SELECT_1 | TX_SSI);
    
	// Enable PD:0,1,3 and PF as outputs
	HWREG(GPIO_PORTD_BASE+GPIO_O_DIR) |= (CLOCK_SSI | SLAVE_SELECT_0 );
	HWREG(GPIO_PORTF_BASE+GPIO_O_DIR) |= (SLAVE_SELECT_1 | TX_SSI);
    
	// Enable PD2 as an input
	HWREG(GPIO_PORTD_BASE+GPIO_O_DIR) &= ~(RX_SSI);
	
	// Enable pull up resitor for clock line (PD0)
	HWREG(GPIO_PORTD_BASE+GPIO_O_PUR) |= CLOCK_SSI;
	
	// Wait for the SSI1 module to be ready
	while ((HWREG(SYSCTL_PRSSI) & SYSCTL_PRSSI_R1) != SYSCTL_PRSSI_R1);
	
	// Disable SSI before programming mode bits
	HWREG(SSI1_BASE+SSI_O_CR1) &= ~SSI_CR1_SSE;
	
	// Select Master mode (MS) and TXIS indicating end of transmit
	HWREG(SSI1_BASE+SSI_O_CR1) &= ~SSI_CR1_MS;
	HWREG(SSI1_BASE+SSI_O_CR1) |= SSI_CR1_EOT;
	
	// Configure SSI clock source to the system clock
	HWREG(SSI1_BASE+SSI_O_CC) &= SSI_CC_CS_SYSPLL;  //~(BIT0HI|BIT1HI|BIT2HI|BIT3HI); 
	
	// Configure clock prescaler
	HWREG(SSI1_BASE+SSI_O_CPSR) |= CPSDVSR;
	//HWREG(SSI0_BASE+SSI_O_CPSR) = (HWREG(SSI0_BASE+SSI_O_CPSR) & 0xffffff00)|CPSDVSR;
	
	// Configure clock rate (SCR), phase and polarity (SPH, SPO), mode (FRF), data size (DSS)
	HWREG(SSI1_BASE+SSI_O_CR0) = (HWREG(SSI1_BASE+SSI_O_CR0) & 0xffff0000) | (SCR<<8) | (1<<7) | (1<<6) | (0xf);
	
	// Locally enable interrupts(TXIM in SSIIM) 
	HWREG(SSI1_BASE+SSI_O_IM) |= SSI_IM_TXIM;   // Unmask TXIM FIFO interrupt
	
	// Enable SSI module 
	HWREG(SSI1_BASE+SSI_O_CR1) |= SSI_CR1_SSE;
	
	// Globally enable interrupts
	__enable_irq();
	
	// Enable NVIC interrupt for SSI when starting to transmit
	HWREG(NVIC_EN1) |= BIT2HI;
    
    #ifdef PRINTING_SPI
        printf("\rSPI.C: Finished Initilizing SPI \n\r");
    #endif  
}

void WriteData( uint16_t Data, uint8_t Line ) {

    switch (Line){
      
        case SLAVE_0 :
            HWREG(GPIO_PORTD_BASE+GPIO_O_DEN) |= SLAVE_SELECT_0;
        break; 

        case SLAVE_1:
            HWREG(GPIO_PORTF_BASE+GPIO_O_DEN) |= SLAVE_SELECT_1;
        break;
        
        default:
            
        break;
    
    }
     // Unmask the EOT interrupt
    HWREG(SSI1_BASE+SSI_O_IM) |= SSI_IM_TXIM;
 
    // Write data to register
    HWREG(SSI1_BASE+SSI_O_DR) = Data;
    
}

uint8_t ReadData( void ) {
	// Read from data register
	return HWREG(SSI1_BASE+SSI_O_DR);
}




/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

