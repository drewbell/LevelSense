/****************************************************************************
 Module
   SpeedCounterService.c

 Revision
   1.0.1

 Description
   This is a service to manage encoder reading and LED readout under the 
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/26/17 09:58 afb      v1
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/

#include "SpeedCounterService.h"

// the common headers for C99 types 
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

// the headers to access the GPIO subsystem
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"									//THISGUY
#include "inc/hw_sysctl.h"
#include "inc/hw_timer.h"

#include "inc/hw_pwm.h"
#include "inc/hw_nvic.h"

// the headers to access the TivaWare Library
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BITDEFS.H"

//my modules used
#include "ShiftRegisterWrite.h"
#include "PWM.h"
#include "MotorService.h"

/*----------------------------- Module Defines ----------------------------*/


#define WT0CCP0_MUX 7 		//timer alternative function select register value
#define CtrlISRDebugLine

#define EightLEDs 255
#define SevenLEDs 137
#define SixLEDs 63
#define FiveLEDs 31
#define FourLEDs 15
#define ThreeLEDs 7
#define TwoLEDs 3
#define OneLED 1
#define MovingAvgSize 50

#define MinTickThreshold 16000  //number of ticks per interrupt when spun as fast as possible
#define ThresholdInterval 5000	//numer of ticks in each of the led bins
#define MaxTickThreshold 40000	//number of ticks per interrupt when spun as slowly as possible (with spin still reasonably smooth)
#define BarUnit (MaxTickThreshold - MinTickThreshold)/6

//RPM Conversions
#define ClockSpeed 40000000
#define SecPerMin 60
#define GearRotIn 59
#define GearRotOut 10
#define GEAR_RATIO 5.9f
#define PulsesPerRev 512
#define TICKperPULSEperREV ClockSpeed/PulsesPerRev                    //two step calc of Period 2 RPM conversion because it otherwise causes overflow
#define PER2RPM TICKperPULSEperREV*SecPerMin*GearRotOut/(GearRotIn)

#define ALL_BITS (0xff<<2)
#define HalfSecond 500
#define PrintInterval 350
#define SpartaPrintInterval 300

#define SetFlag 1
#define ClearFlag 0
#define ClearValue 0

#define TicksPerMS 40000/32
#define PriorityLevelOne 1
#define CTRL_TIME_PERIOD 2

#define	RPM_MIN	0.0f
#define RPM_MAX 55.0f
#define MINDUTY 0
#define MAXDUTY 100

#define SR_INITVAL 0x0

//printdefines
//#define printCtrlDebug
//#define Part2_3
//#define Part2_5
//#define Part2_6
//#define printPeriod

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

void InitInputCapturePeriod(void);
void InitControlPeriod (void);
float clamp( float val, float ClampLow, float ClampHigh);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

//input capture module vars
static uint32_t Period;
static uint32_t LastCapture;
static uint32_t Bars = 0;
static uint32_t PeriodLog[MovingAvgSize];
static uint32_t RPM = 0; 
static uint8_t RotateFlag = 0;
static float TargetRPM = 0.0; 
static uint8_t RequestedDuty = 0;

volatile static float debug_IntegralTerm = 0.0;      //integrator for control effort
volatile static float debug_RPMError = 0.0;                //make static for speed

// PID Gains
static float pGain = 1.2;
static float iGain = 0.5;
static float dGain = 0;

static uint16_t CtrlCount = 0;
static uint16_t JumpCount = 0;


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitializeSpeedCounter

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and initializes the input capture setup to read the encoder
 Notes

 Author
     Drew Bell, 01/26/17, 10:00
****************************************************************************/
bool InitializeSpeedCounter ( uint8_t Priority )
{
  ES_Event ThisEvent;

  MyPriority = Priority;
	
	//initialize Shift register
	SR_Init();
	SR_Write(SR_INITVAL);
	
  //Initialize output pin PB3 for use as a time measuring pulse
	
	// SET BIT 1 TO ENABLE PORT B Clock and wait until peripheral reports that its clock is ready
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1; 
	while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R1) != SYSCTL_PRGPIO_R1) 
		;
			
	//Write to the digital enable register to connect pin 3 to be digital I/O ports
	HWREG(GPIO_PORTB_BASE+GPIO_O_DEN) |= ( GPIO_PIN_3 );
	
	//Set PB3 to be output
	HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) |= ( GPIO_PIN_3 );
	
	//Initialize as low
	HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= BIT3LO;	
  
	//call function to init the input capture
	InitInputCapturePeriod();
	
	//call init function to init control period
	InitControlPeriod();

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
     PostSpeedCounter

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     Drew Bell, 01/26/11, 19:25
****************************************************************************/
bool PostSpeedCounter( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunSpeedCounter

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Simple run function to handle new edge event and relating calculations + bitbanging
 Notes
   
 Author   Drew Bell, 01/26/17, 15:23
****************************************************************************/
ES_Event RunSpeedCounter( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
	static uint8_t HistoryPosition = 0;
	static uint32_t SmoothedPeriod = 0;
	uint8_t inc;
	uint8_t ClearInc;
	uint32_t SumPeriod = 0;
	uint32_t PeriodPerSec;
	static uint16_t PrintInc = 0;
	volatile static uint8_t PrintFlag = 0;
	static uint8_t stepflag = 1;
	
	if((ThisEvent.EventType == ES_NEW_KEY) && (ThisEvent.EventParam == ' ')){
		if (stepflag){
			printf("\n\r ***********Step UP**********\n\r");
			SetDC_A(100);
			stepflag = 0; 
			
		}
		else if(stepflag == 0){
			printf("\n\r ***********Step DOWN**********\n\r");
			SetDC_A(0);
			stepflag = 1; 
		}
	}
			
	
	//conditional declarations to eliminate warnings
	#ifdef Part2_6	
	static uint16_t PrintIncPWM = 0;
	#endif 
	
	#ifdef printPeriod
	static uint16_t PrintInc = 0;
	#endif
	
	//increment a print counter that can be used by multiple 
	if(PrintInc < PrintInterval) {
			PrintInc = PrintInc + 1;
			PrintFlag = 0;
	}
	else if(PrintInc >= PrintInterval){
			PrintInc = 0;
			PrintFlag = 1;		//let everyone print if they want to
		}
	
	//printing important control numbers
	#ifdef printCtrlDebug
	if(PrintFlag) {
		printf("\n\rT_RPM: %0.2f	RPM: %d	RPMErr: %0.2f	R_Duty: %d	I_Term = %0.2f\n\r", TargetRPM, RPM, debug_RPMError, RequestedDuty, debug_IntegralTerm);
	}
	#endif
		
	//extra logic to respond to 0 RPM situation 
	if((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == RotationTimeout)) {
		
		RotateFlag = ClearFlag;
	
	//clear all of the variables in the RPM Calculation
		//clear moving average to not impact RPM on next ADTimer timeout
		for (ClearInc = 0; ClearInc < (MovingAvgSize - 1) ; ClearInc++){
				PeriodLog[ClearInc] = ClearValue;
			}
		//clear PeriodPerSec
			PeriodPerSec = ClearValue;
		//clear SumPeriod
			SumPeriod = ClearValue;
		//set RPM to zero
		RPM = 0;

	printf("\n\rROTATION TIMEOUT!! Setting RPM = %d\n\r", RPM);
}
	
//if EventType of ThisEvent is ES_NewEdge
	if(ThisEvent.EventType == ES_NewEdge) {
		
	//implement a moving average of the period with the number of elements in history set by MovingAvgSize
	//Put value of period in the HistoryPosition cell of the PeriodLog
		PeriodLog[HistoryPosition] = Period;
		//sum up periods in Period Log
		for (inc = 0; inc < (MovingAvgSize - 1) ; inc++){
			SumPeriod = SumPeriod + PeriodLog[inc];
		}
		//SmoothedPeriod equals sum of elements of PeriodLog divided by MovingAvgSize
		SmoothedPeriod = SumPeriod/MovingAvgSize;
		//to get ready for next cycle, if History Position is greater than or equal to (MovingAvgSize -1)
		if (HistoryPosition >= (MovingAvgSize - 1)) {
				//set HistoryPosition equal to zero
				HistoryPosition = 0;
		}
			else {
				//else add one to history position
				HistoryPosition = HistoryPosition+1;
		}
		//if Part2_3 is defined, raise and lower a line to determine how long it takes for the speed scale to happen
		#ifdef Part2_3 
			HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= BIT3HI;
		#endif

			if(SmoothedPeriod < MinTickThreshold){
				Bars = OneLED;
			}
			else if((SmoothedPeriod >= (MinTickThreshold)) && (SmoothedPeriod < (MinTickThreshold+1*ThresholdInterval) ) ){
				Bars = TwoLEDs;
			}
			else if((SmoothedPeriod >= (MinTickThreshold+1*ThresholdInterval)) && (SmoothedPeriod < (MinTickThreshold+2*ThresholdInterval))){
				Bars = ThreeLEDs;
			}
			else if((SmoothedPeriod >= (MinTickThreshold+2*ThresholdInterval)) && (SmoothedPeriod < (MinTickThreshold+3*ThresholdInterval))){
				Bars = FourLEDs;
			}
			else if((SmoothedPeriod >= (MinTickThreshold+3*ThresholdInterval)) && (SmoothedPeriod < (MinTickThreshold+4*ThresholdInterval))){
				Bars = FiveLEDs;
			}
			else if((SmoothedPeriod >= (MinTickThreshold+4*ThresholdInterval)) && (SmoothedPeriod < (MinTickThreshold+5*ThresholdInterval))){
				Bars = SixLEDs;
			}
			else if((SmoothedPeriod >= (MinTickThreshold+5*ThresholdInterval)) && (SmoothedPeriod < (MinTickThreshold+6*ThresholdInterval))){
				Bars = SevenLEDs;
			}
			else if((SmoothedPeriod >= (MinTickThreshold+6*ThresholdInterval)) ){
				Bars = EightLEDs;
			}
		
	SR_Write(Bars);
	
		#ifdef Part2_3 
			HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= BIT3LO;
		#endif	
	
		#ifdef printPeriod
			if(PrintFlag){
				printf("\n\rSmoothedPeriod = %d, Period = %d\n\r", SmoothedPeriod, Period);
			}
		#endif
	}

	//Use integer math to calculate scaled version of encoder period to LED value
	//write scaled version of period to LEDs via shift register
	// calculate RPM
		
	if((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == ADTimer)) {
		
		#ifdef Part2_5
			//if Part2_5 is defined, raise pin PB3 to show start of RPM calc
			//raise a line at the start of RPM calc
			HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= BIT3HI;
		#endif
		
		if(RotateFlag) {
		
		PeriodPerSec = ClockSpeed / SmoothedPeriod;
		RPM = PeriodPerSec*SecPerMin*GearRotOut/(PulsesPerRev*GearRotIn);
		}
	
		#ifdef Part2_5
			//if Part2_3 is defined, lower pin PB3 to show end of RPM calc
			//lower a line at the end of RPM calc
			HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= BIT3LO;
		#endif
		
	#ifdef Part2_6	
		//if Part2_6 is defined, raise pin PB3 to show start of printing to TeraTerm
		//raise a line at the start of the print
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= BIT3HI;
		
		if(PrintFlag){
				//print period to TeraTerm
			printf("\n\rRPM = %d\n\r",RPM);
		}

		//lower a line at the end of the print
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= BIT3LO;
	#endif
	}
	
  return ReturnEvent;
}

/****************************************************************************
 Function
    InitInputCapturePeriod

 Parameters
   None

 Returns
   None

 Description
   Sets up input capture interrupt
 Notes
   
 Author
   Drew Bell, 01/26/17, 15:23
****************************************************************************/

void InitInputCapturePeriod (void) {
	
printf("\n\rInit InputCptrePeriod...");

// start by enabling the clock to the timer (Wide Timer 0)
HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R0;
	
// enable the clock to Port C
HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R2;
	
// since we added this Port C clock init, we can immediately start
// into configuring the timer, no need for further delay
// make sure that timer (Timer A) is disabled before configuring
HWREG(WTIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
	
// set it up in 32bit wide (individual, not concatenated) mode
// the constant name derives from the 16/32 bit timer, but this is a 32/64
// bit timer so we are setting the 32bit mode
HWREG(WTIMER0_BASE+TIMER_O_CFG) = TIMER_CFG_16_BIT;
	
// we want to use the full 32 bit count, so initialize the Interval Load
// register to 0xffff.ffff (its default value :-)
HWREG(WTIMER0_BASE+TIMER_O_TAILR) = 0xffffffff;
	
// set up timer A in capture mode (TAMR=3, TAAMS = 0),
// for edge time (TACMR = 1) and up-counting (TACDIR = 1)
HWREG(WTIMER0_BASE+TIMER_O_TAMR) =
(HWREG(WTIMER0_BASE+TIMER_O_TAMR) & ~TIMER_TAMR_TAAMS) |
(TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP);

// To set the event to rising edge, we need to modify the TAEVENT bits
// in GPTMCTL. Rising edge = 00, so we clear the TAEVENT bits
HWREG(WTIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEVENT_M;

// Now Set up the port to do the capture (clock was enabled earlier)
// start by setting the alternate function for Port C bit 4 (WT0CCP0)
HWREG(GPIO_PORTC_BASE+GPIO_O_AFSEL) |= BIT4HI;

// Then, map bit 4's alternate function to WT0CCP0
// 7 is the mux value to select WT0CCP0, 16 to shift it over to the
// right nibble for bit 4 (4 bits/nibble * 4 bits)
HWREG(GPIO_PORTC_BASE+GPIO_O_PCTL) =
(HWREG(GPIO_PORTC_BASE+GPIO_O_PCTL) & 0xfff0ffff) + (WT0CCP0_MUX<<16); 

// Enable pin on Port C for digital I/O
HWREG(GPIO_PORTC_BASE+GPIO_O_DEN) |= BIT4HI;

// make pin 4 on Port C into an input
HWREG(GPIO_PORTC_BASE+GPIO_O_DIR) &= BIT4LO;

// back to the timer to enable a local capture interrupt
HWREG(WTIMER0_BASE+TIMER_O_IMR) |= TIMER_IMR_CAEIM;

// enable the Timer A in Wide Timer 0 interrupt in the NVIC
// it is interrupt number 94 so appears in EN2 at bit 30
HWREG(NVIC_EN2) |= BIT30HI;

// make sure interrupts are enabled globally
__enable_irq();

// now kick the timer off by enabling it and enabling the timer to
// stall while stopped by the debugger
HWREG(WTIMER0_BASE+TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL);

printf("Init InputCptrePeriod Done");
}

/****************************************************************************
 Function
    InputCaptureResponse

 Parameters
   None

 Returns
   None

 Description
   ISR response to input capture interrupt
 Notes
   
 Author
   Drew Bell, 01/26/17, 15:23
****************************************************************************/
void InputCaptureResponse( void ){
	
	//declare temp capture variable
	uint32_t ThisCapture;
	
	// start by clearing the source of the interrupt, the input capture event
	HWREG(WTIMER0_BASE+TIMER_O_ICR) = TIMER_ICR_CAECINT;
		
	// now grab the captured value and calculate the period
	ThisCapture = HWREG(WTIMER0_BASE+TIMER_O_TAR);
	Period = ThisCapture - LastCapture;
	// update LastCapture to prepare for the next edge
	LastCapture = ThisCapture;
	//post NewEdge Event to SpeedCounterService to update LED bar
	ES_Event ThisEvent;
	ThisEvent.EventType = (ES_NewEdge);
	PostSpeedCounter(ThisEvent);
	
	//set RotationTimeout timer
		ES_Timer_InitTimer(RotationTimeout, HalfSecond);
	//Set Rotating flag
		RotateFlag = SetFlag;
}


/****************************************************************************
 Function
    InitControlPeriod

 Parameters
   None

 Returns
   None

 Description
   Sets up a periodic timer for the control law interrupt response routine
 Notes
   
 Author
   Drew Bell, 01/31/17, 15:23
****************************************************************************/

void InitControlPeriod (void) {
	
	printf("\n\rInit CtrlPrd...");
	
//Start by enabling the clock to the timer (Wide Timer 0)
	HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R0;
//Wait until the clock get going
	while ((HWREG(SYSCTL_PRWTIMER) & SYSCTL_PRWTIMER_R0) != SYSCTL_PRWTIMER_R0) 
		;
//Make sure that timer (Timer B) is disabled before configuring
	HWREG(WTIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TBEN;
//Set it up in 32bit wide (individual, not concatenated) mode
	HWREG(WTIMER0_BASE+TIMER_O_CFG) = TIMER_CFG_16_BIT;
	
//Set up timer B in periodic mode so that it repeats the time-outs
	HWREG(WTIMER0_BASE+TIMER_O_TBMR) = (HWREG(WTIMER0_BASE+TIMER_O_TBMR)& ~TIMER_TBMR_TBMR_M)|
	TIMER_TBMR_TBMR_PERIOD;
//Set timeout to 2mS
	HWREG(WTIMER0_BASE+TIMER_O_TBILR) = TicksPerMS * CTRL_TIME_PERIOD;
//Enable a local timeout interrupt
	HWREG(WTIMER0_BASE+TIMER_O_IMR) |= TIMER_IMR_TBTOIM;
//Enable the Timer B in Wide Timer 0 interrupt in the NVIC it is interrupt number 95 so appears in EN2 at bit 31
	//HWREG(NVIC_EN2) = BIT31HI;
    
    // NVIC DISABLED ********************************
    
    //Set control priority lower than encoder priority by writing 1 to NVIC priority register 23 interrupt D
//This sets the interrupt priority to 1, whereas default is 0, the highest level. Priorities 0 to 7 are valid
	//HWREG(NVIC_PRI23) = (HWREG(NVIC_PRI23) & ~NVIC_PRI23_INTD_M) | (PriorityLevelOne << NVIC_PRI23_INTD_S);
//Make sure interrupts are enabled globally
	__enable_irq();
	
//Now kick the timer off by enabling it and enabling the timer to stall while stopped by the debugger
	//HWREG(WTIMER0_BASE+TIMER_O_CTL) |= (TIMER_CTL_TBEN | TIMER_CTL_TBSTALL);
	
		//printf("Init CtrlPrd Done!");
		//printf("\n\n\rCurrent Gains: kp = %.2f	ki = %.2f	kd = %.2f\n\r", pGain, iGain, dGain);
		//printf("\n\r**********************************************************\n\n\r");
	
	return;
}

/****************************************************************************
 Function
    ControlResponse

 Parameters
   None

 Returns
   None

 Description
   ISR response to the periodic control loop interrupt
 Notes
   
 Author
   Drew Bell, 01/31/17, 15:23
****************************************************************************/
void ControlResponse( void ){
	
	static float IntegralTerm = 0.0;      //integrator for control effort
	static float RPMError;                //make static for speed
	static float LastError = 0.0;         // for derivative control
	static uint32_t ThisPeriod = 10;           //make static for speed
	
	//Start by clearing the source of the interrupt
	HWREG(WTIMER0_BASE+TIMER_O_ICR) = TIMER_ICR_TBTOCINT;
	
	//increment a counter var
	CtrlCount++;
	
	//Allow for timing of this routine by raising PB3
	#ifdef CtrlISRDebugLine
	HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= BIT3HI;
	#endif

	//call getter function to get TargetPRM from motor service
	TargetRPM = GetTargetRPM();
	TargetRPM = clamp(TargetRPM, RPM_MIN, RPM_MAX);
	
	//Implement control law
	//ThisPeriod equals Period
	ThisPeriod = Period;
	
	//Calculate RPM by taking Period2RPM conversion divided by Period
	RPM = PER2RPM / ThisPeriod;
	
	//Find RPMError by taking TargetRPM minus RPM
	RPMError = TargetRPM - RPM;
	
	//IntegralTerm equals Integral Term plus RPMError
	IntegralTerm += (iGain*RPMError);
	
	//Anti-windup clamp on the iGain
	IntegralTerm = clamp(IntegralTerm , MINDUTY , MAXDUTY);
	
	//To be compatible with Zeigler Nichols Tuning, RequestedDuty equals Kp*(RPMError + (Ki * IntegralTerm)) + Kd*(RPMError – LastError) 
	RequestedDuty = pGain*(RPMError + IntegralTerm ) + dGain*(RPMError - LastError);
	
	//Requested duty is clamped between zero and 100
	RequestedDuty = clamp(RequestedDuty , MINDUTY , MAXDUTY);
	
	//LastError equals RPMError to update last error for next round
	LastError = RPMError;
	
	//SetDuty of PB6 as RequestedDuty
	if(JumpCount < 1000){
	//SetDC_A(RequestedDuty);
		SetDC_A(0);
	}
	else if (JumpCount >= 1000){
		SetDC_A(100);
	}
	
	debug_RPMError = RPMError;
	debug_IntegralTerm = IntegralTerm;
		
	//Lower signal line PB3 to show end of execution
	#ifdef CtrlISRDebugLine
	HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= BIT3LO;
	#endif
	
	//printf("\n\rRPM: %d", RPM);
	
	JumpCount++;
	
	//End of ControlResponse
}

/***************************************************************************
 private functions
 ***************************************************************************/


/****************************************************************************
 Function
    clamp

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

float clamp( float val, float ClampLow, float ClampHigh) {

if(val > ClampHigh)    //if val is too high
	return ClampHigh;     //return ClampHigh
if(val < ClampLow)
	return ClampLow;      //if val is too low
return val;             //otherwise return val because val is just right

}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

