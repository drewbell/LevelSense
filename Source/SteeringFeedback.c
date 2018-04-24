/****************************************************************************
 Module
   SteeringFeedback.c

 Revision
   1.0.1

 Description
   This is a simple service for operating steering wheel feedback service under the 
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 12/05/12 09:58 afb      began conversion from Templateservice.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "SteeringFeedback.h"
#include <stdint.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "inc/hw_pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/qei.h"
#include "driverlib/pwm.h"

#include "SpeedCounterService.h"
#include "MotorService.h"
#include "constants.h"
#include "QEI.h"
#include <math.h>


/*----------------------------- Module Defines ----------------------------*/


    

// TEST PRINTS
// #define TEST_DAC_CONV           // shows the ticks + DAC values for steering and throttle
//#define TEST_STEER_FEEDBACK    // shows the steering strength, drive mode, and signed steer
//#define PRINT_DUTY_COMMAND
//#define TEST_STEER_ONLY

#define TWENTY_FIVE_MS ONE_SEC/40

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

void updateSteeringForce(uint16_t steeringTicks);
uint16_t steeringTo12Bit(uint16_t steeringTicks);
uint16_t throttleTo12Bit(uint16_t throttleTicks);


/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static float RTC_Strength = 0.0;    // RTC_Strength has range [0.0, 100.0]
static uint16_t steering_12bit;
static uint16_t throttle_12bit; 

uint8_t motorDriveTable[4][3] = {    
                            {1, 1, 0},  // FWD mode
                            {1, 0, 1},  // REV mode
                            {1, 1, 1},  // BRAKE mode
                            {0, 0, 0}   // FREE mode 
                            };

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitSteeringFeedback

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any 
     other required initialization for this service
 Notes

 Author
     Drew Bell, 12/05/12, 10:00
****************************************************************************/
bool InitSteeringFeedback ( uint8_t Priority )
{
    ES_Event ThisEvent;

    MyPriority = Priority;

    printf("Init Steering Feedback...");
    
   //Configure PWM Clock to match system
   SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    // enable port a clock
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
	{
	}
    
    // set PA5 and PA6 to GPIO outputs and set low
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_6, 0x00);

    // set up PWM for PA7 (M1PWM3)
    // first enable the peripheral clock
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1))
    {
    }
    
    // configure PA7 to be used by PWM
    GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_7);
    
    // configure the alternate function of the PWM pin
    GPIOPinConfigure(GPIO_PA7_M1PWM3);      // PA7 is M1PWM3
    
    // make sure output is disabled
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, false);
    
    //
    // Configure the PWM generator for count down mode with immediate updates
    // to the parameters.
    //
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    
    //
    // Set the period. 
    //
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, MOTOR_PWM_PERIOD);

    // set the initial strength as default
    setReturnStrength(DEFAULT_RTC_STRENGTH);
    
    //
    // Start the timers in generator 0.
    //
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);
    
    // enable steering feedback from the start
    setReturnStrength(DEFAULT_RTC_STRENGTH);
    ES_Timer_InitTimer(ENCODER_INPUT_TIMER, TWENTY_FIVE_MS);
    
    printf("\n\r   Encoder 0 range: %i", ENCODER_0_RANGE);
    printf("\n\rInit SteeringFeedback Done.\n\r");

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
     PostSteeringFeedback

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
bool PostSteeringFeedback( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunSteeringFeedback

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
ES_Event RunSteeringFeedback( ES_Event ThisEvent )
{
    ES_Event ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
    
  
    switch(ThisEvent.EventType){
        
        case ES_ENABLE_RTC:
             // if the RTC_Strength is 0.0, set to a non-zero default
            if(RTC_Strength == (float) 0.0){       
                setReturnStrength(DEFAULT_RTC_STRENGTH);
            }
            else{   // else set to RTC_Strength              
                setReturnStrength(RTC_Strength);    // set param as the RTC strength
            }
            ES_Timer_InitTimer(ENCODER_INPUT_TIMER, TWENTY_FIVE_MS);
            break;
            
        case ES_DISABLE_RTC:
            // set RTC strength to 0
            setReturnStrength(0.0);
            ES_Timer_StopTimer(ENCODER_INPUT_TIMER);
            break;
        
        case ES_INC_RTC_STRENGTH:               // case INC_RTC_FORCE
            RTC_Strength += RTC_STRENGTH_INC;   // raise RTX stength by an increment
            setReturnStrength(RTC_Strength);
            break;
        
        case ES_DEC_RTC_STRENGTH:               // case DEC_RTC_FORCE
            RTC_Strength -= RTC_STRENGTH_INC;   // decrease RTC 
            setReturnStrength(RTC_Strength);
            break;
        
        case ES_TIMEOUT:
            // if timeout is the ecoder input timer, update steering force and get throttle
            if(ThisEvent.EventParam == ENCODER_INPUT_TIMER){
                
                // get steeringTicks and store as 12-bit value for DAC
                uint16_t steeringTicks = getEncoderPosition(ENCODER_0);
                steering_12bit = steeringTo12Bit(steeringTicks);
                
                // get throttleTicks and store as 12-bit value for DAC
                uint16_t throttleTicks = getEncoderPosition(ENCODER_1);
                throttle_12bit = throttleTo12Bit(throttleTicks);
                
                // test print of DAC conversion
                #ifdef TEST_DAC_CONV
                static uint8_t printcount;
                printcount++;
                if(printcount > 5){
                    printf("\n\rTest DAC: \n\r");
                    printf("   Steering -> TICKS %i, DAC: %i\n\r", steeringTicks, steering_12bit); 
                    printf("   Throttle -> TICKS %i, DAC: %i\n\r", throttleTicks, throttle_12bit); 
                    printcount = 0; 
                }
                #endif
                
                // update the steering force    
                updateSteeringForce(steeringTicks);
                
                // re-kick the timer
                ES_Timer_InitTimer(ENCODER_INPUT_TIMER, TWENTY_FIVE_MS);
            }
    }
    
    
  return ReturnEvent;
}


/****************************************************************************
 Function
    setReturnStrength

 Parameters
   takes a float called strength from 0.0 to 100.0

 Returns
   nothing

 Description
    A function that takes a input strength from 0.0 to 100.0 and sets the 
    PWM output to that value times the highest allowable duty for a stalled
    motor. 
 Notes
    - This function assumes the highest safe duty cycle for a stalled motor
      is 25% (Source: Pololu's website). Even then, 25% is probably pushing it.
    - The duty is kept above a floor of 1.0 strength
    - To enable the output, use the setMotorMode to an appropriate mode
   
 Author
   Drew Bell, 12/05/17, 15:23
****************************************************************************/

void setReturnStrength(float strength){
    uint32_t pulseWidth = 0;
    static uint16_t printCount = 0;
      
    float duty = clamp(strength, MIN_RTC_DUTY, MAX_RTC_DUTY);
    
//    if(duty < (float) 0.01){    // if strength near zero, impose a floor for motor reasons
//        RTC_Strength = 0.01;
//    }
    // calculate the correct pulseWidth
    pulseWidth = (uint32_t) (duty * (float)MOTOR_PWM_PERIOD);
        
    // sets the duty cycle for M1PWM 3 to a corresponding duty cycle 
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, pulseWidth);
    
    //printf("\n\rRTC Strength: %.2f. RTC PW: %d", RTC_Strength, duty);
    
    printCount++;
    #ifdef PRINT_DUTY_COMMAND
        
    if(printCount == 0x1f){
        printf("\n\rDuty: %.2f. Pulse: %d. Str: %.2f", duty, pulseWidth, strength);
        printCount = 0;
    }
    #endif
    
}

/****************************************************************************
 Function
    setMotorMode

 Parameters
   uint8_t mode

 Returns
   nothing

 Description
    A function that takes a mode input and commands the motor driver to execute
    the specified drive behavior.
 Notes
    See driveModeTable and related defines for drive modes
   
 Author
   Drew Bell, 12/05/17, 15:23
****************************************************************************/
void setMotorMode(uint8_t mode){
    //        | enable | IN1 | IN2 | 
    // 0 fwd    
    // 1 rev
    // 2 brake
    // 3 free
    
    uint8_t newMode = int_clamp(mode, 0, 3);    // make sure mode input is valid
    
    // set the drive input lines based on the the mode passed using motorDriveTable
    uint8_t PA5_driveState = GPIO_PIN_5 & motorDriveTable[mode][IN1]<<5;
    uint8_t PA6_driveState = GPIO_PIN_6 & motorDriveTable[mode][IN2]<<6;
    
    // set these inputs to the pins
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_6, PA5_driveState | PA6_driveState);
    
    // if the mode indicates active enable line, set it so
    if(motorDriveTable[mode][MOTOR_ENABLE]){
        PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT , true);     // enable PWM output
    }
    else{
        PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT , false);     // disable PWM output
    }
    
    /*
    printf("\n\rMotor mode: %d. PA5: %x, PA6: %x, ENA: %i, str: %.2f", newMode, PA5_driveState, 
        PA6_driveState, motorDriveTable[mode][MOTOR_ENABLE], RTC_Strength);
    */
}

/*===========================================================================
 Function: get12BitSteeringCmd
 Parameters: void
 Returns: uint16_t steering_12bit  
 Description: Returns steering command to other services
 Author: Drew Bell 12/09/17
 Notes: 

  When	Who	 What/Why
 ------	---	 --------
 12/6/17  db		Created function 

===========================================================================*/
uint16_t get12BitSteeringCmd( void ) {
	return steering_12bit;
}

/*===========================================================================
 Function: get12BitThrottleCmd
 Parameters: void
 Returns: uint16_t throttle_12bit  
 Description: Returns throttle command to other services
 Author: Drew Bell 12/09/17
 Notes: 

  When	Who	 What/Why
 ------	---	 --------
 12/6/17  db		Created function 

===========================================================================*/
uint16_t get12BitThrottleCmd( void ) {
	return throttle_12bit;
}


/***************************************************************************
 private functions
 ***************************************************************************/
/****************************************************************************
 Function
    updateSteerForce

 Parameters
   none

 Returns
   nothing

 Description
    This function updates the steering return force.

 Notes
 First pass: non-interrupt code the simply turns on the force if the position 
 is outside of 3 degrees
 
 Author
   Drew Bell, 12/06/17, 15:23
****************************************************************************/

void updateSteeringForce(uint16_t steeringTicks){
    // initialize some local vars with default values
    float newStrength = 0.0;
    uint8_t driveMode = MOTOR_OFF; 
    static uint8_t printCount = 0;
    static uint32_t sumError = 0;
    static float K_static = 0.40f;
    static float Ki = 0.00001f;
    static float Kp = 0.012f;       //  spring stiffness
    
    
    // calculate a signed position and clamp between 0 and max encoder range
    int16_t signedSteering = steeringTicks - ENCODER_0_CENTER;

    // if the wheel is in the deadband, set rtc strength to 0
    if(signedSteering >= STEER_DB_EDGE_CCW && signedSteering <= STEER_DB_EDGE_CW){
        newStrength = 0.0f;
        driveMode = MOTOR_OFF;
    }
    // else calculate the signed return force
    else{
        
        // Force profile: constant base plus spring force
        // first calculate a base strength with the same sign as the steering
        float baseStrength = -K_static * copysignf(BASE_RETURN_STRENGTH, signedSteering);
        // then calculate a proportional return force
        float proportionalStrength = -Kp * signedSteering;
        
        // calculate the integrator term and clamp to a max % stength
        sumError = sumError + signedSteering;
        float integratorStrength =  (-Ki * (float) sumError);
        if(integratorStrength > 50.f){
            integratorStrength = 50.f;
        }
        else if(integratorStrength < -50.f){
            integratorStrength = -50.f;
        }
        
        // sum new strength to apply
        //newStrength = baseStrength + proportionalStrength + integratorStrength;
        newStrength = proportionalStrength;
        //printf("\n\rNS = %.2f", fabs(newStrength));
    }
    
    // if the strength is greater than 0, drive forward
    if(newStrength > 0.0f){
        driveMode = MOTOR_FWD;
    }
    // else if strength is less than 0, drive rev
    else if(newStrength < 0.0f){
        driveMode = MOTOR_REV;
    }
    
    // set drive mode and the return strength magnitude to abs value of the newStrength
    setMotorMode(driveMode);
    setReturnStrength(fabs(newStrength));
    
    // increment printCount outside of ifdef to avoid compiler warning
     printCount++;
    
    // print steer feedback strength, mode, and signed steering position
    #ifdef TEST_STEER_FEEDBACK
    if(printCount == 20){
        printf("\n\rSteering Feedback State: \n\r   Strength: %.2f \n\r   Signed Steer: %i   sumError: %i\n\r", 
            newStrength, signedSteering, sumError);
        printf("   Drive mode:%i\n\r", driveMode);
        if(driveMode == MOTOR_CW){
            printf("   Drive mode: ON - Clockwise\n\r");
        }
        else if(driveMode == MOTOR_CCW){
            printf("   Drive mode: ON - Counter-clockwise\n\r");
        }
        else if(driveMode == MOTOR_BRAKE){
            printf("   Drive mode: ON - Brake\n\r");
        }
        else if(driveMode == MOTOR_OFF){
            printf("   Drive mode: OFF\n\r");
        }
        printCount = 0;     // reset counter
    }
    #endif
		
		#ifdef TEST_STEER_ONLY
		printf("\n\r%i", signedSteering);
		#endif
    
}

/****************************************************************************
 Function
    steeringTo12Bit

 Parameters
   uint16_t steeringTicks

 Returns
   uint16_t steering12Bits

 Description
    This function takes a 16 bit number as a steering output from the encoder
    and maps it to 12 bits, reading to be send to the DAC

 Notes
 
 Author
   Drew Bell, 12/08/17, 15:23
****************************************************************************/
uint16_t steeringTo12Bit(uint16_t steeringTicks){
    // convert a a 12 bit number, careful not to overflow 16 bits
    uint32_t convertSteering = steeringTicks * STEER_TO_12_BIT / 10;    
    if(convertSteering > MAX_12_BITS){
        printf("ERROR: Steering Feedback Conversion Overflow\n\r");
    }
    uint16_t new12BitSteering = int16_clamp((uint16_t) convertSteering, 0, MAX_12_BITS);
    //printf("\n\r12 bit steering: %i", steering_12bit);
    
    return (uint16_t) new12BitSteering; 
}

/****************************************************************************
 Function
    throttleTo12Bit

 Parameters
   uint16_t throttleTicks, throttleMin, throttleMax

 Returns
   uint16_t throttle12Bits

 Description
    This function takes a 16 bit number as a throttle output from the encoder
    and maps it to 12 bits, reading to be send to the DAC

 Notes
 
 Author
   Drew Bell, 12/08/17, 15:23
****************************************************************************/
uint16_t throttleTo12Bit(uint16_t throttleTicks){
     
    // first clamp the ticks within a predefined min or max to avoid spurious calculation
    uint16_t clampedTicks = int16_clamp(throttleTicks, THROTTLE_MIN_TICK, THROTTLE_MAX_TICK);
    
    // calulate the 
    uint16_t new12BitThrottle = HALF_OF_12_BITS + (-clampedTicks + THROTTLE_MIDDLE) * THROTTLE_BIT_CONV 
            + THROTTLE_TUNE_FACTOR;
    
    
      
    // clamp the output just to be safe
    new12BitThrottle = int16_clamp(new12BitThrottle, 0, MAX_12_BITS);
    
    return new12BitThrottle; 
}
    


/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

