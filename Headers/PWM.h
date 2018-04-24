/****************************************************************************
 
  Header file for PWM Module for ME218 Lab6 

 ****************************************************************************/

#ifndef PWM_H
#define PWM_H

#include "ES_Types.h"

// Public Function Prototypes

void InitPWM ( void );			//initialize PWM function
void SetDC_A ( uint8_t PWMDuty_A); 			//set the PWM duty cycle to given parameter, valid inputs are 0 to 100
void SetPWMPeriod( uint16_t NewPWMPeriod);   //set PWM duty cycle by input desired in uS
	
#endif /* PWM_H */

