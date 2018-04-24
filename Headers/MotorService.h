/****************************************************************************
 
  Header file for MotorService
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef MotorService_H
#define MotorService_H

#include "ES_Types.h"
#include "ES_Configure.h"
#include "ES_Events.h"

// Public Function Prototypes

bool InitializeMotorService ( uint8_t Priority );
bool PostMotorService( ES_Event ThisEvent );
ES_Event RunMotorService( ES_Event ThisEvent );
uint16_t GetTargetRPM ( void ); 
uint8_t int_clamp( uint8_t val, uint8_t ClampLow, uint8_t ClampHigh);
uint16_t int16_clamp( uint16_t val, uint16_t ClampLow, uint16_t ClampHigh);


#endif /* MotorService_H */

