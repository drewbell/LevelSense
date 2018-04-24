/****************************************************************************
 
  Header file for SpeedCounterService 
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef SpeedCounterService_H
#define SpeedCounterService_H

#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "ES_Events.h"

// Public Function Prototypes

bool InitializeSpeedCounter ( uint8_t Priority );
bool PostSpeedCounter( ES_Event ThisEvent );
ES_Event RunSpeedCounter( ES_Event ThisEvent );
float clamp( float val, float ClampLow, float ClampHigh);

#endif /* SpeedCounterService_H */

