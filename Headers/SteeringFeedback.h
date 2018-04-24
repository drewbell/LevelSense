/****************************************************************************
 
  Header file for Steering Feedback service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef SteeringFeedback_H
#define SteeringFeedback_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitSteeringFeedback ( uint8_t Priority );
bool PostSteeringFeedback( ES_Event ThisEvent );
ES_Event RunSteeringFeedback( ES_Event ThisEvent );
void setReturnStrength(float strength);
void setMotorMode(uint8_t mode);
uint16_t get12BitThrottleCmd( void );
uint16_t get12BitSteeringCmd( void );

#endif /* SteeringFeedback_H */

