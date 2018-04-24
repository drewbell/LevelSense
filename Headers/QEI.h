/****************************************************************************
 
  Header file for QEI service 
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef QEI_H
#define QEI_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitQEI ( uint8_t Priority );
bool PostQEI( ES_Event ThisEvent );
ES_Event RunQEI( ES_Event ThisEvent );
uint16_t getEncoderPosition(uint8_t qeiModule);


#endif /* QEI_H */

