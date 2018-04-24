/****************************************************************************
 
  Header file for ADService
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ADService_H
#define ADService_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "ADMulti.h"		  /* gets A/D library */ 

// typedefs for the states
// State definitions for use with the query function
typedef enum { InitADService, Sampling } ADState_t;

// Public Function Prototypes

bool InitializeADService ( uint8_t Priority );
bool PostADService ( ES_Event ThisEvent );
ES_Event RunADService( ES_Event ThisEvent );
uint16_t GetADReading( void );

#endif /* ADService_H */

