/****************************************************************************
 
  Header file for template service 
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ButtonDebounce_H
#define ButtonDebounce_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "ES_Events.h"
//#include "StepService.h"  /* pulls in Step Service public functions */

// typedefs for the states
// State definitions for use with the query function
typedef enum { DEBOUNCING_INIT, Ready2Sample, DEBOUNCING_DIR, DEBOUNCING_MODE, 
	DEBOUNCING_BOTH } ButtonState_t;

// Public Function Prototypes

bool InitializeButtonDebounce ( uint8_t Priority );
bool PostButtonDebounce ( ES_Event ThisEvent );
ES_Event RunButtonDebounceSM( ES_Event ThisEvent );
bool CheckButtonEvents(void);



#endif /* ButtonDebounce_H */

