/*
 ============================================================
 Name: InputMainService.h
 Author: Michal Rittikaidachar
 Version:1
 Date Created: 12/8/17
 Description: Header File for Input MainService.c
 Notes:
 ============================================================
 */

#ifndef InputMainSErvice_H
#define InputMainSevice_H

#include "ES_Configure.h"
#include "ES_Types.h"

// Public Function Prototypes
bool InitSPIService( uint8_t Priority );
bool PostSPIService( ES_Event ThisEvent );
ES_Event RunSPIService( ES_Event ThisEvent );
void InitSPI(void);

// typedefs for the states
typedef enum {InitInput, Waiting, XmittingThrottle, XmittingSteering} InputServiceState_t ;

#endif /* InputMainService_H */

