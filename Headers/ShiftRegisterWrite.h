/****************************************************************************
 Module
   ShiftRegisterWrite.c

 Revision
   0.0.1

 Description
   This module acts as the low level interface to a write only shift register.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 10/24/16 14:55 afb     first rodeo
 
****************************************************************************/

#ifndef SHIFTREGISTERWRITE_H
#define SHIFTREGISTERWRITE_H

// create your own header file comment block
// and protect against multiple inclusions

#include <stdint.h>

/*
Fuction: SR_Init
Usage: SR_Init();
------------------
This functionN initialized the shift register.

*/

void SR_Init(void);

/*
Fuction: SR_GetCurrentRegister
Usage: current_register = SR_GetCurrentRegister()
-------------------------------------------------
This function gets the current register.

*/

uint8_t SR_GetCurrentRegister(void);

/*
Fuction SR_Write
Usage: SR_Write(value);
------------------
This function writes the current register

*/

void SR_Write(uint8_t NewValue);

#endif //SHIFTREGISTERWRITE_H
