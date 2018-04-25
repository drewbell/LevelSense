/****************************************************************************
 Module
   ShiftRegisterRead.c

 Revision
   0.0.1

 Description
   This module acts as the low level interface to a read only shift register.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 04/24/18 14:55 afb     converting over from shiftRegisterWrite
 
****************************************************************************/

#ifndef SHIFT_REGISTER_READ_H
#define SHIFT_REGISTER_READ_H

// create your own header file comment block
// and protect against multiple inclusions

#include <stdint.h>

/*
Fuction: SR_Read_Init
Usage: SR_Read_Init();
------------------
This functionn initialized the shift register.

*/

void SR_Read_Init(void);

/*
Fuction: SR_Read_GetCurrReg
Usage: current_register = SR_Read_GetCurrReg()
-------------------------------------------------
This function gets the current register.

*/

uint8_t SR_Read_GetCurrReg(void);

/*
Fuction SR_Read
Usage: SR_Read(value);
------------------
This function reads the current register

*/

uint8_t SR_Read(void);

#endif //SHIFT_REGISTER_READ_H
