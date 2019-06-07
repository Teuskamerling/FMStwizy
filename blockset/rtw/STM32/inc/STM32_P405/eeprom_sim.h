/************************************************************************************//**
* \file         eeprom_sim.h
* \brief        Simulated EEPROM driver header file.
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*  Copyright 2019 (c)  by HAN Automotive   http://www.han.nl        All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* Permission is hereby granted, free of charge, to any person obtaining a copy of this 
* software and associated documentation files (the "Software"), to deal in the Software
* without restriction, including without limitation the rights to use, copy, modify, merge,
* publish, distribute, sublicense, and/or sell copies of the Software, and to permit
* persons to whom the Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
* OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.
* \endinternal
****************************************************************************************/
#ifndef EEPROM_SIM_H
#define EEPROM_SIM_H

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>                                   /* ANSI C types                  */


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void     EepromSimInit(void);
void     EepromSimSynchronize(void);
void     EepromSimBulkErase(void);
void     EepromSimErase(uint16_t address, uint16_t len);
uint8_t *EepromSimGetPtr(uint16_t address);
uint8_t  EepromSimUINT8Get(uint16_t address);
int8_t   EepromSimSINT8Get(uint16_t address);
uint16_t EepromSimUINT16Get(uint16_t address);
int16_t  EepromSimSINT16Get(uint16_t address);
uint32_t EepromSimUINT32Get(uint16_t address);
int32_t  EepromSimSINT32Get(uint16_t address);
void     EepromSimUINT8Set(uint16_t address, uint8_t value);
void     EepromSimSINT8Set(uint16_t address, int8_t value);
void     EepromSimUINT16Set(uint16_t address, uint16_t value);
void     EepromSimSINT16Set(uint16_t address, int16_t value);
void     EepromSimUINT32Set(uint16_t address, uint32_t value);
void     EepromSimSINT32Set(uint16_t address, int32_t value);


#endif /* EEPROM_SIM_H */
/********************************* end of eeprom_sim.h *********************************/


