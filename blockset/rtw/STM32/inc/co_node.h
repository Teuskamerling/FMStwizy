/************************************************************************************//**
* \file         co_node.h
* \brief        CANopen node header file.
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
#ifndef CO_NODE_H
#define CO_NODE_H

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>                                   /* ANSI C types                  */
#include "CANopen.h"                                  /* CANopen stack header          */
#include "CO_eeprom.h"                                /* CANopen EEPROM support        */


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void     CoNodeInit(uint8_t canChannel);
uint8_t  CoNodeGetChannel(void);
uint8_t  CoNodeODReadUINT8(uint16_t index, uint8_t subindex);
int8_t   CoNodeODReadSINT8(uint16_t index, uint8_t subindex);
uint16_t CoNodeODReadUINT16(uint16_t index, uint8_t subindex);
int16_t  CoNodeODReadSINT16(uint16_t index, uint8_t subindex);
uint32_t CoNodeODReadUINT32(uint16_t index, uint8_t subindex);
int32_t  CoNodeODReadSINT32(uint16_t index, uint8_t subindex);
void     CoNodeODWriteUINT8(uint16_t index, uint8_t subindex, uint8_t data);
void     CoNodeODWriteSINT8(uint16_t index, uint8_t subindex, int8_t data);
void     CoNodeODWriteUINT16(uint16_t index, uint8_t subindex, uint16_t data);
void     CoNodeODWriteSINT16(uint16_t index, uint8_t subindex, int16_t data);
void     CoNodeODWriteUINT32(uint16_t index, uint8_t subindex, uint32_t data);
void     CoNodeODWriteSINT32(uint16_t index, uint8_t subindex, int32_t data);



#endif /* CO_NODE_H */
/********************************* end of co_node.h ************************************/


