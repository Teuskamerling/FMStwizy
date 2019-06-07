/************************************************************************************//**
* \file         diag.h
* \brief        Diagnostics module header file. 
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
 \endinternal
****************************************************************************************/
#ifndef DIAG_H
#define DIAG_H

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>                                   /* ANSI C types                  */


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Possible diagnostic sessions. */
typedef enum
{
  DIAG_SESSION_DEFAULT = 1,            /**< default diagnostic session                 */
  DIAG_SESSION_ECU_EXTENDED = 3        /**< ECU extended session                       */
} tDiagSession;

/** \brief Function type for the get seed callback handler. */
typedef void (* tDiagCallbackGetSeed)(void);

/** \brief Function type for the verify key callback handler. */
typedef void (* tDiagCallbackVerifyKey)(void);


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void     DiagInit(uint8_t require_seedkey, uint16_t txBufferSize);
void     DiagSetSeed(uint32_t seed);
uint32_t DiagGetSeed(void);
uint32_t DiagGetKey(void);
void     DiagSetKeyVerified(uint8_t keyOkay);
void     DiagRegisterGetSeedCallback(tDiagCallbackGetSeed callbackPtr);
void     DiagRegisterVerifyKeyCallback(tDiagCallbackVerifyKey callbackPtr);


#endif /* DIAG_H */
/********************************* end of diag.h ***************************************/
