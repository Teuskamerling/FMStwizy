/************************************************************************************//**
* \file         errorcodes.h
* \brief        Error codes module header file.
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
#ifndef ERRORCODES_H
#define ERRORCODES_H

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>                                   /* ANSI C types                  */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Maximum number of errors that can be stored by this module. */
#define ERRCODES_NUM_OF_MAX_ERRORS    (48)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Structure type for grouping the data the belongs to an error code. */
typedef struct
{
  uint16_t code;
  uint8_t  param;
  uint8_t  occurrence;
  uint32_t timestamp;
} __attribute__((packed)) tErrCodeData;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void    ErrCodesInit(void);
void    ErrCodesSetError(uint16_t errCode, uint8_t errParam, uint8_t errSaveEeprom);
void    ErrCodesDelSavedErrors(void);
void    ErrCodesDelActiveErrors(void);
void    ErrCodesDelActiveError(uint16_t errCode, uint16_t errParam);
uint8_t ErrCodesGetNumSavedErrors(void);
uint8_t ErrCodesGetNumActiveErrors(void);
uint8_t ErrCodesGetSavedErrors(tErrCodeData *errData, uint8_t startIndexErrors, uint8_t MaxNumErrors);
uint8_t ErrCodesGetActiveErrors(tErrCodeData *errData, uint8_t startIndexErrors, uint8_t MaxNumErrors);
uint8_t ErrCodesSearchSavedError(uint16_t errCode, uint8_t errParam, tErrCodeData *errData);
uint8_t ErrCodesSearchActiveError(uint16_t errCode, uint8_t errParam, tErrCodeData *errData);
uint8_t ErrCodesTestError(uint16_t errCode, uint16_t errParam);



#endif /* ERRORCODES_H */
/********************************* end of errorcodes.h *********************************/


