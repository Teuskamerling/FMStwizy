/************************************************************************************//**
* \file         filelogger.h
* \brief        SD-card file logger header file.
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
#ifndef FILELOGGER_H
#define FILELOGGER_H

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>                                   /* ANSI C types                  */

/****************************************************************************************
* Data definitions
****************************************************************************************/
extern const char LoggerFileName[];
extern const char LoggerFirstLine[];
extern uint32_t fileLoggerMaxFileLength;

/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Element data types. */
typedef enum
{
  FILELOGGER_8BIT_SIGNED,                                        /**< int8_t type      */
  FILELOGGER_8BIT_UNSIGNED,                                      /**< uint8_t type     */
  FILELOGGER_16BIT_SIGNED,                                       /**< int16_t type     */
  FILELOGGER_16BIT_UNSIGNED,                                     /**< uint16_t type    */
  FILELOGGER_32BIT_SIGNED,                                       /**< int32_t type     */
  FILELOGGER_32BIT_UNSIGNED,                                     /**< uint32_t type    */
  FILELOGGER_FLOAT_SINGLE,                                       /**< float type       */
  FILELOGGER_FLOAT_DOUBLE                                        /**< double type      */
} tFileLoggerElementType;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void FileLoggerInit(uint8_t maxElements, uint16_t loggingIntervalTimeMs, 
                    uint8_t dotDecimalSeparator, uint8_t autoStart);
void FileLoggerInitElement(uint8_t elementIdx, void *elementPtr, 
                           tFileLoggerElementType elementType);
void FileLoggerStart(void);
void FileLoggerStop(void);


#endif /* FILELOGGER_H */
/********************************* end of filelogger.h *********************************/


