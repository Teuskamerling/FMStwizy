/************************************************************************************//**
* \file         os.h
* \brief        Operating system header file.
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
#ifndef OS_H
#define OS_H

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>                                   /* ANSI C types                  */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "list.h"
#include "timers.h"


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Configures the frequency of the timer used for run-time measurements. It is
 *         used to scale the timer used for the system tick. Note that some changes for
 *         this where made in the FreeRTOS port.
 */
#define OS_RUNTIME_COUNTER_FREQUENCY   (configTICK_RATE_HZ * 10)


/****************************************************************************************
* Function prototypes
****************************************************************************************/
int      main(void);
void     OsIrqEnter(void);
void     OsIrqExit(void);
uint32_t OsEnterCriticalSection(void);
void     OsLeaveCriticalSection(uint32_t old_cs_state);
void     OsSystemReset(void);
uint8_t  OsGetCpuLoad(void);
uint8_t  OsGetCpuLoadMax(void);
uint32_t OsGetFreeHeapSize(void);
uint32_t OsGetTaskFreeStackSize(char *name);
uint32_t OsGetSystemTime(void);
void     OsRunTimeMeasurementTickEvent(void);



#endif /* OS_H */
/********************************* end of os.h *****************************************/


