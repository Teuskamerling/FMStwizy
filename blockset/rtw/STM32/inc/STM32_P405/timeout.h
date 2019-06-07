/************************************************************************************//**
* \file         timeout.h
* \brief        Timer outputs (output compare) driver header file.
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
#ifndef TIMEOUT_H
#define TIMEOUT_H

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>                                   /* ANSI C types                  */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/* timer module identifiers. note that these are used as array indexers so make
 * sure the first one has value 0 and the others are increments.
 */
#define TIMEOUT_MODULE_TIM1   (0)                     /* Timer module 1                */
#define TIMEOUT_MODULE_TIM2   (1)                     /* Timer module 2                */
#define TIMEOUT_MODULE_TIM3   (2)                     /* Timer module 3                */
#define TIMEOUT_MODULE_TIM4   (3)                     /* Timer module 4                */


/* timer input capture pin identifiers. note that these are used as array indexers so
 * make sure the first one has value 0 and the others are increments.
 */
#define TIMEOUT_TIM1_PIN_PA8   (0)              /* PA-8   (Ch. 1) - CH1N=PB13          */
#define TIMEOUT_TIM1_PIN_PA9   (1)              /* UEXT-3 (Ch. 2) - CH2N=PB0           */
#define TIMEOUT_TIM1_PIN_PA10  (2)              /* UEXT-4 (Ch. 3) - CH3N=PB1           */
#define TIMEOUT_TIM2_PIN_PA1   (3)              /* PA-1   (Ch. 2)                      */
#define TIMEOUT_TIM3_PIN_PC6   (4)              /* PC-6   (Ch. 1)                      */
#define TIMEOUT_TIM3_PIN_PC7   (5)              /* PC-7   (Ch. 2)                      */
#define TIMEOUT_TIM3_PIN_PC8   (6)              /* PC-8   (Ch. 3)                      */
#define TIMEOUT_TIM3_PIN_PC9   (7)              /* PC-9   (Ch. 4)                      */
#define TIMEOUT_TIM4_PIN_PB6   (8)              /* UEXT-5 (Ch. 1)                      */
#define TIMEOUT_TIM4_PIN_PB7   (9)              /* UEXT-6 (Ch. 2)                      */
#define TIMEOUT_TIM4_PIN_PB8   (10)             /* PB-8   (Ch. 3) - CANRX              */
#define TIMEOUT_TIM4_PIN_PB9   (11)             /* PB-9   (Ch. 4) - CANTX              */


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Function type for the user specified free running counter overflow interrupt 
 *         handler.
 */
typedef void (* tTimeoutCallbackOverflow)(void);

/** \brief Function type for the user specified output compare event interrupt handler.*/
typedef void (* tTimeoutCallbackCompareEvent)(void);

/** \brief Configuration values for the output compare event action. */
typedef enum
{
  SETLOW  = 0,                                  /**< Set the channel pin low           */
  SETHIGH = 1,                                  /**< Set the channel pin high          */
  INVERT  = 2,                                  /**< Invert the channel pin's state    */
  NONE    = 3                                   /**< Do nothing (pure software timer   */
} tTimeoutAction;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void     TimeoutInitModule(uint8_t module_id, uint32_t frequency);
void     TimeoutScheduleCompareEvent(uint8_t pin_id, uint16_t eventCounter, tTimeoutAction action);
uint16_t TimeoutGetFreeRunningCounter(uint8_t module_id);
void     TimeoutResetFreeRunningCounter(uint8_t module_id);
uint16_t TimeoutGetLastEventCounter(uint8_t pin_id);
void     TimeoutRegisterOverflowCallback(uint8_t module_id, tTimeoutCallbackOverflow callbackPtr);
void     TimeoutRegisterCompareEventCallback(uint8_t pin_id, tTimeoutCallbackCompareEvent callbackPtr);
void     Timeout1UpdateTimeout10Interrupt(void);
void     Timeout1CaptureInterrupt(void);
void     Timeout2Interrupt(void);
void     Timeout3Interrupt(void);
void     Timeout4Interrupt(void);


#endif /* TIMEOUT_H */
/********************************* end of timeout.h ************************************/


