/************************************************************************************//**
* \file         timein.h
* \brief        Timer inputs driver header file.
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
#ifndef TIMEIN_H
#define TIMEIN_H

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>                                   /* ANSI C types                  */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/* timer input capture pin identifiers. note that these are used as array indexers so
 * make sure the first one has value 0 and the others are increments.
 */
#define TIMEIN_TIM1_PIN_PA8    (0)              /* PA-8   (Ch. 1) - CH1N=PB13          */
#define TIMEIN_TIM1_PIN_PA9    (1)              /* UEXT-3 (Ch. 2) - CH2N=PB0           */
#define TIMEIN_TIM1_PIN_PA10   (2)              /* UEXT-4 (Ch. 3) - CH3N=PB1           */
#define TIMEIN_TIM2_PIN_PA1    (3)              /* PA-1   (Ch. 2)                      */
#define TIMEIN_TIM3_PIN_PC6    (4)              /* PC-6   (Ch. 1)                      */
#define TIMEIN_TIM3_PIN_PC7    (5)              /* PC-7   (Ch. 2)                      */
#define TIMEIN_TIM3_PIN_PC8    (6)              /* PC-8   (Ch. 3)                      */
#define TIMEIN_TIM3_PIN_PC9    (7)              /* PC-9   (Ch. 4)                      */
#define TIMEIN_TIM4_PIN_PB6    (8)              /* UEXT-5 (Ch. 1)                      */
#define TIMEIN_TIM4_PIN_PB7    (9)              /* UEXT-6 (Ch. 2)                      */
#define TIMEIN_TIM4_PIN_PB8    (10)             /* PB-8   (Ch. 3) - CANRX              */
#define TIMEIN_TIM4_PIN_PB9    (11)             /* PB-9   (Ch. 4) - CANTX              */


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Function type for the user specified edge detected interrupt handler */
typedef void (* tTimeinCallbackEdgeDetected)(void);


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void     TimeinConfigure(uint8_t pin_id);
void     TimeinConfigureZeroHzTimeout(uint8_t pin_id, uint16_t zero_hz_timeout_ms);
void     TimeinRegisterEdgeDetectedCallback(uint8_t pin_id, tTimeinCallbackEdgeDetected callbackPtr);
uint32_t TimeinGetFrequency(uint8_t pin_id);
uint16_t TimeinGetDutyCycle(uint8_t pin_id);
uint32_t TimeinGetLastEdgeCount(uint8_t pin_id);
uint8_t  TimeinGetLastEdgeState(uint8_t pin_id);
uint32_t TimeinGetFreeRunningCounter(uint8_t pin_id);
uint32_t TimeinGetEdgeCounter(uint8_t pin_id);
uint8_t  TimeinIsZeroHzDetected(uint8_t pin_id);
void     Timein1UpdateTimein10Interrupt(void);
void     Timein1CaptureInterrupt(void);
void     Timein2Interrupt(void);
void     Timein3Interrupt(void);
void     Timein4Interrupt(void);


#endif /* TIMEIN_H */
/********************************* end of timein.h *************************************/


