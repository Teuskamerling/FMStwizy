/************************************************************************************//**
* \file         servo.h
* \brief        Servo driver header file.
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

#ifndef SERVO_H_
#define SERVO_H_

/****************************************************************************************
Include files
****************************************************************************************/
#include "stm32f10x.h"

/****************************************************************************************
* Macro definitions
****************************************************************************************/
#define SERVO_TIM1_PIN_PA8 (0)
#define SERVO_TIM1_PIN_PA9 (1)
#define SERVO_TIM1_PIN_PA1 (2)
#define SERVO_TIM2_PIN_PA0 (3)
#define SERVO_TIM2_PIN_PA1 (4)
#define SERVO_TIM2_PIN_PA3 (5)
#define SERVO_TIM3_PIN_PC6 (6)
#define SERVO_TIM3_PIN_PC7 (7)
#define SERVO_TIM3_PIN_PC8 (8)
#define SERVO_TIM4_PIN_PB6 (9)
#define SERVO_TIM4_PIN_PB7 (10)
#define SERVO_TIM4_PIN_PB8 (11)
#define SERVO_TIM4_PIN_PB9 (12)

/****************************************************************************************
* Type definitions
****************************************************************************************/

/****************************************************************************************
* Function prototypes
****************************************************************************************/
void ServoInit(const uint8_t pinID, const uint16_t min_pulse_width_in_us, const uint16_t max_pulse_width_in_us);
void ServoUpdate(const uint8_t pinID, uint16_t pulse_width_in_us);

#endif /* SERVO_H_ */
/********************************* end of servo.h *************************************/
