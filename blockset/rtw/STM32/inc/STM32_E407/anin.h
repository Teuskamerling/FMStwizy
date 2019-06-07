/************************************************************************************//**
* \file         anin.h
* \brief        Analog inputs driver header file.
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
#ifndef ANIN_H
#define ANIN_H

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>                                   /* ANSI C types                  */
#include "model.h"									  /* Variables changes by Simulink */
#include "stm32f4xx.h"                                /* STM32 registers               */
#include "stm32f4xx_conf.h"                           /* STM32 peripheral drivers      */
#include "SYS_config.h"								  /* For defines from Simulink	   */

/****************************************************************************************
* Macro definitions
****************************************************************************************/
/* analog input port identifiers. note that these are used as array indexers so make
 * sure the first one has value 0 and the others are increments.
 */
#define ANIN_PORTF_PIN3     (0)                /* Connector PF - 6       (ADC3_IN9)    */
#define ANIN_PORTF_PIN4     (1)                /* Connector PF - 7       (ADC3_IN14)   */
#define ANIN_PORTF_PIN5     (2)                /* Connector PF - 8       (ADC3_IN15)   */
#define ANIN_PORTF_PIN6     (3)                /* Connector 2 pin 2 / A1 (ADC3_IN4)    */
#define ANIN_PORTF_PIN7     (4)                /* Connector 2 pin 3 / A2 (ADC3_IN5)    */
#define ANIN_PORTF_PIN8     (5)                /* Connector 2 pin 4 / A3 (ADC3_IN6)    */
#define ANIN_PORTF_PIN9     (6)                /* Connector 2 pin 5 / A4 (ADC3_IN7)    */
#define ANIN_PORTF_PIN10    (7)                /* Connector 2 pin 6 / A5 (ADC3_IN8)    */
#define ANIN_PORTC_PIN0     (8)                /* Connector 2 pin 1 / A0 (ADC123_IN10) */
#define ANIN_PORTC_PIN2     (9)                /* Connector UEXT pin 7   (ADC123_IN12) */

/** \brief Total number of analog input channels */
#define ANIN_MAX_CHANNELS         (10)

/** \brief Invalid index value for array aninChannelCfgInfo */
#define ANIN_INVALID_CHANNEL_IDX  (ANIN_MAX_CHANNELS)

/** \brief Total number of samples in the median filter */
#define FILTERSIZE 16

/** \brief Number of samples for averaging in the filter */
#define LOWPASSFILTERSIZE 8

/** \brief Constant for deviding the final result */
#define LOWPASSFILTERSHIFT 3 /* divide by 8 */


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Structure type with pin mapping information. */
typedef struct
{
  uint32_t peripheral;
  GPIO_TypeDef* port;
  uint16_t pin;
  uint8_t adcChannel;
} tAninPinMapping;

/** \brief Structure type to keep track of a channel configuration. */
typedef struct
{
  uint8_t pinIdx;
  uint8_t configuredFlg;
  uint8_t filterFlg;
} tAninChannelCfgInfo;

/** \brief Struct that holds all information for filtering the analog inputs
 */
typedef struct
{
  uint8_t pinId;
  uint32_t Samples[FILTERSIZE];
  uint32_t FilterResult;
  uint32_t SampleAgeIndex;	
}AninFilterStruct;

/****************************************************************************************
* Global data declarations
****************************************************************************************/
/** \brief Variable that holds the numder of analog channels with filtering */
extern uint8_t nrFilteredADCchannels;

/****************************************************************************************
* Function prototypes
****************************************************************************************/
void     AninConfigure(uint8_t id, uint8_t filtered);
uint16_t AninGet(uint8_t id, uint8_t filtered);
void ADCconversionTaskSetPeriod(uint32_t period);
void ADCconversionTask(void *pvParameters);
void ADCconversionTaskFunction(void);


#endif /* ANIN_H */
/********************************* end of anin.h ***************************************/


