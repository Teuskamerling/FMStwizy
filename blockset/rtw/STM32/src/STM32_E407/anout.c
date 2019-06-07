/************************************************************************************//**
* \file         anout.c
* \brief        Analog outputs driver source file.
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

/****************************************************************************************
* Include files
****************************************************************************************/
#include "stm32f4xx.h"
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */

/****************************************************************************************
* Macro definitions
****************************************************************************************/
#define DAC_DHR12R1 (*(volatile uint32_t *)0x40007408)
#define DAC_DHR12R2 (*(volatile uint32_t *)0x40007414)
#define DAC_DHR12RD (*(volatile uint32_t *)0x40007420)

/************************************************************************************//**
** \brief     Initializes the analog outputs driver.
** \return    none.
**
****************************************************************************************/
void DAC_init(unsigned short DACchannel)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  DAC_InitTypeDef  DAC_InitStructure;

  // DAC Periph clock enable
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

  // GPIOA clock enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  // Configure PA.04, .05 as analog
  if (DACchannel== 1){ 
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
  }else if (DACchannel == 2){
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5; 
  }else {
	/* For MISRA-C compliance */
  }
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // DAC channel Configuration
  // DAC_DeInit();
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
  
  // DAC Channel 1 Init
  if (DACchannel == 1){
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);
  DAC_Cmd(DAC_Channel_1, ENABLE); 
  } else {
	/* For MISRA-C compliance */
  }

  // DAC Channel 2 Init
  if (DACchannel == 2){
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);
  DAC_Cmd(DAC_Channel_2, ENABLE);
  } else {
	/* For MISRA-C compliance */
  }

}

/************************************************************************************//**
** \brief     Updates the analog output value.
** \return    none.
**
****************************************************************************************/
void DAC_update( unsigned short DACvalue, unsigned short DACchannel )
{
if (DACchannel == 1)
	DAC_DHR12R1 = DACvalue;
else if (DACchannel == 2)
	DAC_DHR12R2 = DACvalue; //F4 name?? ->?DACC2DHR
else { 
	//Invalid channel
	ErrCodesSetError(ER_CODE_ANOUT_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
	}
    //DAC_DHR12RD = ((uint32_t) DAC2Value << 16 ) + DAC1Value; /* To set both channels at the same time, not used */
}
