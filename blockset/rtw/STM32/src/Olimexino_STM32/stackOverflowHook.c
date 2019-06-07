/************************************************************************************//**
* \file         stackOverflowHook.c
* \brief        Function to be called if stack overflow occurs.
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
#include "stm32f10x.h"                                /* STM32 registers               */
#include "stm32f10x_conf.h"                           /* STM32 peripheral drivers      */
#include "stm32f10x_gpio.h"							  /* GPIO drivers				   */
#include "FreeRTOS.h"								  /* Include FreeRTOS			   */
#include "task.h"									  /* For the xTaskHandle		   */

/************************************************************************************//**
** \brief     Sets an digital output high when stack overflow occurs
** \return    none.
**
****************************************************************************************/
void vApplicationStackOverflowHook( xTaskHandle xTask, signed char *pcTaskName )
{
  GPIO_InitTypeDef gpio_init;
  
  /* enable the port's peripheral clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  /* prepare pin configuration */
  gpio_init.GPIO_Pin = GPIO_Pin_10; /* PC10 is connected to D26 on the Olimexino */
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  /* initialize the pin */
  GPIO_Init(GPIOC, &gpio_init);
  
  /* set the new pin state */
  GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_SET);
  while(1==1){} /* Program gets stuck here */
} /*** end of vApplicationStackOverflowHook ***/


/************************************ end of stackOverflowHook.c ***********************************/