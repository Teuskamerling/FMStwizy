/************************************************************************************//**
* \file         quadencoder.c
* \brief        Quadrature encoder inputs driver source file.
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
#include "quadencoder.h"                              /* quadrature encoder driver     */
#include "os.h"                                       /* for operating system          */
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */
#include "stm32f4xx.h"                                /* STM32 registers               */
#include "stm32f4xx_conf.h"                           /* STM32 peripheral drivers      */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
#define QUADENC_TIMER_COUNT_MAX    (65535)
#define QUADENC_TIMER_COUNT_INIT   (32767)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief    Structure type with timer mapping information.
 *   \details A timer with quadrature encoder functionality always uses the timer
 *            channel 1 and channel 2 input pins. This structure defines which port pins
 *            are linked to a particular timer peripheral for this purposes.
 */
typedef struct
{
  TIM_TypeDef * timer_peripheral;
  uint32_t timer_peripheral_clk;
  uint8_t timer_apb_number;
  uint8_t timer_gpio_af;
  uint32_t timer_ch1_port_peripheral_clk;
  GPIO_TypeDef* timer_ch1_port;
  uint16_t timer_ch1_pin;
  uint16_t timer_ch1_pin_source;
  uint32_t timer_ch2_port_peripheral_clk;
  GPIO_TypeDef* timer_ch2_port;
  uint16_t timer_ch2_pin;
  uint16_t timer_ch2_pin_source;
} tQuadEncTimerMapping;


/****************************************************************************************
* Constant data declarations
****************************************************************************************/
/** \brief Array with all configuration parameters of a quadrature encoder module. */
const static tQuadEncTimerMapping timerMapping[] =
{
  { TIM1, RCC_APB2Periph_TIM1, 2, GPIO_AF_TIM1, RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_9,  GPIO_PinSource9,  RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_11, GPIO_PinSource11 },
  { TIM3, RCC_APB1Periph_TIM3, 1, GPIO_AF_TIM3, RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_6,  GPIO_PinSource6,  RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_5,  GPIO_PinSource5  },
  { TIM4, RCC_APB1Periph_TIM4, 1, GPIO_AF_TIM4, RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_12, GPIO_PinSource12, RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_13, GPIO_PinSource13 },
  { TIM8, RCC_APB2Periph_TIM8, 2, GPIO_AF_TIM8, RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_6,  GPIO_PinSource6,  RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_7,  GPIO_PinSource7  }
};

/** \brief Array with the signed 32-bit result counters. */
static int32_t quadEncCounters[sizeof(timerMapping)/sizeof(timerMapping[0])];

/** \brief Array to determine if a specific quadrature encoder module is in use. */
static uint8_t quadEncInitialized[sizeof(timerMapping)/sizeof(timerMapping[0])] =
{
  FALSE,
  FALSE,
  FALSE,
  FALSE
};


/************************************************************************************//**
** \brief     Configures a quadrature encoder module.
** \param     id Quad encoder module identifier.
** \param     config Pin configuration (QUADENC_CFG_xxx).
** \return    none.
**
****************************************************************************************/
void QuadEncConfigure(uint8_t id, tQuadEncCfg config)
{
	  GPIO_InitTypeDef gpio_init;
	  TIM_TimeBaseInitTypeDef timer_init;

	  /* make sure the id is valid before using it as an array indexer */
	  if (!(id < sizeof(timerMapping)/sizeof(timerMapping[0])))
	  {
	    ErrCodesSetError(ER_CODE_QUADENC_INVALID_MODULE, ER_PARAM_SEVERITY_CRITICAL, TRUE);
	  }
	  /* turn on the clocks for each of the ports needed and the alternate functions */
	  RCC_AHB1PeriphClockCmd(timerMapping[id].timer_ch1_port_peripheral_clk, ENABLE);
	  RCC_AHB1PeriphClockCmd(timerMapping[id].timer_ch2_port_peripheral_clk, ENABLE);
	  /* configure the timer channel 1 and 2 pins */
	  gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
	  gpio_init.GPIO_Mode = GPIO_Mode_AF;
	  if (config == QUADENC_CFG_FLOATING)
	  {
        gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  }
	  else if (config == QUADENC_CFG_PULLUP)
	  {
        gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
	  }
	  else
	  {
	        gpio_init.GPIO_PuPd = GPIO_PuPd_DOWN;
	  }
	  gpio_init.GPIO_Pin = timerMapping[id].timer_ch1_pin;
	  GPIO_Init(timerMapping[id].timer_ch1_port, &gpio_init);
	  gpio_init.GPIO_Pin = timerMapping[id].timer_ch2_pin;
	  GPIO_Init(timerMapping[id].timer_ch2_port, &gpio_init);
	  /* connect the pins to their timer module alternate function */
	  GPIO_PinAFConfig(timerMapping[id].timer_ch1_port, timerMapping[id].timer_ch1_pin_source, timerMapping[id].timer_gpio_af);
    GPIO_PinAFConfig(timerMapping[id].timer_ch2_port, timerMapping[id].timer_ch2_pin_source, timerMapping[id].timer_gpio_af);
	  /* turn on the timer clock */
	  if (timerMapping[id].timer_apb_number == 1)
	  {
        RCC_APB1PeriphClockCmd(timerMapping[id].timer_peripheral_clk, ENABLE);
	  }
	  else
	  {
	    RCC_APB2PeriphClockCmd(timerMapping[id].timer_peripheral_clk, ENABLE);
	  }

	  /* configure the timer */
	  timer_init.TIM_Prescaler = 0;
	  timer_init.TIM_Period = QUADENC_TIMER_COUNT_MAX;
	  timer_init.TIM_ClockDivision = TIM_CKD_DIV1;
	  timer_init.TIM_CounterMode = TIM_CounterMode_Up;
	  timer_init.TIM_RepetitionCounter = 0;

	  TIM_TimeBaseInit(timerMapping[id].timer_peripheral, &timer_init);
	  /* set them up as encoder inputs. set both inputs to rising polarity to let it use both edges.
	   * this results in 4 counts per channel pulse period
	   */
	  TIM_EncoderInterfaceConfig(timerMapping[id].timer_peripheral, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	  /* must configure the auto reload value for correctly using the encoder interface */
	  TIM_SetAutoreload(timerMapping[id].timer_peripheral, QUADENC_TIMER_COUNT_MAX);
	  /* give the counter an initial value in the middle of the 16-bit range so that it can count up and down */
	  TIM_SetCounter(timerMapping[id].timer_peripheral, QUADENC_TIMER_COUNT_INIT);
	  /* enable the timer */
	  TIM_Cmd(timerMapping[id].timer_peripheral, ENABLE);
	  /* reset the counters */
	  quadEncCounters[id] = 0;
	  /* flags this module as initialized */
	  quadEncInitialized[id] = TRUE;
} /*** end of QuadEncConfigure ***/


/************************************************************************************//**
** \brief     Obtains the quadrature encoder counter value. it counts up for one
**            direction and down for the opposite direction.
** \param     id Quad encoder module identifier.
** \return    32-bit signed counter value.
**
****************************************************************************************/
int32_t QuadEncGetCounter(uint8_t id)
{
  /* make sure the id is valid before using it as an array indexer */
  if (!(id < sizeof(timerMapping)/sizeof(timerMapping[0])))
  {
    ErrCodesSetError(ER_CODE_QUADENC_INVALID_MODULE, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* return the result */
  return quadEncCounters[id];
} /*** end of QuadEncGetCounter ***/


/************************************************************************************//**
** \brief     Resets the quadrature encoder counter value to 0.
** \param     id Quad encoder module identifier.
** \return    none.
**
****************************************************************************************/
void QuadEncResetCounter(uint8_t id)
{
  /* make sure the id is valid before using it as an array indexer */
  if (!(id < sizeof(timerMapping)/sizeof(timerMapping[0])))
  {
    ErrCodesSetError(ER_CODE_QUADENC_INVALID_MODULE, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* reset the counter value */
  quadEncCounters[id] = 0;
} /*** end of QuadEncResetCounter ***/


/************************************************************************************//**
** \brief     Reads the 16-bit internal counters and stores it in a 32-bit counter to
**            have more range. This function should be called periodically at a rate fast
**            enough to make sure the 16-bit internal counters don't overflow.
** \return    none.
**
****************************************************************************************/
void QuadEncUpdate(void)
{
  uint8_t idx;
  uint16_t internalCounterCurrent;
  uint16_t deltaCount;

  /* loop through all supported modules */
  for (idx=0; idx<(sizeof(timerMapping)/sizeof(timerMapping[0])); idx++)
  {
    if (quadEncInitialized[idx] == TRUE)
    {
      /* read the current value */
      internalCounterCurrent = TIM_GetCounter(timerMapping[idx].timer_peripheral);
      /* reset back into the initial value */
      TIM_SetCounter(timerMapping[idx].timer_peripheral, QUADENC_TIMER_COUNT_INIT);
      /* counting up? */
      if (internalCounterCurrent > QUADENC_TIMER_COUNT_INIT)
      {
        /* determine delta counts since last call */
        deltaCount = (uint16_t)(internalCounterCurrent - QUADENC_TIMER_COUNT_INIT);
        /* update 32-bit the counter value */
        quadEncCounters[idx] += deltaCount;
      }
      /* counting down */
      else if (internalCounterCurrent < QUADENC_TIMER_COUNT_INIT)
      {
        /* determine delta counts since last call */
        deltaCount = (uint16_t)(QUADENC_TIMER_COUNT_INIT - internalCounterCurrent);
        /* update 32-bit the counter value */
        quadEncCounters[idx] -= deltaCount;
      }
    }
  }
} /*** end of QuadEncUpdate ***/


/************************************ end of quadencoder.c *****************************/


