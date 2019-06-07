/************************************************************************************//**
* \file         timein.c
* \brief        Timer inputs driver source file.
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
#include "timein.h"                                   /* Timer input driver            */
#include "os.h"                                       /* for operating system          */
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */
#include "stm32f4xx.h"                                /* STM32 registers               */
#include "stm32f4xx_conf.h"                           /* STM32 peripheral drivers      */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/* timer module identifiers. note that these are used as array indexers so make
 * sure the first one has value 0 and the others are increments.
 */
#define TIMEIN_MODULE_TIM1    (0)                     /* Timer module 1                */
#define TIMEIN_MODULE_TIM2    (1)                     /* Timer module 2                */
#define TIMEIN_MODULE_TIM3    (2)                     /* Timer module 3                */
#define TIMEIN_MODULE_TIM4    (3)                     /* Timer module 4                */
#define TIMEIN_MODULE_TIM8    (4)                     /* Timer module 8                */
#define TIMEIN_MODULE_TIM9    (5)                     /* Timer module 9                */
#define TIMEIN_MODULE_TIM10   (6)                     /* Timer module 10               */
#define TIMEIN_MODULE_TIM11   (7)                     /* Timer module 11               */
#define TIMEIN_MODULE_TIM13   (8)                     /* Timer module 13               */
#define TIMEIN_MODULE_TIM14   (9)                     /* Timer module 14               */

/** \brief timer interrupt priority */
#define TIMER_NVIC_IRQ_PRIO             (configLIBRARY_KERNEL_INTERRUPT_PRIORITY)

/** \brief base frequency for the timer modules free running counter */
#define TIMER_COUNTER_FREQUENCY         (21000000)

/** \brief Sets the number of input capture channels for a timer module. This is
 *         hardware defined.
 */
#define TIMER_MAX_CHANNELS_PER_MODULE   (4)

/** \brief Scaling factor for the duty cycle. It is a percentage so normally only in the
 *         range 0..100. Adding a scaling factor increases the resolution.
 */
#define TIMER_DUTY_CYCLE_SCALING        (100)

/** \brief Determines the number of counts or the free running counter. */
#define TIMER_PERIOD_COUNTS             (65536)

/** \brief Macro for easy access to the maximum number of supported timer modules. */
#define TIMER_MODULES_MAX               (sizeof(moduleMapping)/sizeof(moduleMapping[0]))


/****************************************************************************************
* Type definitions
****************************************************************************************/
/**< \brief Structure type with module mapping information. */
typedef struct
{
  TIM_TypeDef * timer_peripheral;
  uint32_t timer_peripheral_clk;
  uint8_t timer_apb_number;
  uint8_t timer_prescale_divider;
  uint8_t timer_nvic_channel;
} tTimeinModuleMapping;

/**< \brief Structure type with pin mapping information. */
typedef struct
{
  uint32_t peripheral_clk;
  GPIO_TypeDef* port;
  uint16_t pin;
  uint8_t module_idx;
  uint8_t channel_idx;
  uint16_t capture_irq_flg;
  uint16_t polarity_bit;
  uint8_t gpio_af;
  uint16_t pin_source;
} tTimeinPinMapping;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void TimeinInit(uint8_t module_id);
static void TimeinOverflowInterrupt(uint8_t module_id);
static void TimeinCaptureInterrupt(uint8_t pin_id);


/****************************************************************************************
* Constant data declarations
****************************************************************************************/
/** \brief Array with all configuration parameters of a timer module. note that
 *         TIM2-7 and TIM12-14 have a max input clock frequency of 84 Mhz as opposed
 *         to 168 MHz so these modules need an extra prescale divider.
 */
const static tTimeinModuleMapping moduleMapping[] =
{
  { TIM1,  RCC_APB2Periph_TIM1,  2, 1, TIM1_CC_IRQn            }, /* idx 0: TIMEIN_MODULE_TIM1  */
  { TIM2,  RCC_APB1Periph_TIM2,  1, 2, TIM2_IRQn               }, /* idx 1: TIMEIN_MODULE_TIM2  */
  { TIM3,  RCC_APB1Periph_TIM3,  1, 2, TIM3_IRQn               }, /* idx 2: TIMEIN_MODULE_TIM3  */
  { TIM4,  RCC_APB1Periph_TIM4,  1, 2, TIM4_IRQn               }, /* idx 3: TIMEIN_MODULE_TIM4  */
  { TIM8,  RCC_APB2Periph_TIM8,  2, 1, TIM8_CC_IRQn            }, /* idx 4: TIMEIN_MODULE_TIM8  */
  { TIM9,  RCC_APB2Periph_TIM9,  2, 1, TIM1_BRK_TIM9_IRQn      }, /* idx 5: TIMEIN_MODULE_TIM9  */
  { TIM10, RCC_APB2Periph_TIM10, 2, 1, TIM1_UP_TIM10_IRQn      }, /* idx 6: TIMEIN_MODULE_TIM10 */
  { TIM11, RCC_APB2Periph_TIM11, 2, 1, TIM1_TRG_COM_TIM11_IRQn }, /* idx 7: TIMEIN_MODULE_TIM11 */
  { TIM13, RCC_APB1Periph_TIM13, 1, 2, TIM8_UP_TIM13_IRQn      }, /* idx 8: TIMEIN_MODULE_TIM13 */
  { TIM14, RCC_APB1Periph_TIM14, 1, 2, TIM8_TRG_COM_TIM14_IRQn }  /* idx 9: TIMEIN_MODULE_TIM14 */
};

/** \brief Array with all configuration parameters of an input capture pin. */
const static tTimeinPinMapping pinMapping[] =
{
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_9,  TIMEIN_MODULE_TIM1,  0, TIM_IT_CC1, TIM_CCER_CC1P, GPIO_AF_TIM1,  GPIO_PinSource9  }, /* idx 0:  TIMEIN_TIM1_PIN_PE9   */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_11, TIMEIN_MODULE_TIM1,  1, TIM_IT_CC2, TIM_CCER_CC2P, GPIO_AF_TIM1,  GPIO_PinSource11 }, /* idx 1:  TIMEIN_TIM1_PIN_PE11  */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_13, TIMEIN_MODULE_TIM1,  2, TIM_IT_CC3, TIM_CCER_CC3P, GPIO_AF_TIM1,  GPIO_PinSource13 }, /* idx 2:  TIMEIN_TIM1_PIN_PE13  */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_14, TIMEIN_MODULE_TIM1,  3, TIM_IT_CC4, TIM_CCER_CC4P, GPIO_AF_TIM1,  GPIO_PinSource14 }, /* idx 3:  TIMEIN_TIM1_PIN_PE14  */
  { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_5,  TIMEIN_MODULE_TIM2,  0, TIM_IT_CC1, TIM_CCER_CC1P, GPIO_AF_TIM2,  GPIO_PinSource5  }, /* idx 4:  TIMEIN_TIM2_PIN_PA5   */
  { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_6,  TIMEIN_MODULE_TIM3,  0, TIM_IT_CC1, TIM_CCER_CC1P, GPIO_AF_TIM3,  GPIO_PinSource6  }, /* idx 5:  TIMEIN_TIM3_PIN_PA6   */
  { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_5,  TIMEIN_MODULE_TIM3,  1, TIM_IT_CC2, TIM_CCER_CC2P, GPIO_AF_TIM3,  GPIO_PinSource5  }, /* idx 6:  TIMEIN_TIM3_PIN_PB5   */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_12, TIMEIN_MODULE_TIM4,  0, TIM_IT_CC1, TIM_CCER_CC1P, GPIO_AF_TIM4,  GPIO_PinSource12 }, /* idx 7:  TIMEIN_TIM4_PIN_PD12  */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_13, TIMEIN_MODULE_TIM4,  1, TIM_IT_CC2, TIM_CCER_CC2P, GPIO_AF_TIM4,  GPIO_PinSource13 }, /* idx 8:  TIMEIN_TIM4_PIN_PD13  */
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_6,  TIMEIN_MODULE_TIM8,  0, TIM_IT_CC1, TIM_CCER_CC1P, GPIO_AF_TIM8,  GPIO_PinSource6  }, /* idx 9:  TIMEIN_TIM8_PIN_PC6   */
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_7,  TIMEIN_MODULE_TIM8,  1, TIM_IT_CC2, TIM_CCER_CC2P, GPIO_AF_TIM8,  GPIO_PinSource7  }, /* idx 10: TIMEIN_TIM8_PIN_PC7   */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_5,  TIMEIN_MODULE_TIM9,  0, TIM_IT_CC1, TIM_CCER_CC1P, GPIO_AF_TIM9,  GPIO_PinSource5  }, /* idx 11: TIMEIN_TIM9_PIN_PE5   */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_6,  TIMEIN_MODULE_TIM9,  1, TIM_IT_CC2, TIM_CCER_CC2P, GPIO_AF_TIM9,  GPIO_PinSource6  }, /* idx 12: TIMEIN_TIM9_PIN_PE6   */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_6,  TIMEIN_MODULE_TIM10, 0, TIM_IT_CC1, TIM_CCER_CC1P, GPIO_AF_TIM10, GPIO_PinSource6  }, /* idx 13: TIMEIN_TIM10_PIN_PF6  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_7,  TIMEIN_MODULE_TIM11, 0, TIM_IT_CC1, TIM_CCER_CC1P, GPIO_AF_TIM11, GPIO_PinSource7  }, /* idx 14: TIMEIN_TIM11_PIN_PF7  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_8,  TIMEIN_MODULE_TIM13, 0, TIM_IT_CC1, TIM_CCER_CC1P, GPIO_AF_TIM13, GPIO_PinSource8  }, /* idx 15: TIMEIN_TIM13_PIN_PF8  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_9,  TIMEIN_MODULE_TIM14, 0, TIM_IT_CC1, TIM_CCER_CC1P, GPIO_AF_TIM14, GPIO_PinSource9  }  /* idx 16: TIMEIN_TIM14_PIN_PF9  */
};


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Function pointers for the user specified edge detected interrupt handler. */
static tTimeinCallbackEdgeDetected callbackEdgeDetected[TIMER_MODULES_MAX][TIMER_MAX_CHANNELS_PER_MODULE];

/** \brief Holds information about if a timer module's pin was already configured. */
static volatile uint8_t pin_configured[TIMER_MODULES_MAX][TIMER_MAX_CHANNELS_PER_MODULE];

/** \brief Holds information about if a timer module was already configured. */
static volatile uint8_t module_initialized[TIMER_MODULES_MAX] = { FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE };

/** \brief Holds the most recent calculated frequency for a input capture pin. */
static volatile uint32_t frequencies[TIMER_MODULES_MAX][TIMER_MAX_CHANNELS_PER_MODULE];

/** \brief Holds the most recent calculated duty cycle for a input capture pin. */
static volatile uint16_t duty_cycles[TIMER_MODULES_MAX][TIMER_MAX_CHANNELS_PER_MODULE];

/** \brief Holds the most recent counter value for a input capture pin. This software
 *         driver maintains a 32-bit free running counter for each timer module that
 *         starts at zero upon initialization. This is a timestamp value of this 32-bit
 *         free running counter from when the last edge was detected.
 */
static volatile uint32_t last_edge_count[TIMER_MODULES_MAX][TIMER_MAX_CHANNELS_PER_MODULE];

/** \brief Holds the state (high/low) of most recent detected edge of a input capture
 *         pin.
 */
static volatile uint8_t last_edge_high[TIMER_MODULES_MAX][TIMER_MAX_CHANNELS_PER_MODULE];

/** \brief Holds the number of overflows that happened on a timer module since it
 *         was initialized. This is needed to compute the 32-bit timestamp for
 *         last_edge_count[].
 */
static volatile uint16_t free_running_counter_overflows[TIMER_MODULES_MAX];

/** \brief Holds the number of edges that were detected for each pin since
 *         initialization
 */
static volatile uint32_t edge_counter[TIMER_MODULES_MAX][TIMER_MAX_CHANNELS_PER_MODULE];

/** \brief Holds the number of overflow that happened during a measurement on a
 *         specific input capture pin.
 */
static volatile uint16_t overflows[TIMER_MODULES_MAX][TIMER_MAX_CHANNELS_PER_MODULE];

/** \brief Flag to determine if a measurement on a specific input capture pin is
 *         active.
 */
static volatile uint8_t active[TIMER_MODULES_MAX][TIMER_MAX_CHANNELS_PER_MODULE];

/** \brief Flag to determine if a 0 Hz scenario was detected.
 */
static volatile uint8_t zero_hz_detected[TIMER_MODULES_MAX][TIMER_MAX_CHANNELS_PER_MODULE];

/** \brief Holds the number of overflows after which a zero Hz scenario is detected. */
static volatile uint16_t zero_hz_overflows[TIMER_MODULES_MAX][TIMER_MAX_CHANNELS_PER_MODULE];


/************************************************************************************//**
** \brief     Initializes the timer inputs driver for a specific timer module.
** \param     module_id Timer module identifier.
** \return    none.
**
****************************************************************************************/
static void TimeinInit(uint8_t module_id)
{
  NVIC_InitTypeDef  NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef tim_base;
  uint16_t prescaler;
  uint8_t i;

  /* make sure the id is valid before using it as an array indexer */
  if (!(module_id < sizeof(moduleMapping)/sizeof(moduleMapping[0])))
  {
    ErrCodesSetError(ER_CODE_TIMEIN_INVALID_MODULE, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* only initialize if not yet initialized */
  if (module_initialized[module_id] == TRUE)
  {
    /* nothing left to do because this module is already initialized */
    return;
  }

  /* reset the zero Hz detection flags. note that this cannot be done in TimeinConfigure
   * because this function is called upon zero Hz detection and it would incorrectly
   * reset this flag then. Also reset the callback function pointers for all channels.
   */
  for (i=0; i<TIMER_MAX_CHANNELS_PER_MODULE; i++)
  {
    zero_hz_detected[module_id][i] = FALSE;
    zero_hz_overflows[module_id][i] = 0;
    callbackEdgeDetected[module_id][i] = NULL;
    pin_configured[module_id][i] = FALSE;
  }
  /* turn on the timer clock */
  if (moduleMapping[module_id].timer_apb_number == 1)
  {
    RCC_APB1PeriphClockCmd(moduleMapping[module_id].timer_peripheral_clk, ENABLE);
  }
  else
  {
    RCC_APB2PeriphClockCmd(moduleMapping[module_id].timer_peripheral_clk, ENABLE);
  }
  /* enable the timer module global interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = moduleMapping[module_id].timer_nvic_channel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMER_NVIC_IRQ_PRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* timer 1 is special as it has separate IRQ channels */
  if (moduleMapping[module_id].timer_peripheral == TIM1)
  {
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMER_NVIC_IRQ_PRIO;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  }
  /* timer 8 is special as it has separate IRQ channels */
  if (moduleMapping[module_id].timer_peripheral == TIM8)
  {
    NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMER_NVIC_IRQ_PRIO;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  }
  /* reset the free running counter overflows variable */
  free_running_counter_overflows[module_id] = 0;
  /* compute the prescaler value */
  prescaler = (SystemCoreClock / (TIMER_COUNTER_FREQUENCY * moduleMapping[module_id].timer_prescale_divider  )) - 1;
  /* configure the timer module and enable it */
  tim_base.TIM_Period = (TIMER_PERIOD_COUNTS - 1);
  tim_base.TIM_Prescaler = prescaler;
  tim_base.TIM_ClockDivision = 0;
  tim_base.TIM_CounterMode = TIM_CounterMode_Up;
  tim_base.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(moduleMapping[module_id].timer_peripheral, &tim_base);
  TIM_ARRPreloadConfig(moduleMapping[module_id].timer_peripheral, ENABLE);
  TIM_SetCounter(moduleMapping[module_id].timer_peripheral, 0);
  TIM_Cmd(moduleMapping[module_id].timer_peripheral, ENABLE);
  /* enable the counter overflow interrupt request for the module */
  TIM_ITConfig(moduleMapping[module_id].timer_peripheral, TIM_IT_Update, ENABLE);
  /* set module to initialized */
  module_initialized[module_id] = TRUE;
} /*** end of TimeinInit ***/


/************************************************************************************//**
** \brief     Configures a timer module channel pin for input capture operation.
** \param     pin_id Pin identifier.
** \return    none.
**
****************************************************************************************/
void TimeinConfigure(uint8_t pin_id)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;

  /* make sure the id is valid before using it as an array indexer */
  if (!(pin_id < sizeof(pinMapping)/sizeof(pinMapping[0])))
  {
    ErrCodesSetError(ER_CODE_TIMEIN_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* make sure the timer module that this pin belongs to is initialized */
  TimeinInit(pinMapping[pin_id].module_idx);

  /* only configure if not yet configured */
  if (pin_configured[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] == TRUE)
  {
    /* nothing left to do because this pin is already configured */
    return;
  }
  /* set channel to inactive by default */
  active[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = FALSE;
  /* initialize locals for this channel */
  last_edge_high[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = FALSE;
  edge_counter[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = 0;
  frequencies[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = 0;
  duty_cycles[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = 0;
  /* enable the port's peripheral clock */
  RCC_AHB1PeriphClockCmd(pinMapping[pin_id].peripheral_clk, ENABLE);
  /* initialize the GPIO pin */
  GPIO_InitStructure.GPIO_Pin =  pinMapping[pin_id].pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(pinMapping[pin_id].port, &GPIO_InitStructure);
  /* connect the pin to it's timer module alternate function */
  GPIO_PinAFConfig(pinMapping[pin_id].port, pinMapping[pin_id].pin_source, pinMapping[pin_id].gpio_af);
  /* configure the input capture channel */
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  /* make sure to configure the correct channel */
  if (pinMapping[pin_id].channel_idx == 0)
  {
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, &TIM_ICInitStructure);
    /* enable the input capture interrupt request for the channel */
    TIM_ITConfig(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, TIM_IT_CC1, ENABLE);
  }
  else if (pinMapping[pin_id].channel_idx == 1)
  {
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, &TIM_ICInitStructure);
    /* enable the input capture interrupt request for the channel */
    TIM_ITConfig(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, TIM_IT_CC2, ENABLE);
  }
  else if (pinMapping[pin_id].channel_idx == 2)
  {
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
    TIM_ICInit(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, &TIM_ICInitStructure);
    /* enable the input capture interrupt request for the channel */
    TIM_ITConfig(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, TIM_IT_CC3, ENABLE);
  }
  else if (pinMapping[pin_id].channel_idx == 3)
  {
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
    TIM_ICInit(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, &TIM_ICInitStructure);
    /* enable the input capture interrupt request for the channel */
    TIM_ITConfig(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, TIM_IT_CC4, ENABLE);
  }
  /* set pin to configured */
  pin_configured[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = TRUE;
} /*** end of TimeinConfigure ***/


/************************************************************************************//**
** \brief     Configures a timer module channel pin's zero Hz detection timeout.
** \param     pin_id Pin identifier.
** \param     zero_hz_timeout_ms Number of milliseconds after which the zero Hz detection
**                               flag should be set in case no edge is detected during
**                               this time. set this value to zero to not use this
**                               feature.
** \return    none.
**
****************************************************************************************/
void TimeinConfigureZeroHzTimeout(uint8_t pin_id, uint16_t zero_hz_timeout_ms)
{
  uint32_t overflow_time_us;

  /* convert zero Hz detection time to overflows */
  if (zero_hz_timeout_ms > 0)
  {
    /* compute how many microseconds go in one overflow */
    overflow_time_us = TIMER_PERIOD_COUNTS/(TIMER_COUNTER_FREQUENCY/1000000); //2730
    /* configure the number of overflow for the zero Hz timeout time */
    zero_hz_overflows[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = (zero_hz_timeout_ms*1000)/overflow_time_us;
  }
  else
  {
    /* set it to zero to indicate that the feature is not used */
    zero_hz_overflows[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = 0;
  }
} /*** end of TimeinConfigureZeroHzTimeout ***/


/************************************************************************************//**
** \brief     Registers the callback handler that gets called by the driver each time
**            an edge was detected in the input pin. Note that this callback is called
**            on interrupt level.
** \param     pin_id Pin identifier.
** \param     callbackPtr Function pointer to the callback.
** \return    none.
**
****************************************************************************************/
void TimeinRegisterEdgeDetectedCallback(uint8_t pin_id, tTimeinCallbackEdgeDetected callbackPtr)
{
  /* make sure the id is valid before using it as an array indexer */
  if (!(pin_id < sizeof(pinMapping)/sizeof(pinMapping[0])))
  {
    ErrCodesSetError(ER_CODE_TIMEIN_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* store the callback function pointer */
  callbackEdgeDetected[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = callbackPtr;
} /*** end of TimeinRegisterEdgeDetectedCallback ***/


/************************************************************************************//**
** \brief     Obtains the last measured frequency for a specific timer channel.
** \param     pin_id Pin identifier.
** \return    Frequency in Hz.
**
****************************************************************************************/
uint32_t TimeinGetFrequency(uint8_t pin_id)
{
  /* make sure the id is valid before using it as an array indexer */
  if (!(pin_id < sizeof(pinMapping)/sizeof(pinMapping[0])))
  {
    ErrCodesSetError(ER_CODE_TIMEIN_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* return the value */
  return frequencies[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx];
} /*** end of TimeinGetFrequency ***/


/************************************************************************************//**
** \brief     Obtains the last measured duty cycle for a specific timer channel.
** \param     pin_id Pin identifier.
** \return    Duty cycle in %.
**
****************************************************************************************/
uint16_t TimeinGetDutyCycle(uint8_t pin_id)
{
  /* make sure the id is valid before using it as an array indexer */
  if (!(pin_id < sizeof(pinMapping)/sizeof(pinMapping[0])))
  {
    ErrCodesSetError(ER_CODE_TIMEIN_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* return the value */
  return duty_cycles[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx];
} /*** end of TimeinGetDutyCycle ***/


/************************************************************************************//**
** \brief     Obtains the last detected edge count for a specific timer channel. This is
**            a timestamp value of the timer module's 32-bit free running counter that
**            is maintained by this driver and runs at a speed of TIMER_COUNTER_FREQUENCY.
** \param     pin_id Pin identifier.
** \return    Edge timestamp value.
**
****************************************************************************************/
uint32_t TimeinGetLastEdgeCount(uint8_t pin_id)
{
  /* return the value */
  return last_edge_count[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx];
} /*** end of TimeinGetLastEdgeCount ***/


/************************************************************************************//**
** \brief     Obtains the state of the last detected edge timer channel.
** \param     pin_id Pin identifier.
** \return    TRUE is the last edge was a rising edge, FALSE is the last edge was a
**            falling edge.
**
****************************************************************************************/
uint8_t TimeinGetLastEdgeState(uint8_t pin_id)
{
  /* return the value */
  return last_edge_high[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx];
} /*** end of TimeinGetLastEdgeState ***/


/************************************************************************************//**
** \brief     Obtains the current value of the timer module's 32-bit free running counter
**            that is maintained by this driver and runs at a speed of
**            TIMER_COUNTER_FREQUENCY.
** \param     pin_id Pin identifier.
** \return    counter value.
**
****************************************************************************************/
uint32_t TimeinGetFreeRunningCounter(uint8_t pin_id)
{
  uint16_t free_running_counter_overflows_cpy;
  uint16_t free_running_counter_cpy;
  uint32_t saved_cs_state;

  /* read values for computing the 32-bit free running counter as a critical section */
  saved_cs_state = OsEnterCriticalSection();
  free_running_counter_overflows_cpy = free_running_counter_overflows[pinMapping[pin_id].module_idx];
  free_running_counter_cpy = TIM_GetCounter(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral);
  OsLeaveCriticalSection(saved_cs_state);
  /* compute and return the 32-bit free running counter */
  return ((free_running_counter_overflows_cpy * TIMER_PERIOD_COUNTS) + free_running_counter_cpy);
} /*** end of TimeinGetFreeRunningCounter ***/


/************************************************************************************//**
** \brief     Obtains the number of edges that were detected since initialization.
** \param     pin_id Pin identifier.
** \return    Edge counter value.
**
****************************************************************************************/
uint32_t TimeinGetEdgeCounter(uint8_t pin_id)
{
  /* return the value */
  return edge_counter[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx];
} /*** end of TimeinGetEdgeCounter ***/


/************************************************************************************//**
** \brief     Determines is the zero Hz situation was detected or not. Note that if this
**            flag is set, if will automatically be reset by the driver upon detection
**            of the next edge.
** \param     pin_id Pin identifier.
** \return    TRUE is zero Hz was detected, FALSE otherwise.
**
****************************************************************************************/
uint8_t TimeinIsZeroHzDetected(uint8_t pin_id)
{
  /* return the value */
  return zero_hz_detected[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx];
} /*** end of TimeinIsZeroHzDetected ***/


/************************************************************************************//**
** \brief     Similar to the TIM_ClearITPendingBit function in the peripheral driver
**            library with the only modification that is uses bit-banding to clear the
**            interrupt flags. The problem with the original function is that the
**            bit clearing is not done atomically and therefore it can happen in rare
**            cases that during a flag clear operation of the updated event, an input
**            capture flag is set and unfortunately cleared as well. This would still
**            cause in interrupt but without the correct event flag set.
** \param     TIMx: where x can be 1 to 17 to select the TIM peripheral.
** \param     TIM_IT: specifies the pending bit to clear.
** \return    none.
**
****************************************************************************************/
void TimeinClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT)
{
  /* macro to convert PERI address to the bit-band address. it takes the base register
   * address as 'a' and the bit-number as 'b'. note that the bit-number starts at 0,
   * so it is not the same as the bit-mask.
   */
  #define BITBAND_PERI(a,b) ((PERIPH_BB_BASE + (a-PERIPH_BASE)*32 + (b*4)))
  /* macro to convert a bit-mask to a bit-number */
  #define BITNR(x) (x&0x1?0:x&0x2?1:x&0x4?2:x&0x8?3: \
                    x&0x10?4:x&0x20?5:x&0x40?6:x&0x80?7: \
                    x&0x100?8:x&0x200?9:x&0x400?10:x&0x800?11: \
                    x&0x1000?12:x&0x2000?13:x&0x4000?14:x&0x8000?15: \
                    x&0x10000?16:x&0x20000?17:x&0x40000?18:x&0x80000?19: \
                    x&0x100000?20:x&0x200000?21:x&0x400000?22:x&0x800000?23: \
                    x&0x1000000?24:x&0x2000000?25:x&0x4000000?26:x&0x8000000?27: \
                    x&0x10000000?28:x&0x20000000?29:x&0x40000000?30:x&0x80000000?31:32)
  /* macro that can be used to directly write a 1 or 0 to a bit in the bit-banding
   * address space.
   */
  #define REGBIT(regaddr, bitmask) (*(uint32_t volatile*)(BITBAND_PERI((uint32_t)regaddr,BITNR(bitmask))))

  /* clear the IT pending bit using bit-banding */
  REGBIT(&(TIMx->SR), TIM_IT) = 0;
} /*** end of TimeinClearITPendingBit ***/



/************************************************************************************//**
** \brief     Generic interrupt service routine handler for the free running counter
**            overflow interrupt event.
** \param     module_id Timer module identifier.
** \return    none.
**
****************************************************************************************/
static void TimeinOverflowInterrupt(uint8_t module_id)
{
  uint8_t cnt;

  /* this ISR is shared with others, so make sure this module is configured before
   * processing this interrupt.
   */
  if (module_initialized[module_id] == FALSE)
  {
    /* interrupt not for us */
    return;
  }

  /* clear timer counter overflow event flag for the channel of the event */
  TimeinClearITPendingBit(moduleMapping[module_id].timer_peripheral, TIM_IT_Update);
  /* increment the free running counter overflow counter */
  free_running_counter_overflows[module_id]++;
  /* update the overflow counter for the channels */
  for (cnt=0; cnt<TIMER_MAX_CHANNELS_PER_MODULE; cnt++)
  {
    overflows[module_id][cnt]++;
    /* run zero Hz detection if a measurement is active and this feature is active */
    if ( (zero_hz_overflows[module_id][cnt] > 0) && (active[module_id][cnt] == TRUE) )
    {
      /* zero Hz detected? */
      if (overflows[module_id][cnt] >= zero_hz_overflows[module_id][cnt])
      {
        /* set flag */
        zero_hz_detected[module_id][cnt] = TRUE;
        /* set channel to inactive */
        active[module_id][cnt] = FALSE;
        frequencies[module_id][cnt] = 0;
        duty_cycles[module_id][cnt] = 0;
      }
    }
  }
} /*** end of TimeinOverflowInterrupt ***/


/************************************************************************************//**
** \brief     Generic interrupt service routine handler for the input capture interrupt
**            event.
** \param     pin_id Pin identifier.
** \return    none.
**
****************************************************************************************/
static void TimeinCaptureInterrupt(uint8_t pin_id)
{
  uint16_t edge_cnt;
  uint32_t period_cnt;
  static uint32_t duty_cnt[TIMER_MODULES_MAX][TIMER_MAX_CHANNELS_PER_MODULE];
  static uint16_t rising_edge_cnt_prev[TIMER_MODULES_MAX][TIMER_MAX_CHANNELS_PER_MODULE];

  /* this ISR is shared with others, so make sure this pin is configured before
   * processing this interrupt.
   */
  if (pin_configured[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] == FALSE)
  {
    /* interrupt not for us */
    return;
  }

  /* clear timer module's input capture flag for the channel of the event */
  TimeinClearITPendingBit(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, pinMapping[pin_id].capture_irq_flg);
  /* increment the edge counter */
  edge_counter[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx]++;
  /* store the capture counter value of this edge */
  if (pinMapping[pin_id].channel_idx == 0)
  {
    edge_cnt = TIM_GetCapture1(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral);
  }
  else if (pinMapping[pin_id].channel_idx == 1)
  {
    edge_cnt = TIM_GetCapture2(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral);
  }
  else if (pinMapping[pin_id].channel_idx == 2)
  {
    edge_cnt = TIM_GetCapture3(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral);
  }
  else if (pinMapping[pin_id].channel_idx == 3)
  {
    edge_cnt = TIM_GetCapture4(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral);
  }
  /* edge detected so reset the zero Hz detection flag  */
  zero_hz_detected[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = FALSE;
  /* store this edge counter value */
  last_edge_count[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = edge_cnt + (free_running_counter_overflows[pinMapping[pin_id].module_idx] * TIMER_PERIOD_COUNTS);
  /* was this a rising edge? */
  if ((moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral->CCER & pinMapping[pin_id].polarity_bit) == 0)
  {
    /* store the edge state */
    last_edge_high[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = TRUE;
    /* reconfigure for falling edge trigger */
    moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral->CCER |= pinMapping[pin_id].polarity_bit;
    /* was this the first edge so the start of the signal? */
    if (active[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] == FALSE)
    {
      /* signal detected so set it to active */
      active[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = TRUE;
    }
    else
    {
      /* determine the counter for the measured period */
      if (edge_cnt > rising_edge_cnt_prev[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx])
      {
        period_cnt = (edge_cnt - rising_edge_cnt_prev[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx]) + \
                     (overflows[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] * TIMER_PERIOD_COUNTS);
      }
      else
      {
        period_cnt = ((0xFFFF - rising_edge_cnt_prev[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx]) + \
                     edge_cnt) + ((overflows[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx]-1) * TIMER_PERIOD_COUNTS);
      }
      /* calculate the frequency */
      frequencies[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = (uint32_t) TIMER_COUNTER_FREQUENCY / period_cnt;
      /* calculate the duty cycle as a percentage. note that for low frequency signals
       * when duty_cnt is > (2^32 - 1) / (100 * TIMER_DUTY_CYCLE_SCALING), the
       * duty cycle calculation can overflow. in this case an optimized calculation
       * should be used. This optimization allows duty cycle measurements for
       * signals >= 1Hz, which is fine because this is also the lowest measurable
       * frequency by this module.
       */
      if (duty_cnt[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] >= 429496)
      {
        duty_cycles[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = (duty_cnt[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] * \
                                                                                     TIMER_DUTY_CYCLE_SCALING) / (period_cnt / 100);
      }
      else
      {
        duty_cycles[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = (duty_cnt[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] * \
                                                                                     100 * TIMER_DUTY_CYCLE_SCALING) / period_cnt;
      }
    }
    /* reset the overflow counter for the next period */
    overflows[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = 0;
    /* store the capture counter for the next iteration */
    rising_edge_cnt_prev[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = edge_cnt;
  }
  /* is must have been a falling edge */
  else
  {
    /* store the edge state */
    last_edge_high[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = FALSE;
    /* reconfigure for rising edge trigger */
    moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral->CCER &= ~pinMapping[pin_id].polarity_bit;
    /* only run duty cycle computation if a measurement is active */
    if (active[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] == TRUE)
    {
      /* determine the counter for the measured period */
      if (edge_cnt > rising_edge_cnt_prev[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx])
      {
        duty_cnt[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = (edge_cnt - rising_edge_cnt_prev[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx]) + \
                                                                                  (overflows[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] * TIMER_PERIOD_COUNTS);
      }
      else
      {
        duty_cnt[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = ((0xFFFF - rising_edge_cnt_prev[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx]) + \
                                                                                  edge_cnt) + ((overflows[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx]-1) * TIMER_PERIOD_COUNTS);
      }
    }
  }
  /* invoke callback to pass on edge detected event */
  if (callbackEdgeDetected[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] != NULL)
  {
    callbackEdgeDetected[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx]();
  }
} /*** end of TimeinOverflowInterrupt ***/



/************************************************************************************//**
** \brief     Interrupt service routine for timer update channel 1 and the timer 10
**            interrupt channel.
** \return    none.
**
****************************************************************************************/
void Timein1UpdateTimein10Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
  {
    /* process the event */
    TimeinOverflowInterrupt(TIMEIN_MODULE_TIM1);
  }
  else
  {
    /* --------------- was it a counter overflow event? -------------------------------- */
    if(TIM_GetITStatus(TIM10, TIM_IT_Update) == SET)
    {
      /* process the event */
      TimeinOverflowInterrupt(TIMEIN_MODULE_TIM10);
    }
    else
    {
      /* --------------- was it an input capture event for the 1st channel? -------------- */
      if(TIM_GetITStatus(TIM10, TIM_IT_CC1) == SET)
      {
        /* process the event */
        TimeinCaptureInterrupt(TIMEIN_TIM10_PIN_PF6);
      }
    }
  }
} /*** end of Timein1UpdateTimein10Interrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer capture channel 1.
** \return    none.
**
****************************************************************************************/
void Timein1CaptureInterrupt(void)
{
  /* --------------- was it an input capture event for the 1st channel? -------------- */
  if(TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET)
  {
    /* process the event */
    TimeinCaptureInterrupt(TIMEIN_TIM1_PIN_PE9);
  }
  /* --------------- was it an input capture event for the 2nd channel? -------------- */
  else if(TIM_GetITStatus(TIM1, TIM_IT_CC2) == SET)
  {
    /* process the event */
    TimeinCaptureInterrupt(TIMEIN_TIM1_PIN_PE11);
  }
  /* --------------- was it an input capture event for the 3rd channel? -------------- */
  else if(TIM_GetITStatus(TIM1, TIM_IT_CC3) == SET)
  {
    /* process the event */
    TimeinCaptureInterrupt(TIMEIN_TIM1_PIN_PE13);
  }
  /* --------------- was it an input capture event for the 4th channel? -------------- */
  else if(TIM_GetITStatus(TIM1, TIM_IT_CC4) == SET)
  {
    /* process the event */
    TimeinCaptureInterrupt(TIMEIN_TIM1_PIN_PE14);
  }
} /*** end of Timein1CaptureInterrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer channel 2.
** \return    none.
**
****************************************************************************************/
void Timein2Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
  {
    /* process the event */
    TimeinOverflowInterrupt(TIMEIN_MODULE_TIM2);
  }
  else
  {
    /* --------------- was it an input capture event for the 1st channel? ------------ */
    if(TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET)
    {
      /* process the event */
      TimeinCaptureInterrupt(TIMEIN_TIM2_PIN_PA5);
    }
  }
} /*** end of Timein2Interrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer channel 3.
** \return    none.
**
****************************************************************************************/
void Timein3Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
  {
    /* process the event */
    TimeinOverflowInterrupt(TIMEIN_MODULE_TIM3);
  }
  else
  {
    /* --------------- was it an input capture event for the 1st channel? ------------ */
    if(TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET)
    {
      /* process the event */
      TimeinCaptureInterrupt(TIMEIN_TIM3_PIN_PA6);
    }
    /* --------------- was it an input capture event for the 2nd channel? ------------ */
    else if(TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET)
    {
      /* process the event */
      TimeinCaptureInterrupt(TIMEIN_TIM3_PIN_PB5);
    }
  }
} /*** end of Timein3Interrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer channel 4.
** \return    none.
**
****************************************************************************************/
void Timein4Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
  {
    /* process the event */
    TimeinOverflowInterrupt(TIMEIN_MODULE_TIM4);
  }
  else
  {
    /* --------------- was it an input capture event for the 1st channel? ------------ */
    if(TIM_GetITStatus(TIM4, TIM_IT_CC1) == SET)
    {
      /* process the event */
      TimeinCaptureInterrupt(TIMEIN_TIM4_PIN_PD12);
    }
    /* --------------- was it an input capture event for the 2nd channel? ------------ */
    else if(TIM_GetITStatus(TIM4, TIM_IT_CC2) == SET)
    {
      /* process the event */
      TimeinCaptureInterrupt(TIMEIN_TIM4_PIN_PD13);
    }
  }
} /*** end of Timein4Interrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer update channel 8 and the timer 13
**            interrupt channel.
** \return    none.
**
****************************************************************************************/
void Timein8UpdateTimein13Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM8, TIM_IT_Update) == SET)
  {
    /* process the event */
    TimeinOverflowInterrupt(TIMEIN_MODULE_TIM8);
  }
  else
  {
    /* --------------- was it a counter overflow event? ------------------------------ */
    if(TIM_GetITStatus(TIM13, TIM_IT_Update) == SET)
    {
      /* process the event */
      TimeinOverflowInterrupt(TIMEIN_MODULE_TIM13);
    }
    else
    {
      /* --------------- was it an input capture event for the 1st channel? ---------- */
      if(TIM_GetITStatus(TIM13, TIM_IT_CC1) == SET)
      {
        /* process the event */
        TimeinCaptureInterrupt(TIMEIN_TIM13_PIN_PF8);
      }
    }
  }
} /*** end of Timein8UpdateTimein13Interrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer capture channel 8.
** \return    none.
**
****************************************************************************************/
void Timein8CaptureInterrupt(void)
{
  /* --------------- was it an input capture event for the 1st channel? -------------- */
  if(TIM_GetITStatus(TIM8, TIM_IT_CC1) == SET)
  {
    /* process the event */
    TimeinCaptureInterrupt(TIMEIN_TIM8_PIN_PC6);
  }
  /* --------------- was it an input capture event for the 2nd channel? -------------- */
  else if(TIM_GetITStatus(TIM8, TIM_IT_CC2) == SET)
  {
    /* process the event */
    TimeinCaptureInterrupt(TIMEIN_TIM8_PIN_PC7);
  }
} /*** end of Timein8CaptureInterrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer channel 9.
** \return    none.
**
****************************************************************************************/
void Timein9Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM9, TIM_IT_Update) == SET)
  {
    /* process the event */
    TimeinOverflowInterrupt(TIMEIN_MODULE_TIM9);
  }
  else
  {
    /* --------------- was it an input capture event for the 1st channel? ------------ */
    if(TIM_GetITStatus(TIM9, TIM_IT_CC1) == SET)
    {
      /* process the event */
      TimeinCaptureInterrupt(TIMEIN_TIM9_PIN_PE5);
    }
    /* --------------- was it an input capture event for the 2nd channel? ------------ */
    else if(TIM_GetITStatus(TIM9, TIM_IT_CC2) == SET)
    {
      /* process the event */
      TimeinCaptureInterrupt(TIMEIN_TIM9_PIN_PE6);
    }
  }
} /*** end of Timein9Interrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer channel 11.
** \return    none.
**
****************************************************************************************/
void Timein11Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM11, TIM_IT_Update) == SET)
  {
    /* process the event */
    TimeinOverflowInterrupt(TIMEIN_MODULE_TIM11);
  }
  else
  {
    /* --------------- was it an input capture event for the 1st channel? ------------ */
    if(TIM_GetITStatus(TIM11, TIM_IT_CC1) == SET)
    {
      /* process the event */
      TimeinCaptureInterrupt(TIMEIN_TIM11_PIN_PF7);
    }
  }
} /*** end of Timein11Interrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer channel 14.
** \return    none.
**
****************************************************************************************/
void Timein14Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM14, TIM_IT_Update) == SET)
  {
    /* process the event */
    TimeinOverflowInterrupt(TIMEIN_MODULE_TIM14);
  }
  else
  {
    /* --------------- was it an input capture event for the 1st channel? ------------ */
    if(TIM_GetITStatus(TIM14, TIM_IT_CC1) == SET)
    {
      /* process the event */
      TimeinCaptureInterrupt(TIMEIN_TIM14_PIN_PF9);
    }
  }
} /*** end of Timein14Interrupt ***/


/************************************ end of timein.c **********************************/


