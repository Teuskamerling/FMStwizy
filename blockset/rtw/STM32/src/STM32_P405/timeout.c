/************************************************************************************//**
* \file         timeout.c
* \brief        Timer outputs (output compare) driver source file.
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
#include "timeout.h"                                  /* Timer output driver           */
#include "os.h"                                       /* for operating system          */
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */
#include "stm32f4xx.h"                                /* STM32 registers               */
#include "stm32f4xx_conf.h"                           /* STM32 peripheral drivers      */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief timer interrupt priority */
#define TIMEOUT_NVIC_IRQ_PRIO            (configLIBRARY_KERNEL_INTERRUPT_PRIORITY)

/** \brief Sets the number of output capture channels for a timer module. This is
 *         hardware defined.
 */
#define TIMEOUT_MAX_CHANNELS_PER_MODULE  (4)

/** \brief Determines the number of counts for the free running counter. */
#define TIMEOUT_PERIOD_COUNTS            (65536)

/** \brief Macro for easy access to the maximum number of supported timer modules. */
#define TIMEOUT_MODULES_MAX              (sizeof(moduleMapping)/sizeof(moduleMapping[0]))


/****************************************************************************************
* Type definitions
****************************************************************************************/
/**< \brief Structure type with module mapping information. */
typedef struct
{
  TIM_TypeDef *timer_peripheral;
  uint32_t timer_peripheral_clk;
  uint8_t timer_apb_number;
  uint8_t timer_prescale_divider;
  uint8_t timer_nvic_channel;
} tTimeoutModuleMapping;

/**< \brief Structure type with pin mapping information. */
typedef struct
{
  uint32_t peripheral_clk;
  GPIO_TypeDef* port;
  uint16_t pin;
  uint8_t module_idx;
  uint8_t channel_idx;
  uint16_t compare_irq_flg;
  uint16_t polarity_bit;
  uint8_t gpio_af;
  uint16_t pin_source;
} tTimeoutPinMapping;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void TimeoutOverflowInterrupt(uint8_t module_id);
static void TimeoutCompareInterrupt(uint8_t pin_id);
static void TimeoutClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT);


/****************************************************************************************
* Constant data declarations
****************************************************************************************/
/** \brief Array with all configuration parameters of a timer module. note that
 *         TIM2-7 and TIM12-14 have a max input clock frequency of 84 Mhz as opposed
 *         to 168 MHz so these modules need an extra prescale divider.
 */
const static tTimeoutModuleMapping moduleMapping[] =
{
    { TIM1,  RCC_APB2Periph_TIM1, 2, 1, TIM1_CC_IRQn            }, /* idx 0: TIMEOUT_MODULE_TIM1  */
    { TIM2,  RCC_APB1Periph_TIM2, 1, 2, TIM2_IRQn               }, /* idx 1: TIMEOUT_MODULE_TIM2  */
    { TIM3,  RCC_APB1Periph_TIM3, 1, 2, TIM3_IRQn               }, /* idx 2: TIMEOUT_MODULE_TIM3  */
    { TIM4,  RCC_APB1Periph_TIM4, 1, 2, TIM4_IRQn               }  /* idx 3: TIMEOUT_MODULE_TIM4  */
};

/** \brief Array with all configuration parameters of an output capture pin. */
const static tTimeoutPinMapping pinMapping[] =
{
  { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_8,  TIMEOUT_MODULE_TIM1,  0, TIM_IT_CC1, TIM_CCER_CC1P, GPIO_AF_TIM1,  GPIO_PinSource8  }, /* idx 0:  TIMEOUT_TIM1_PIN_PA8   */
  { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_9,  TIMEOUT_MODULE_TIM1,  1, TIM_IT_CC2, TIM_CCER_CC2P, GPIO_AF_TIM1,  GPIO_PinSource9  }, /* idx 1:  TIMEOUT_TIM1_PIN_PA9   */
  { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_10, TIMEOUT_MODULE_TIM1,  2, TIM_IT_CC3, TIM_CCER_CC3P, GPIO_AF_TIM1,  GPIO_PinSource10 }, /* idx 2:  TIMEOUT_TIM1_PIN_PA10  */
  { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_1,  TIMEOUT_MODULE_TIM2,  1, TIM_IT_CC2, TIM_CCER_CC2P, GPIO_AF_TIM2,  GPIO_PinSource1  }, /* idx 3:  TIMEOUT_TIM2_PIN_PA1   */
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_6,  TIMEOUT_MODULE_TIM3,  0, TIM_IT_CC1, TIM_CCER_CC1P, GPIO_AF_TIM3,  GPIO_PinSource6  }, /* idx 4:  TIMEOUT_TIM3_PIN_PC6   */
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_7,  TIMEOUT_MODULE_TIM3,  1, TIM_IT_CC2, TIM_CCER_CC2P, GPIO_AF_TIM3,  GPIO_PinSource7  }, /* idx 5:  TIMEOUT_TIM3_PIN_PC7   */
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_8,  TIMEOUT_MODULE_TIM3,  2, TIM_IT_CC3, TIM_CCER_CC3P, GPIO_AF_TIM3,  GPIO_PinSource8  }, /* idx 6:  TIMEOUT_TIM3_PIN_PC8   */
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_9,  TIMEOUT_MODULE_TIM3,  3, TIM_IT_CC4, TIM_CCER_CC4P, GPIO_AF_TIM3,  GPIO_PinSource9  }, /* idx 7:  TIMEOUT_TIM3_PIN_PC9   */
  { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_6,  TIMEOUT_MODULE_TIM4,  0, TIM_IT_CC1, TIM_CCER_CC1P, GPIO_AF_TIM4,  GPIO_PinSource6  }, /* idx 8:  TIMEOUT_TIM4_PIN_PB6   */
  { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_7,  TIMEOUT_MODULE_TIM4,  1, TIM_IT_CC2, TIM_CCER_CC2P, GPIO_AF_TIM4,  GPIO_PinSource7  }, /* idx 9:  TIMEOUT_TIM4_PIN_PB7   */
  { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_8,  TIMEOUT_MODULE_TIM4,  2, TIM_IT_CC3, TIM_CCER_CC3P, GPIO_AF_TIM4,  GPIO_PinSource8  }, /* idx 10: TIMEOUT_TIM4_PIN_PB8   */
  { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_9,  TIMEOUT_MODULE_TIM4,  3, TIM_IT_CC4, TIM_CCER_CC4P, GPIO_AF_TIM4,  GPIO_PinSource9  }  /* idx 11: TIMEOUT_TIM4_PIN_PB9   */
};


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Function pointers for the user specified free running counter overflow
 *         detected interrupt handler. 
 */
static tTimeoutCallbackOverflow counterOverflowDetected[TIMEOUT_MODULES_MAX];

/** \brief Function pointers for the user specified output compare event detected
 *         interrupt handler. 
 */
static tTimeoutCallbackCompareEvent compareEventDetected[TIMEOUT_MODULES_MAX][TIMEOUT_MAX_CHANNELS_PER_MODULE];

/** \brief Holds information about if a timer module's pin was already configured. */
static volatile uint8_t pin_configured[TIMEOUT_MODULES_MAX][TIMEOUT_MAX_CHANNELS_PER_MODULE];

/** \brief Holds information about if a timer module was already configured. */
static volatile uint8_t module_initialized[TIMEOUT_MODULES_MAX] = { FALSE, FALSE, FALSE, FALSE };

/** \brief Holds a timestamp of the free running counter from the time that the last
 *         output compare event was detected.
 */
static volatile uint16_t lastEventCounter[TIMEOUT_MODULES_MAX][TIMEOUT_MAX_CHANNELS_PER_MODULE];


/************************************************************************************//**
** \brief     Initializes the timer outputs driver for a specific timer module.
** \param     module_id Timer module identifier.
** \param     frequnecy Frequency in Hz for the free running counter.
** \return    none.
**
****************************************************************************************/
void TimeoutInitModule(uint8_t module_id, uint32_t frequency)
{
  NVIC_InitTypeDef  NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef tim_base;
  uint16_t prescaler;
  uint8_t i;

  /* make sure the id is valid before using it as an array indexer */
  if (!(module_id < sizeof(moduleMapping)/sizeof(moduleMapping[0])))
  {
    ErrCodesSetError(ER_CODE_TIMEOUT_INVALID_MODULE, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* only initialize if not yet initialized */
  if (module_initialized[module_id] == TRUE)
  {
    /* nothing left to do because this module is already initialized */
    return;
  }
  /* reset the free running counter overflow event callback */
  counterOverflowDetected[module_id] = NULL;
  /* reset the output compare event callback and the channel/pin initialization flag. */
  for (i=0; i<TIMEOUT_MAX_CHANNELS_PER_MODULE; i++)
  {
    lastEventCounter[module_id][i] = 0;
    compareEventDetected[module_id][i] = NULL;
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
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMEOUT_NVIC_IRQ_PRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* timer 1 is special as it has separate IRQ channels */
  if (moduleMapping[module_id].timer_peripheral == TIM1)
  {
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMEOUT_NVIC_IRQ_PRIO;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  }
  /* compute the prescaler value */
  prescaler = (SystemCoreClock / (frequency * moduleMapping[module_id].timer_prescale_divider  )) - 1;
  /* configure the timer module and enable it */
  tim_base.TIM_Period = (TIMEOUT_PERIOD_COUNTS - 1);
  tim_base.TIM_Prescaler = prescaler;
  tim_base.TIM_ClockDivision = 0;
  tim_base.TIM_CounterMode = TIM_CounterMode_Up;
  tim_base.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(moduleMapping[module_id].timer_peripheral, &tim_base);
  TIM_ARRPreloadConfig(moduleMapping[module_id].timer_peripheral, DISABLE);
  TIM_SetCounter(moduleMapping[module_id].timer_peripheral, 0);
  TIM_Cmd(moduleMapping[module_id].timer_peripheral, ENABLE);
  /* turn on the outputs */
  TIM_CtrlPWMOutputs(moduleMapping[module_id].timer_peripheral, ENABLE);
  /* enable the counter overflow interrupt request for the module */
  TIM_ITConfig(moduleMapping[module_id].timer_peripheral, TIM_IT_Update, ENABLE);
  /* set module to initialized */
  module_initialized[module_id] = TRUE;
} /*** end of TimeoutInit ***/


/************************************************************************************//**
** \brief     Configures a timer module channel pin for output compare operation, if not
**            already done, and schedule the next output compare event.
** \param     pin_id Pin identifier.
** \param     eventCounter Free running counter value at which the event should be 
**                         generated.
** \param     action The action that should be taken when the event occurred.
** \return    none.
**
****************************************************************************************/
void TimeoutScheduleCompareEvent(uint8_t pin_id, uint16_t eventCounter, tTimeoutAction action)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;


  /* make sure the id is valid before using it as an array indexer */
  if (!(pin_id < sizeof(pinMapping)/sizeof(pinMapping[0])))
  {
    ErrCodesSetError(ER_CODE_TIMEOUT_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* do one-time pin configuration */
  if (pin_configured[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] == FALSE)
  {
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
    /* flag pin as configured */
    pin_configured[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = TRUE;
  }
  /* output compare default values configuration */
  TIM_OCStructInit(&TIM_OCInitStructure);
  /* configure output compare */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = eventCounter;
  if (action == INVERT)
  {
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
  }
  else if (action == SETLOW)
  {
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;
  }
  else if (action == SETHIGH)
  {
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Active;
  }
  else
  {
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  }
  /* store the desired output compare configuration */
  if (pinMapping[pin_id].channel_idx == 0)
  {
    TIM_OC1Init(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, TIM_OCPreload_Disable);
  }
  else if (pinMapping[pin_id].channel_idx == 1)
  {
    TIM_OC2Init(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, TIM_OCPreload_Disable);
  }
  else if (pinMapping[pin_id].channel_idx == 2)
  {
    TIM_OC3Init(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, TIM_OCPreload_Disable);
  }
  else if (pinMapping[pin_id].channel_idx == 3)
  {
    TIM_OC4Init(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, TIM_OCPreload_Disable);
  }
  /* enable the output compare interrupt request for the channel */
  TIM_ITConfig(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, pinMapping[pin_id].compare_irq_flg, ENABLE);
} /*** end of TimeoutScheduleCompareEvent ***/


/************************************************************************************//**
** \brief     Obtains the current value of the timer module's free running counter.
** \param     module_id Timer module identifier.
** \return    Free running counter value.
**
****************************************************************************************/
uint16_t TimeoutGetFreeRunningCounter(uint8_t module_id)
{
  /* make sure the id is valid before using it as an array indexer */
  if (!(module_id < sizeof(moduleMapping)/sizeof(moduleMapping[0])))
  {
    ErrCodesSetError(ER_CODE_TIMEOUT_INVALID_MODULE, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* only access timer module if it was initialized */
  if (module_initialized[module_id] != TRUE)
  {
    return 0;
  }

  /* read out the value of the counter */
  return TIM_GetCounter(moduleMapping[module_id].timer_peripheral);
} /*** end of TimeoutGetFreeRunningCounter ***/


/************************************************************************************//**
** \brief     Resets the value of the timer module's free running counter to 0.
** \param     module_id Timer module identifier.
** \return    none.
**
****************************************************************************************/
void TimeoutResetFreeRunningCounter(uint8_t module_id)
{
  /* make sure the id is valid before using it as an array indexer */
  if (!(module_id < sizeof(moduleMapping)/sizeof(moduleMapping[0])))
  {
    ErrCodesSetError(ER_CODE_TIMEOUT_INVALID_MODULE, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* only access timer module if it was initialized */
  if (module_initialized[module_id] == TRUE)
  {
    /* set the free running counter back to zero */
    TIM_SetCounter(moduleMapping[module_id].timer_peripheral, 0);
  }
} /*** end of TimeoutResetFreeRunningCounter ***/


/************************************************************************************//**
** \brief     Obtains the counter value of when the last output compare event occurred.
** \param     pin_id Pin identifier.
** \return    Counter value of when the last output compare event occurred.
**
****************************************************************************************/
uint16_t TimeoutGetLastEventCounter(uint8_t pin_id)
{
  /* make sure the id is valid before using it as an array indexer */
  if (!(pin_id < sizeof(pinMapping)/sizeof(pinMapping[0])))
  {
    ErrCodesSetError(ER_CODE_TIMEOUT_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* read out the counter that golds the timestamp of when the last output compare
   * occurred.
   */
  return lastEventCounter[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx];
} /*** end of TimeoutGetLastEventCounter ***/


/************************************************************************************//**
** \brief     Registers the callback handler that gets called by the driver each time
**            the 16-bit free running counter for the module overflows.
** \param     module_id Timer module identifier.
** \param     callbackPtr Function pointer to the callback.
** \return    none.
**
****************************************************************************************/
void TimeoutRegisterOverflowCallback(uint8_t module_id, tTimeoutCallbackOverflow callbackPtr)
{
  /* make sure the id is valid before using it as an array indexer */
  if (!(module_id < sizeof(moduleMapping)/sizeof(moduleMapping[0])))
  {
    ErrCodesSetError(ER_CODE_TIMEOUT_INVALID_MODULE, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* store the function pointer */
  counterOverflowDetected[module_id] = callbackPtr;
} /*** end of TimeoutRegisterOverflowCallback ***/


/************************************************************************************//**
** \brief     Registers the callback handler that gets called by the driver each time
**            the channel's output compare event occurred.
** \param     pin_id Pin identifier.
** \param     callbackPtr Function pointer to the callback.
** \return    none.
**
****************************************************************************************/
void TimeoutRegisterCompareEventCallback(uint8_t pin_id, tTimeoutCallbackCompareEvent callbackPtr)
{
  /* make sure the id is valid before using it as an array indexer */
  if (!(pin_id < sizeof(pinMapping)/sizeof(pinMapping[0])))
  {
    ErrCodesSetError(ER_CODE_TIMEOUT_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* store the function pointer */
  compareEventDetected[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = callbackPtr;
} /*** end of TimeoutRegisterCompareEventCallback ***/


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
static void TimeoutClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT)
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
} /*** end of TimeoutClearITPendingBit ***/


/************************************************************************************//**
** \brief     Generic interrupt service routine handler for the free running counter
**            overflow interrupt event.
** \param     module_id Timer module identifier.
** \return    none.
**
****************************************************************************************/
static void TimeoutOverflowInterrupt(uint8_t module_id)
{
  /* this ISR is shared with others, so make sure this module is configured before
   * processing this interrupt.
   */
  if (module_initialized[module_id] == FALSE)
  {
    /* interrupt not for us */
    return;
  }

  /* clear timer counter overflow event flag for the channel of the event */
  TimeoutClearITPendingBit(moduleMapping[module_id].timer_peripheral, TIM_IT_Update);
  /* invoke callback to pass on the counter overflow event */
  if (counterOverflowDetected[module_id] != NULL)
  {
    counterOverflowDetected[module_id]();
  }
} /*** end of TimeoutOverflowInterrupt ***/


/************************************************************************************//**
** \brief     Generic interrupt service routine handler for the output compare interrupt
**            event.
** \param     pin_id Pin identifier.
** \return    none.
**
****************************************************************************************/
static void TimeoutCompareInterrupt(uint8_t pin_id)
{
  uint16_t eventTimestamp;

  /* this ISR is shared with others, so make sure this pin is configured before
   * processing this interrupt.
   */
  if (pin_configured[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] == FALSE)
  {
    /* interrupt not for us */
    return;
  }
  /* make sure to read the configured event counter timestamp from the correct channel */
  if (pinMapping[pin_id].channel_idx == 0)
  {
    eventTimestamp = moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral->CCR1;
  }
  else if (pinMapping[pin_id].channel_idx == 1)
  {
    eventTimestamp = moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral->CCR2;
  }
  else if (pinMapping[pin_id].channel_idx == 2)
  {
    eventTimestamp = moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral->CCR3;
  }
  else if (pinMapping[pin_id].channel_idx == 3)
  {
    eventTimestamp = moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral->CCR4;
  }
  /* clear timer module's output compare flag for the channel of the event */
  TimeoutClearITPendingBit(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, pinMapping[pin_id].compare_irq_flg);
  /* store compare counter into last compare event counter */
  lastEventCounter[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] = eventTimestamp;
  /* invoke callback to pass on compare event */
  if (compareEventDetected[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx] != NULL)
  {
    compareEventDetected[pinMapping[pin_id].module_idx][pinMapping[pin_id].channel_idx]();
  }
} /*** end of TimeoutCompareInterrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer update channel 1 and the timer 10
**            interrupt channel.
** \return    none.
**
****************************************************************************************/
void Timeout1UpdateTimeout10Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
  {
    /* process the event */
    TimeoutOverflowInterrupt(TIMEOUT_MODULE_TIM1);
  }
} /*** end of Timeout1UpdateTimeout10Interrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer compare channel 1.
** \return    none.
**
****************************************************************************************/
void Timeout1CaptureInterrupt(void)
{
  /* --------------- was it an output compare event for the 1st channel? ------------- */
  if(TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET)
  {
    /* process the event */
    TimeoutCompareInterrupt(TIMEOUT_TIM1_PIN_PA8);
  }
  /* --------------- was it an output compare event for the 2nd channel? ------------- */
  else if(TIM_GetITStatus(TIM1, TIM_IT_CC2) == SET)
  {
    /* process the event */
    TimeoutCompareInterrupt(TIMEOUT_TIM1_PIN_PA9);
  }
  /* --------------- was it an output compare event for the 3rd channel? ------------- */
  else if(TIM_GetITStatus(TIM1, TIM_IT_CC3) == SET)
  {
    /* process the event */
    TimeoutCompareInterrupt(TIMEOUT_TIM1_PIN_PA10);
  }
} /*** end of Timeout1CaptureInterrupt ***/



/************************************************************************************//**
** \brief     Interrupt service routine for timer channel 2.
** \return    none.
**
****************************************************************************************/
void Timeout2Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
  {
    /* process the event */
    TimeoutOverflowInterrupt(TIMEOUT_MODULE_TIM2);
  }
  else
  {
    /* --------------- was it an output compare event for the 2nd channel? ----------- */
    if(TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET)
    {
      /* process the event */
      TimeoutCompareInterrupt(TIMEOUT_TIM2_PIN_PA1);
    }
  }
} /*** end of Timeout2Interrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer channel 3.
** \return    none.
**
****************************************************************************************/
void Timeout3Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
  {
    /* process the event */
    TimeoutOverflowInterrupt(TIMEOUT_MODULE_TIM3);
  }
  else
  {
    /* --------------- was it an output compare event for the 1st channel? ----------- */
    if(TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET)
    {
      /* process the event */
      TimeoutCompareInterrupt(TIMEOUT_TIM3_PIN_PC6);
    }
    /* --------------- was it an output compare event for the 2nd channel? ----------- */
    else if(TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET)
    {
      /* process the event */
      TimeoutCompareInterrupt(TIMEOUT_TIM3_PIN_PC7);
    }
    /* --------------- was it an output compare event for the 3rd channel? ----------- */
    else if(TIM_GetITStatus(TIM3, TIM_IT_CC3) == SET)
    {
      /* process the event */
      TimeoutCompareInterrupt(TIMEOUT_TIM3_PIN_PC8);
    }
    /* --------------- was it an output compare event for the 4th channel? ----------- */
    else if(TIM_GetITStatus(TIM3, TIM_IT_CC4) == SET)
    {
      /* process the event */
      TimeoutCompareInterrupt(TIMEOUT_TIM3_PIN_PC9);
    }
  }
} /*** end of Timeout3Interrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer channel 4.
** \return    none.
**
****************************************************************************************/
void Timeout4Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
  {
    /* process the event */
    TimeoutOverflowInterrupt(TIMEOUT_MODULE_TIM4);
  }
  else
  {
    /* --------------- was it an output compare event for the 1st channel? ----------- */
    if(TIM_GetITStatus(TIM4, TIM_IT_CC1) == SET)
    {
      /* process the event */
      TimeoutCompareInterrupt(TIMEOUT_TIM4_PIN_PB6);
    }
    /* --------------- was it an output compare event for the 2nd channel? ----------- */
    else if(TIM_GetITStatus(TIM4, TIM_IT_CC2) == SET)
    {
      /* process the event */
      TimeoutCompareInterrupt(TIMEOUT_TIM4_PIN_PB7);
    }
    /* --------------- was it an output compare event for the 3rd channel? ----------- */
    else if(TIM_GetITStatus(TIM4, TIM_IT_CC3) == SET)
    {
      /* process the event */
      TimeoutCompareInterrupt(TIMEOUT_TIM4_PIN_PB8);
    }
    /* --------------- was it an output compare event for the 4th channel? ----------- */
    else if(TIM_GetITStatus(TIM4, TIM_IT_CC4) == SET)
    {
      /* process the event */
      TimeoutCompareInterrupt(TIMEOUT_TIM4_PIN_PB9);
    }
  }
} /*** end of Timeout4Interrupt ***/


/************************************ end of timeout.c *********************************/


