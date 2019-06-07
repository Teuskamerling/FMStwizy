/************************************************************************************//**
* \file         pwmout.c
* \brief        PWM outputs driver source file.
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
#include "pwmout.h"                                   /* PWM output driver             */
#include "os.h"                                       /* for operating system          */
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */
#include "stm32f4xx.h"                                /* STM32 registers               */
#include "stm32f4xx_conf.h"                           /* STM32 peripheral drivers      */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Number of timer modules per channel. This is hardware defined. */
#define PWMOUT_CHANNELS_PER_MODULE  (4)

/** \brief Maximum value for the 10-bit duty cycle. */
#define PWMOUT_MAX_DUTY_CYCLE       (1023)

/** \brief Timer interrupt priority for the PWM modules. */
#define PWM_NVIC_IRQ_PRIO           (configLIBRARY_KERNEL_INTERRUPT_PRIORITY)

/** \brief Macro for easy access to the maximum number of supported PWM modules. */
#define PWM_MODULES_MAX               (sizeof(moduleMapping)/sizeof(moduleMapping[0]))


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
} tPwmoutModuleMapping;

/**< \brief Structure type with pin mapping information. */
typedef struct
{
  uint8_t peripheral_clk;
  GPIO_TypeDef* port;
  uint16_t pin;
  uint8_t module_idx;
  uint8_t channel_idx;
  uint8_t gpio_af;
  uint16_t pin_source;
} tPwmoutPinMapping;


/****************************************************************************************
* External function prototypes
****************************************************************************************/
extern void TimeinClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT);


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void PwmoutModulePeriodCompleteInterrupt(uint8_t module_id);


/****************************************************************************************
* Constant data declarations
****************************************************************************************/
/** \brief Array with all configuration parameters of a pwm timer module. note that
 *         TIM2-7 and TIM12-14 have a max input clock frequency of 84 Mhz as opposed
 *         to 168 MHz so these modules need an extra prescale divider.
 */
const static tPwmoutModuleMapping moduleMapping[] =
{
  { TIM1,  RCC_APB2Periph_TIM1,  2, 1, TIM1_UP_TIM10_IRQn },      /* idx 0: PWMOUT_MODULE_TIM1   */
  { TIM2,  RCC_APB1Periph_TIM2,  1, 2, TIM2_IRQn },               /* idx 1: PWMOUT_MODULE_TIM2  */
  { TIM3,  RCC_APB1Periph_TIM3,  1, 2, TIM3_IRQn },               /* idx 2: PWMOUT_MODULE_TIM3  */
  { TIM4,  RCC_APB1Periph_TIM4,  1, 2, TIM4_IRQn },               /* idx 3: PWMOUT_MODULE_TIM4  */
  { TIM8,  RCC_APB2Periph_TIM8,  2, 1, TIM8_UP_TIM13_IRQn },      /* idx 4: PWMOUT_MODULE_TIM8   */
  { TIM9,  RCC_APB2Periph_TIM9,  2, 1, TIM1_BRK_TIM9_IRQn },      /* idx 5: PWMOUT_MODULE_TIM9   */
  { TIM10, RCC_APB2Periph_TIM10, 2, 1, TIM1_UP_TIM10_IRQn },      /* idx 6: PWMOUT_MODULE_TIM10  */
  { TIM11, RCC_APB2Periph_TIM11, 2, 1, TIM1_TRG_COM_TIM11_IRQn }, /* idx 7: PWMOUT_MODULE_TIM11  */
  { TIM13, RCC_APB1Periph_TIM13, 1, 2, TIM8_UP_TIM13_IRQn },      /* idx 8: PWMOUT_MODULE_TIM13  */
  { TIM14, RCC_APB1Periph_TIM14, 1, 2, TIM8_TRG_COM_TIM14_IRQn }  /* idx 9: PWMOUT_MODULE_TIM14  */
};

/** \brief Array with all configuration parameters of a pwm output pin. */
const static tPwmoutPinMapping pinMapping[] =
{
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_9,  PWMOUT_MODULE_TIM1,  0, GPIO_AF_TIM1,  GPIO_PinSource9  }, /* idx 0:  PWMOUT_TIM1_PIN_PE9   */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_11, PWMOUT_MODULE_TIM1,  1, GPIO_AF_TIM1,  GPIO_PinSource11 }, /* idx 1:  PWMOUT_TIM1_PIN_PE11  */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_13, PWMOUT_MODULE_TIM1,  2, GPIO_AF_TIM1,  GPIO_PinSource13 }, /* idx 2:  PWMOUT_TIM1_PIN_PE13  */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_14, PWMOUT_MODULE_TIM1,  3, GPIO_AF_TIM1,  GPIO_PinSource14 }, /* idx 3:  PWMOUT_TIM1_PIN_PE14  */
  { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_5,  PWMOUT_MODULE_TIM2,  0, GPIO_AF_TIM2,  GPIO_PinSource5  }, /* idx 4:  PWMOUT_TIM2_PIN_PA5   */
  { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_6,  PWMOUT_MODULE_TIM3,  0, GPIO_AF_TIM3,  GPIO_PinSource6  }, /* idx 5:  PWMOUT_TIM3_PIN_PA6   */
  { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_5,  PWMOUT_MODULE_TIM3,  1, GPIO_AF_TIM3,  GPIO_PinSource5  }, /* idx 6:  PWMOUT_TIM3_PIN_PB5   */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_12, PWMOUT_MODULE_TIM4,  0, GPIO_AF_TIM4,  GPIO_PinSource12 }, /* idx 7:  PWMOUT_TIM4_PIN_PD12  */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_13, PWMOUT_MODULE_TIM4,  1, GPIO_AF_TIM4,  GPIO_PinSource13 }, /* idx 8:  PWMOUT_TIM4_PIN_PD13  */
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_6,  PWMOUT_MODULE_TIM8,  0, GPIO_AF_TIM8,  GPIO_PinSource6  }, /* idx 9:  PWMOUT_TIM8_PIN_PC6   */
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_7,  PWMOUT_MODULE_TIM8,  1, GPIO_AF_TIM8,  GPIO_PinSource7  }, /* idx 10: PWMOUT_TIM8_PIN_PC7   */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_5,  PWMOUT_MODULE_TIM9,  0, GPIO_AF_TIM9,  GPIO_PinSource5  }, /* idx 11: PWMOUT_TIM9_PIN_PE5   */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_6,  PWMOUT_MODULE_TIM9,  1, GPIO_AF_TIM9,  GPIO_PinSource6  }, /* idx 12: PWMOUT_TIM9_PIN_PE6   */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_6,  PWMOUT_MODULE_TIM10, 0, GPIO_AF_TIM10, GPIO_PinSource6  }, /* idx 13: PWMOUT_TIM10_PIN_PF6  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_7,  PWMOUT_MODULE_TIM11, 0, GPIO_AF_TIM11, GPIO_PinSource7  }, /* idx 14: PWMOUT_TIM11_PIN_PF7  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_8,  PWMOUT_MODULE_TIM13, 0, GPIO_AF_TIM13, GPIO_PinSource8  }, /* idx 15: PWMOUT_TIM13_PIN_PF8  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_9,  PWMOUT_MODULE_TIM14, 0, GPIO_AF_TIM14, GPIO_PinSource9  }  /* idx 16: PWMOUT_TIM14_PIN_PF9  */
};

/** \brief Array with all configuration parameters of a pwm output pin for the extra inverted TIM1
 *         channel outputs.
 */
const static tPwmoutPinMapping invertedPinMapping[] =
{
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_8,  PWMOUT_MODULE_TIM1, 0, GPIO_AF_TIM1, GPIO_PinSource8  }, /* idx 0:  PWMOUT_TIM1_PIN_PE9   */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_10, PWMOUT_MODULE_TIM1, 1, GPIO_AF_TIM1, GPIO_PinSource10 }, /* idx 1:  PWMOUT_TIM1_PIN_PE11  */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_12, PWMOUT_MODULE_TIM1, 2, GPIO_AF_TIM1, GPIO_PinSource12 }, /* idx 2:  PWMOUT_TIM1_PIN_PE13  */
};


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Holds information about if a timer module was already configured. */
static volatile uint8_t module_initialized[] = { FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE };

/** \brief Holds information about if a PWM module's pin was already configured. */
static volatile uint8_t pin_configured[] = {
  FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE,
  FALSE, FALSE, FALSE, FALSE, FALSE
};

/** \brief Function pointers for the user specified edge detected interrupt handler. */
static tPwmoutCallbackPeriodComplete callbackPeriodComplete[PWM_MODULES_MAX];


/************************************************************************************//**
** \brief     Initializes a PWM timer module.
** \param     module_id Timer module identifier.
** \param     frequency PWM frequency for the timer module.
** \param     alignment desired PWM signal alignment.
** \return    none.
**
****************************************************************************************/
void PwmoutInit(uint8_t module_id, uint32_t frequency, tPwmoutAlignmentCfg alignment)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  uint16_t prescaler;
  TIM_TimeBaseInitTypeDef tim_base;
  TIM_BDTRInitTypeDef bdtr;

  /* make sure the id is valid before using it as an array indexer */
  if (!(module_id < sizeof(moduleMapping)/sizeof(moduleMapping[0])))
  {
    ErrCodesSetError(ER_CODE_PWMOUT_INVALID_MODULE, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* reset the user specified period complete event handler */
  callbackPeriodComplete[module_id] = NULL;

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
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PWM_NVIC_IRQ_PRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* compute the prescaler value */
  prescaler = ( SystemCoreClock / (frequency * moduleMapping[module_id].timer_prescale_divider  * (PWMOUT_MAX_DUTY_CYCLE + 1))) - 1;
  /* time base configuration */
  TIM_TimeBaseStructInit (&tim_base);
  tim_base.TIM_Period = PWMOUT_MAX_DUTY_CYCLE;
  tim_base.TIM_Prescaler = prescaler;
  tim_base.TIM_ClockDivision = 0;
  tim_base.TIM_RepetitionCounter = 0;

  /* alignment configuration */
  if (alignment == PWMOUT_EDGE_ALIGNMENT)
  {
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;
  }
  else
  {
    tim_base.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
  }
  TIM_TimeBaseInit(moduleMapping[module_id].timer_peripheral, &tim_base);

  if ( (moduleMapping[module_id].timer_peripheral == TIM1) || (moduleMapping[module_id].timer_peripheral == TIM8) )
  {
    TIM_BDTRStructInit(&bdtr);
    bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(moduleMapping[module_id].timer_peripheral, &bdtr);
  }

  /* enable the timer */
  TIM_ARRPreloadConfig(moduleMapping[module_id].timer_peripheral, ENABLE);
  TIM_Cmd(moduleMapping[module_id].timer_peripheral, ENABLE);
  TIM_CtrlPWMOutputs(moduleMapping[module_id].timer_peripheral, ENABLE);
  /* enable the counter overflow interrupt request for the module */
  TIM_ITConfig(moduleMapping[module_id].timer_peripheral, TIM_IT_Update, ENABLE);
  /* flag module as configured */
  module_initialized[module_id] = TRUE;
} /*** end of PwmoutInit ***/


/************************************************************************************//**
** \brief     Configures a PWM output pin.
** \param     pin_id Pin identifier.
** \param     polarity PWM signal polarity.
** \param     inverted_output Enables the generation of an additional inverted PWM signal
**            on a seperate output pin. Note that only TIM1 supports this functionality
**            on channel 1 - channel 3.
** \return    none.
**
****************************************************************************************/
void PwmoutConfigure(uint8_t pin_id, tPwmoutPolarityCfg polarity, tPwmoutInvertedOutputCfg inverted_output)
{
  GPIO_InitTypeDef  gpio_init;
  TIM_OCInitTypeDef tim_ocinit;


  /* make sure the id is valid before using it as an array indexer */
  if (!(pin_id < sizeof(pinMapping)/sizeof(pinMapping[0])))
  {
    ErrCodesSetError(ER_CODE_PWMOUT_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* enable the port's peripheral clock */
  RCC_AHB1PeriphClockCmd(pinMapping[pin_id].peripheral_clk, ENABLE);

  /* prepare pin configuration */
  gpio_init.GPIO_Pin = pinMapping[pin_id].pin;
  gpio_init.GPIO_Mode = GPIO_Mode_AF;
  gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;

  /* initialize the pin */
  GPIO_Init(pinMapping[pin_id].port, &gpio_init);

  /* initialize the pin for the additional inverted signal */
  if ( (inverted_output == PWMOUT_INVERTED_OUTPUT_ON) &&
       (pinMapping[pin_id].module_idx == PWMOUT_MODULE_TIM1) &&
       (pinMapping[pin_id].channel_idx <= 2) )
  {
    RCC_AHB1PeriphClockCmd(invertedPinMapping[pin_id].peripheral_clk, ENABLE);
    gpio_init.GPIO_Pin = invertedPinMapping[pin_id].pin;
    GPIO_Init(invertedPinMapping[pin_id].port, &gpio_init);
    /* connect the pin to it's timer module alternate function */
    GPIO_PinAFConfig(invertedPinMapping[pin_id].port, invertedPinMapping[pin_id].pin_source, invertedPinMapping[pin_id].gpio_af);
  }

  /* connect the pin to it's timer module alternate function */
  GPIO_PinAFConfig(pinMapping[pin_id].port, pinMapping[pin_id].pin_source, pinMapping[pin_id].gpio_af);

  /* configure the timer output channel as PWM */
  TIM_OCStructInit(&tim_ocinit);
  tim_ocinit.TIM_OCMode = TIM_OCMode_PWM1;
  tim_ocinit.TIM_OutputState = TIM_OutputState_Enable;
  if ( (inverted_output == PWMOUT_INVERTED_OUTPUT_ON) &&
       (pinMapping[pin_id].module_idx == PWMOUT_MODULE_TIM1) &&
       (pinMapping[pin_id].channel_idx <= 2) )
  {
    tim_ocinit.TIM_OutputNState = TIM_OutputNState_Enable;
  }
  else
  {
    tim_ocinit.TIM_OutputNState = TIM_OutputNState_Disable;
  }
  tim_ocinit.TIM_Pulse = 0;

  if (polarity == PWMOUT_ACTIVE_HIGH)
  {
    tim_ocinit.TIM_OCPolarity = TIM_OCPolarity_High;
    tim_ocinit.TIM_OCIdleState = TIM_OCIdleState_Reset;
    tim_ocinit.TIM_OCNPolarity = TIM_OCNPolarity_High;
    tim_ocinit.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  }
  else
  {
    tim_ocinit.TIM_OCPolarity = TIM_OCPolarity_Low;
    tim_ocinit.TIM_OCIdleState = TIM_OCIdleState_Reset;
    tim_ocinit.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    tim_ocinit.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  }

  /* make sure to configure the correct channel */
  if (pinMapping[pin_id].channel_idx == 0)
  {
    TIM_OC1Init(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, &tim_ocinit);
    TIM_OC1PreloadConfig(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, TIM_OCPreload_Enable);
  }
  else if (pinMapping[pin_id].channel_idx == 1)
  {
    TIM_OC2Init(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, &tim_ocinit);
    TIM_OC2PreloadConfig(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, TIM_OCPreload_Enable);
  }
  else if (pinMapping[pin_id].channel_idx == 2)
  {
    TIM_OC3Init(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, &tim_ocinit);
    TIM_OC3PreloadConfig(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, TIM_OCPreload_Enable);
  }
  else if (pinMapping[pin_id].channel_idx == 3)
  {
    TIM_OC4Init(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, &tim_ocinit);
    TIM_OC4PreloadConfig(moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral, TIM_OCPreload_Enable);
  }
  /* flag pin as configured */
  pin_configured[pin_id] = TRUE;
} /*** end of DigoutConfigure ***/


/************************************************************************************//**
** \brief     Sets the new duty cycle of the channel, which will be processed the next
**            time PwmoutUpdate is called.
** \param     pin_id Pin identifier.
** \param     duty_cycle Duty cycle value (10-bit).
** \return    none.
**
****************************************************************************************/
void PwmoutSet(uint8_t pin_id, uint16_t duty_cycle)
{
  /* make sure the id is valid before using it as an array indexer */
  if (!(pin_id < sizeof(pinMapping)/sizeof(pinMapping[0])))
  {
    ErrCodesSetError(ER_CODE_PWMOUT_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* only update if pin is actually configured */
  if (pin_configured[pin_id] == FALSE)
  {
    return;
  }

  /* limit duty cycle to its max possible value */
  duty_cycle = (duty_cycle > PWMOUT_MAX_DUTY_CYCLE) ? PWMOUT_MAX_DUTY_CYCLE : duty_cycle;

  /* make sure to set the duty cycle for the correct channel */
  if (pinMapping[pin_id].channel_idx == 0)
  {
    (moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral)->CCR1 = duty_cycle;
  }
  else if (pinMapping[pin_id].channel_idx == 1)
  {
    (moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral)->CCR2 = duty_cycle;
  }
  else if (pinMapping[pin_id].channel_idx == 2)
  {
    (moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral)->CCR3 = duty_cycle;
  }
  else if (pinMapping[pin_id].channel_idx == 3)
  {
    (moduleMapping[pinMapping[pin_id].module_idx].timer_peripheral)->CCR4 = duty_cycle;
  }
} /*** end of PwmoutSet ***/


/************************************************************************************//**
** \brief     Registers the callback handler that gets called by the driver each time
**            a PWM modules period completed. Note that this callback is called on
**            interrupt level.
** \param     module_id PWM module identifier.
** \param     callbackPtr Function pointer to the callback.
** \return    none.
**
****************************************************************************************/
void PwmoutRegisterPeriodCompleteCallback(uint8_t module_id, tPwmoutCallbackPeriodComplete callbackPtr)
{
  /* make sure the id is valid before using it as an array indexer */
  if (!(module_id < sizeof(moduleMapping)/sizeof(moduleMapping[0])))
  {
    ErrCodesSetError(ER_CODE_PWMOUT_INVALID_MODULE, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* store the callback function pointer */
  callbackPeriodComplete[module_id] = callbackPtr;
} /*** end of PwmoutRegisterPeriodCompleteCallback ***/


/************************************************************************************//**
** \brief     Function gets called at interrupt level when a module's configured PWM
**            period completes.
** \param     module_id PWM module identifier.
** \return    none.
**
****************************************************************************************/
static void PwmoutModulePeriodCompleteInterrupt(uint8_t module_id)
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
  TimeinClearITPendingBit(moduleMapping[module_id].timer_peripheral, TIM_IT_Update);

  /* invoke callback to pass on period complete event */
  if (callbackPeriodComplete[module_id] != NULL)
  {
    callbackPeriodComplete[module_id]();
  }
} /*** end of PwmoutModulePeriodCompleteInterrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer update channel 1 and the timer 10
**            interrupt channel.
** \return    none.
**
****************************************************************************************/
void Pwmout1UpdatePwmout10Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
  {
    /* process the event */
    PwmoutModulePeriodCompleteInterrupt(PWMOUT_MODULE_TIM1);
  }
  /* --------------- was it a counter overflow event? -------------------------------- */
  else if(TIM_GetITStatus(TIM10, TIM_IT_Update) == SET)
  {
    /* process the event */
    PwmoutModulePeriodCompleteInterrupt(PWMOUT_MODULE_TIM10);
  }
} /*** end of Pwmout1UpdatePwmout10Interrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer channel 2.
** \return    none.
**
****************************************************************************************/
void Pwmout2Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
  {
    /* process the event */
    PwmoutModulePeriodCompleteInterrupt(PWMOUT_MODULE_TIM2);
  }
} /*** end of Pwmout2Interrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer channel 3.
** \return    none.
**
****************************************************************************************/
void Pwmout3Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
  {
    /* process the event */
    PwmoutModulePeriodCompleteInterrupt(PWMOUT_MODULE_TIM3);
  }
} /*** end of Pwmout3Interrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer channel 4.
** \return    none.
**
****************************************************************************************/
void Pwmout4Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
  {
    /* process the event */
    PwmoutModulePeriodCompleteInterrupt(PWMOUT_MODULE_TIM4);
  }
} /*** end of Pwmout4Interrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer update channel 8 and the timer 13
**            interrupt channel.
** \return    none.
**
****************************************************************************************/
void Pwmout8UpdatePwmout13Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM8, TIM_IT_Update) == SET)
  {
    /* process the event */
    PwmoutModulePeriodCompleteInterrupt(PWMOUT_MODULE_TIM8);
  }
  /* --------------- was it a counter overflow event? -------------------------------- */
  else if(TIM_GetITStatus(TIM13, TIM_IT_Update) == SET)
  {
    /* process the event */
    PwmoutModulePeriodCompleteInterrupt(PWMOUT_MODULE_TIM13);
  }
} /*** end of Pwmout8UpdatePwmout13Interrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer channel 9.
** \return    none.
**
****************************************************************************************/
void Pwmout9Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM9, TIM_IT_Update) == SET)
  {
    /* process the event */
    PwmoutModulePeriodCompleteInterrupt(PWMOUT_MODULE_TIM9);
  }
} /*** end of Pwmout9Interrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer channel 11.
** \return    none.
**
****************************************************************************************/
void Pwmout11Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM11, TIM_IT_Update) == SET)
  {
    /* process the event */
    PwmoutModulePeriodCompleteInterrupt(PWMOUT_MODULE_TIM11);
  }
} /*** end of Pwmout11Interrupt ***/


/************************************************************************************//**
** \brief     Interrupt service routine for timer channel 14.
** \return    none.
**
****************************************************************************************/
void Pwmout14Interrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM14, TIM_IT_Update) == SET)
  {
    /* process the event */
    PwmoutModulePeriodCompleteInterrupt(PWMOUT_MODULE_TIM14);
  }
} /*** end of Pwmout14Interrupt ***/


/************************************ end of pwmout.c **********************************/


