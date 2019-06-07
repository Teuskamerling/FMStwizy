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
#include "stm32f10x.h"                                /* STM32 registers               */
#include "stm32f10x_conf.h"                           /* STM32 peripheral drivers      */


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
  uint32_t timer_remap_cfg;
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
/** \brief Array with all configuration parameters of a pwm timer module. */
const static tPwmoutModuleMapping moduleMapping[] =
{
  { TIM1, RCC_APB2Periph_TIM1, 2, 0                  , TIM1_UP_IRQn },
  { TIM2, RCC_APB1Periph_TIM2, 1, 0                  , TIM2_IRQn    },
  { TIM3, RCC_APB1Periph_TIM3, 1, GPIO_FullRemap_TIM3, TIM3_IRQn    },
  { TIM4, RCC_APB1Periph_TIM4, 1, 0                  , TIM4_IRQn    }
};

/** \brief Array with all configuration parameters of a pwm output pin. */
const static tPwmoutPinMapping pinMapping[] =
{
  { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_8,  PWMOUT_MODULE_TIM1, 0 }, /* idx 0:  PWMOUT_TIM1_PIN_PA8   */
  { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_9,  PWMOUT_MODULE_TIM1, 1 }, /* idx 1:  PWMOUT_TIM1_PIN_PA9   */
  { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_10, PWMOUT_MODULE_TIM1, 2 }, /* idx 2:  PWMOUT_TIM1_PIN_PA10  */
  { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_11, PWMOUT_MODULE_TIM1, 3 }, /* idx 3:  PWMOUT_TIM1_PIN_PA11  */
  { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_0,  PWMOUT_MODULE_TIM2, 0 }, /* idx 4:  PWMOUT_TIM2_PIN_PA0   */
  { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_1,  PWMOUT_MODULE_TIM2, 1 }, /* idx 5:  PWMOUT_TIM2_PIN_PA1   */
  { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_2,  PWMOUT_MODULE_TIM2, 2 }, /* idx 6:  PWMOUT_TIM2_PIN_PA2   */
  { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_3,  PWMOUT_MODULE_TIM2, 3 }, /* idx 7:  PWMOUT_TIM2_PIN_PA3   */
  { RCC_APB2Periph_GPIOC, GPIOC, GPIO_Pin_6,  PWMOUT_MODULE_TIM3, 0 }, /* idx 8:  PWMOUT_TIM3_PIN_PC6   */
  { RCC_APB2Periph_GPIOC, GPIOC, GPIO_Pin_7,  PWMOUT_MODULE_TIM3, 1 }, /* idx 9:  PWMOUT_TIM3_PIN_PC7   */
  { RCC_APB2Periph_GPIOC, GPIOC, GPIO_Pin_8,  PWMOUT_MODULE_TIM3, 2 }, /* idx 10: PWMOUT_TIM3_PIN_PC8   */
  { RCC_APB2Periph_GPIOC, GPIOC, GPIO_Pin_9,  PWMOUT_MODULE_TIM3, 3 }, /* idx 11: PWMOUT_TIM3_PIN_PC9   */
  { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_6,  PWMOUT_MODULE_TIM4, 0 }, /* idx 12: PWMOUT_TIM4_PIN_PB6   */
  { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_7,  PWMOUT_MODULE_TIM4, 1 }, /* idx 13: PWMOUT_TIM4_PIN_PB7   */
  { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_8,  PWMOUT_MODULE_TIM4, 2 }, /* idx 14: PWMOUT_TIM4_PIN_PB8   */
  { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_9,  PWMOUT_MODULE_TIM4, 3 }  /* idx 15: PWMOUT_TIM4_PIN_PB9   */
};

/** \brief Array with all configuration parameters of a pwm output pin for the extra inverted TIM1
 *         channel outputs.
 */
const static tPwmoutPinMapping invertedPinMapping[] =
{
  { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_13, PWMOUT_MODULE_TIM1, 0 }, /* idx 0:  PWMOUT_TIM1_PIN_PA8   */
  { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_14, PWMOUT_MODULE_TIM1, 1 }, /* idx 1:  PWMOUT_TIM1_PIN_PA9   */
  { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_15, PWMOUT_MODULE_TIM1, 2 }, /* idx 2:  PWMOUT_TIM1_PIN_PA10  */
};


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Holds information about if a timer module was already configured. */
static volatile uint8_t module_initialized[] = { FALSE, FALSE, FALSE, FALSE };

/** \brief Holds information about if a PWM module's pin was already configured. */
static volatile uint8_t pin_configured[] = {
  FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE,
  FALSE, FALSE, FALSE, FALSE
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
  prescaler = (SystemCoreClock / (frequency * (PWMOUT_MAX_DUTY_CYCLE + 1))) - 1;
  /* time base configuration */
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

  /* enable the timer */
  TIM_ARRPreloadConfig(moduleMapping[module_id].timer_peripheral, ENABLE);
  TIM_Cmd(moduleMapping[module_id].timer_peripheral, ENABLE);
  TIM_CtrlPWMOutputs(moduleMapping[module_id].timer_peripheral, ENABLE); // added
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
  RCC_APB2PeriphClockCmd(pinMapping[pin_id].peripheral_clk | RCC_APB2Periph_AFIO, ENABLE);

  /* prepare pin configuration */
  gpio_init.GPIO_Pin = pinMapping[pin_id].pin;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  /* initialize the pin */
  GPIO_Init(pinMapping[pin_id].port, &gpio_init);

  /* initialize the pin for the additional inverted signal */
  if ( (inverted_output == PWMOUT_INVERTED_OUTPUT_ON) &&
       (pinMapping[pin_id].module_idx == PWMOUT_MODULE_TIM1) &&
       (pinMapping[pin_id].channel_idx <= 2) )
  {
    RCC_APB2PeriphClockCmd(invertedPinMapping[pin_id].peripheral_clk, ENABLE);
    gpio_init.GPIO_Pin = invertedPinMapping[pin_id].pin;
    GPIO_Init(invertedPinMapping[pin_id].port, &gpio_init);
  }

  /* map port pins to timer module. note that this must be done after GPIO init */
  if (moduleMapping[pinMapping[pin_id].module_idx].timer_remap_cfg != 0)
  {
    GPIO_PinRemapConfig(moduleMapping[pinMapping[pin_id].module_idx].timer_remap_cfg, ENABLE);
  }

  /* configure the timer output channel as PWM */
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
** \brief     Sets the new frequency for the timer.
** \param     tim_id Timer identifier.
** \param     frequency Frequency value
** \return    none.
**
****************************************************************************************/
void PwmoutSetFrequency(uint8_t tim_id, uint32_t frequency)
{
  /* make sure the id is valid before using it as an array indexer */
  if (!(tim_id < sizeof(moduleMapping)/sizeof(moduleMapping[0])))
  {
    ErrCodesSetError(ER_CODE_PWMOUT_INVALID_MODULE, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
    
  uint16_t prescaler = (SystemCoreClock / (frequency * (PWMOUT_MAX_DUTY_CYCLE + 1))) - 1;
  
  moduleMapping[tim_id].timer_peripheral->PSC = prescaler;
	
} /*** end of PwmoutSetFrequency ***/

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
** \brief     Interrupt service routine for timer update channel 1 
**            interrupt channel.
** \return    none.
**
****************************************************************************************/
void Pwmout1UpdateInterrupt(void)
{
  /* --------------- was it a counter overflow event? -------------------------------- */
  if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
  {
    /* process the event */
    PwmoutModulePeriodCompleteInterrupt(PWMOUT_MODULE_TIM1);
  }
} /*** end of Pwmout1UpdateInterrupt ***/


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


/************************************ end of pwmout.c **********************************/


