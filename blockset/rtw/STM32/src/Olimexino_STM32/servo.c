/************************************************************************************//**
* \file         servo.c
* \brief        Servo driver source file.
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

/*
 * f_TIM = 1 MHz
 *
 *       CNT
 *        |
 *        |
 *    ARR |
 *   19999+-----------------------------------------------------------.--------
 *        |                                                        .  .
 *        |                                                     .     .
 *        |                                                  .        .
 *        |                                               .           .
 *        |                                            .              .
 *        |                                         .                 .
 *        |                                      .                    .
 *        |                                   .                       .
 *        |                                .                          .
 *        |                             .                             .
 *        |                          .                                .
 *        |                       .                                   .
 *        |                    .                                      .
 *        |                 .                                         .
 *        |              .                                            .
 *    2000+ - - - - - .                                               .
 *        |        .  |                                               .
 *    1000+- - -.- - - - - - - - - - - - - - - - - - - - - - - - - - -.- - -.-
 *        |  .  |     |                                               .  .  |
 *       0.-----+-----------------------------------------------------.--------
 *
 *        +-----+-----+                                               +-----+
 *        |     |     |                                               |     |
 *        |     |     |                                               |     |
 * OC pin +     +-----+-----------------------------------------------+     +--
 *
 *        |<--->| min_pulse_width_in_us, typically 1000 us
 *
 *        |<--------->| max_pulse_width_in_us, typically 2000 us
 *
 *        |<------------------------- 20 ms ------------------------->|
 */

/****************************************************************************************
* Include files
****************************************************************************************/
#include "servo.h"
#include "stdio.h"
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */

/****************************************************************************************
* Macro definitions
****************************************************************************************/

/****************************************************************************************
* Type definitions
****************************************************************************************/
//#define ERROR_CHECK_ENABLED

typedef struct
{
	TIM_TypeDef *tim;
	uint32_t rcc_tim_clk;
	void (*rcc_apb_periph_clock_cmd_tim)(uint32_t, FunctionalState);
	// ------------------------------------------------------------------------
	uint32_t remap;
	void (*tim_ctrl_pwm_outputs)(TIM_TypeDef*, FunctionalState);

} tTimerMapping;

typedef struct
{
	void (*tim_oc_init)(TIM_TypeDef*, TIM_OCInitTypeDef*);
	void (*tim_oc_preload_config)(TIM_TypeDef*, uint16_t);
	void (*tim_set_compare)(TIM_TypeDef*, uint16_t);

} tCannelMapping;

typedef struct
{
	uint8_t timer_mapping_idx;
	// ------------------------------------------------------------------------
	uint8_t channel_mapping_idx;
	// ------------------------------------------------------------------------
	GPIO_TypeDef *gpio_port;
	uint16_t gpio_oc_pin;
	uint32_t rcc_gpio_port_clk;
	void (*rcc_apb_periph_clock_cmd_gpio)(uint32_t, FunctionalState);

} tPinMapping;

/****************************************************************************************
* External function prototypes
****************************************************************************************/

/****************************************************************************************
* Function prototypes
****************************************************************************************/

/****************************************************************************************
* Constant data declarations
****************************************************************************************/
/** \brief Array with all timer specific configuration parameters of a servo module. */
const static tTimerMapping timerMapping[] =
{
	{ TIM1, RCC_APB2Periph_TIM1, RCC_APB2PeriphClockCmd, 0,                   TIM_CtrlPWMOutputs},
	{ TIM2, RCC_APB1Periph_TIM2, RCC_APB1PeriphClockCmd, 0,                   NULL              },
	{ TIM3, RCC_APB1Periph_TIM3, RCC_APB1PeriphClockCmd, GPIO_FullRemap_TIM3, NULL              },
	{ TIM4, RCC_APB1Periph_TIM4, RCC_APB1PeriphClockCmd, 0,                   NULL              }
};

/** \brief Array with all output compare specific configuration parameters of a servo module. */
const static tCannelMapping channelMapping[] =
{
	{ TIM_OC1Init, TIM_OC1PreloadConfig, TIM_SetCompare1},
	{ TIM_OC2Init, TIM_OC2PreloadConfig, TIM_SetCompare2},
	{ TIM_OC3Init, TIM_OC3PreloadConfig, TIM_SetCompare3},
	{ TIM_OC4Init, TIM_OC4PreloadConfig, TIM_SetCompare4}
};

/** \brief Array with all configuration parameters of a servo module. */
const static tPinMapping pinMapping[] =
{
	{ // idx 0: SERVO_TIM1_PIN_PA8
		0, // TIM1
		0, // Channel 1
		GPIOA, GPIO_Pin_8, RCC_APB2Periph_GPIOA, RCC_APB2PeriphClockCmd,
	},
	{ // idx 1: SERVO_TIM1_PIN_PA9
		0, // TIM1
		1, // Channel 2
		GPIOA, GPIO_Pin_9, RCC_APB2Periph_GPIOA, RCC_APB2PeriphClockCmd,
	},
	{ // idx 2: SERVO_TIM1_PIN_PA10
		0, // TIM1
		2, // Channel 3
		GPIOA, GPIO_Pin_10, RCC_APB2Periph_GPIOA, RCC_APB2PeriphClockCmd,
	},

	{ // idx 3: SERVO_TIM2_PIN_PA0
		1, // TIM2
		0, // Channel 1
		GPIOA, GPIO_Pin_0, RCC_APB2Periph_GPIOA, RCC_APB2PeriphClockCmd,
	},
	{ // idx 4: SERVO_TIM2_PIN_PA1
		1, // TIM2
		1, // Channel 2
		GPIOA, GPIO_Pin_1, RCC_APB2Periph_GPIOA, RCC_APB2PeriphClockCmd,
	},
	{ // idx 5: SERVO_TIM2_PIN_PA3
		1, // TIM2
		3, // Channel 4
		GPIOA, GPIO_Pin_3, RCC_APB2Periph_GPIOA, RCC_APB2PeriphClockCmd,
	},

	{ // idx 6: SERVO_TIM3_PIN_PC6
		2, // TIM3
		0, // Channel 1
		GPIOC, GPIO_Pin_6, RCC_APB2Periph_GPIOC, RCC_APB2PeriphClockCmd,
	},
	{ // idx 7: SERVO_TIM3_PIN_PC7
		2, // TIM3
		1, // Channel 2
		GPIOC, GPIO_Pin_7, RCC_APB2Periph_GPIOC, RCC_APB2PeriphClockCmd,
	},
	{ // idx 8: SERVO_TIM3_PIN_PC8
		2, // TIM3
		2, // Channel 3
		GPIOC, GPIO_Pin_8, RCC_APB2Periph_GPIOC, RCC_APB2PeriphClockCmd,
	},

	{ // idx 9: SERVO_TIM4_PIN_PB6
		3, // TIM4
		0, // Channel 1
		GPIOB, GPIO_Pin_6, RCC_APB2Periph_GPIOB, RCC_APB2PeriphClockCmd,
	},
	{ // idx 10: SERVO_TIM4_PIN_PB7
		3, // TIM4
		1, // Channel 2
		GPIOB, GPIO_Pin_7, RCC_APB2Periph_GPIOB, RCC_APB2PeriphClockCmd,
	},
	{ // idx 11: SERVO_TIM4_PIN_PB8
		3, // TIM4
		2, // Channel 3
		GPIOB, GPIO_Pin_8, RCC_APB2Periph_GPIOB, RCC_APB2PeriphClockCmd,
	},
	{ // idx 12: SERVO_TIM4_PIN_PB9
		3, // TIM4
		3, // Channel 4
		GPIOB, GPIO_Pin_9, RCC_APB2Periph_GPIOB, RCC_APB2PeriphClockCmd,
	},

};

/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Array with all minimum pulse widths for each servo module. */
static uint16_t m_min_pulse_width_in_us[12] =
{
	1000,1000,1000,1000,
	1000,1000,1000,1000,
	1000,1000,1000,1000
};

/** \brief Array with all maximum pulse widths for each servo module. */
static uint16_t m_max_pulse_width_in_us[12] =
{
	2000,2000,2000,2000,
	2000,2000,2000,2000,
	2000,2000,2000,2000
};

/************************************************************************************//**
** \brief     Initializes a servo module.
** \param     pinID Pin identifier.
** \param     min_pulse_width_in_us Minimum pulse width in microseconds.
** \param     max_pulse_width_in_us Maximum pulse width in microseconds.
** \return    none.
**
****************************************************************************************/
void ServoInit(const uint8_t pinID, const uint16_t min_pulse_width_in_us, const uint16_t max_pulse_width_in_us)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef       TIM_OCInitStructure;

    // make sure the id is valid before using it as an array indexer
    if (!(pinID < sizeof(pinMapping)/sizeof(pinMapping[0])))
    {
      ErrCodesSetError(ER_CODE_SERVO_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }
   
	// Save the parameters to global variables
	m_min_pulse_width_in_us[pinID] = min_pulse_width_in_us;
	m_max_pulse_width_in_us[pinID] = max_pulse_width_in_us;

	// Temporary local variables
	register uint32_t t_idx = pinMapping[pinID].timer_mapping_idx;
	register uint32_t c_idx = pinMapping[pinID].channel_mapping_idx;

	//[..] To use the Timer in Output Compare mode, the following steps are
	//     mandatory:
	//(#) Enable TIM clock using
	//    RCC_APBxPeriphClockCmd(RCC_APBxPeriph_TIMx, ENABLE) function.

	timerMapping[t_idx].rcc_apb_periph_clock_cmd_tim(timerMapping[t_idx].rcc_tim_clk, ENABLE);

	//(#) Configure the TIM pins by configuring the corresponding GPIO pins
	pinMapping[pinID].rcc_apb_periph_clock_cmd_gpio(pinMapping[pinID].rcc_gpio_port_clk, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = pinMapping[pinID].gpio_oc_pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(pinMapping[pinID].gpio_port, &GPIO_InitStructure);

	// Remap I/O pins if applicable
	if(timerMapping[t_idx].remap != 0)
	{
		GPIO_PinRemapConfig(timerMapping[t_idx].remap, ENABLE);
	}

	//(#) Configure the Time base unit as described in the first part of this
	//    driver, if needed, else the Timer will run with the default
	//    configuration:
	//    (++) Autoreload value = 0xFFFF.
	//    (++) Prescaler value = 0x0000.
	//    (++) Counter mode = Up counting.
	//    (++) Clock Division = TIM_CKD_DIV1.
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(timerMapping[t_idx].tim, &TIM_TimeBaseStructure);

	//(#) Fill the TIM_OCInitStruct with the desired parameters including:
	//    (++) The TIM Output Compare mode: TIM_OCMode.
	//    (++) TIM Output State: TIM_OutputState.
	//    (++) TIM Pulse value: TIM_Pulse.
	//    (++) TIM Output Compare Polarity : TIM_OCPolarity.
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse = m_min_pulse_width_in_us[pinID];
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//(#) Call TIM_OCxInit(TIMx, &TIM_OCInitStruct) to configure the desired
	//    channel with the corresponding configuration.
	channelMapping[c_idx].tim_oc_init(timerMapping[t_idx].tim, &TIM_OCInitStructure);
	channelMapping[c_idx].tim_oc_preload_config(timerMapping[t_idx].tim, TIM_OCPreload_Enable);

	// Enable PWM outputs if applicable
	if(timerMapping[t_idx].tim_ctrl_pwm_outputs != NULL)
	{
		timerMapping[t_idx].tim_ctrl_pwm_outputs(timerMapping[t_idx].tim, ENABLE);
	}

	//(#) Call the TIM_Cmd(ENABLE) function to enable the TIM counter.
	TIM_Cmd(timerMapping[t_idx].tim, ENABLE);
}

/************************************************************************************//**
** \brief     Updates the pulse width.
** \param     pinID Pin identifier.
** \param     pulse_width_in_us New pulse width in microsecond.
** \return    none.
**
****************************************************************************************/
void ServoUpdate(const uint8_t pinID, uint16_t pulse_width_in_us)
{
    // make sure the id is valid before using it as an array indexer
    if (!(pinID < sizeof(pinMapping)/sizeof(pinMapping[0])))
    {
      ErrCodesSetError(ER_CODE_SERVO_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }
    
	// Verify input parameter
	if(pulse_width_in_us < m_min_pulse_width_in_us[pinID])
	{
		pulse_width_in_us = m_min_pulse_width_in_us[pinID];
	}

	if(pulse_width_in_us > m_max_pulse_width_in_us[pinID])
	{
		pulse_width_in_us = m_max_pulse_width_in_us[pinID];
	}
	
	// Set the new compare value
	channelMapping[pinMapping[pinID].channel_mapping_idx].tim_set_compare(timerMapping[pinMapping[pinID].timer_mapping_idx].tim, pulse_width_in_us);
}

/********************************* end of servo.c *************************************/