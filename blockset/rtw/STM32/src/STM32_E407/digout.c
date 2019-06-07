/************************************************************************************//**
* \file         digout.c
* \brief        Digital outputs driver source file.
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
#include "digout.h"                                   /* Digital output driver         */
#include "os.h"                                       /* for operating system          */
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */
#include "stm32f4xx.h"                                /* STM32 registers               */
#include "stm32f4xx_conf.h"                           /* STM32 peripheral drivers      */


/****************************************************************************************
* Type definitions
****************************************************************************************/
/**< \brief Structure type with pin mapping information. */
typedef struct
{
  uint32_t peripheral;
  GPIO_TypeDef* port;
  uint16_t pin;
} tDigoutPinMapping;


/****************************************************************************************
* Constant data declarations
****************************************************************************************/
const static tDigoutPinMapping pinMapping[] =
{
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_0  },          /* idx  0: DIGOUT_PORTC_PIN0  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_6  },          /* idx  1: DIGOUT_PORTF_PIN6  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_7  },          /* idx  2: DIGOUT_PORTF_PIN7  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_8  },          /* idx  3: DIGOUT_PORTF_PIN8  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_9  },          /* idx  4: DIGOUT_PORTF_PIN9  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_10 },          /* idx  5: DIGOUT_PORTF_PIN10 */
  { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_7  },          /* idx  6: DIGOUT_PORTB_PIN7  */
  { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_6  },          /* idx  7: DIGOUT_PORTB_PIN6  */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_2  },          /* idx  8: DIGOUT_PORTE_PIN2  */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_4  },          /* idx  9: DIGOUT_PORTE_PIN4  */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_5  },          /* idx 10: DIGOUT_PORTE_PIN5  */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_6  },          /* idx 11: DIGOUT_PORTE_PIN6  */
  { RCC_AHB1Periph_GPIOG, GPIOG, GPIO_Pin_7  },          /* idx 12: DIGOUT_PORTG_PIN7  */
  { RCC_AHB1Periph_GPIOG, GPIOG, GPIO_Pin_8  },          /* idx 13: DIGOUT_PORTG_PIN8  */
  { RCC_AHB1Periph_GPIOG, GPIOG, GPIO_Pin_12 },          /* idx 14: DIGOUT_PORTG_PIN12 */
  { RCC_AHB1Periph_GPIOG, GPIOG, GPIO_Pin_15 },          /* idx 15: DIGOUT_PORTG_PIN15 */
  { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_4  },          /* idx 16: DIGOUT_PORTA_PIN4  */
  { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_5  },          /* idx 17: DIGOUT_PORTB_PIN5  */
  { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_6  },          /* idx 18: DIGOUT_PORTA_PIN6  */
  { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_5  },          /* idx 19: DIGOUT_PORTA_PIN5  */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_0  },          /* idx 20: DIGOUT_PORTD_PIN0  */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_1  },          /* idx 21: DIGOUT_PORTD_PIN1  */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_2  },          /* idx 22: DIGOUT_PORTD_PIN2  */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_3  },          /* idx 23: DIGOUT_PORTD_PIN3  */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_4  },          /* idx 24: DIGOUT_PORTD_PIN4  */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_5  },          /* idx 25: DIGOUT_PORTD_PIN5  */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_6  },          /* idx 26: DIGOUT_PORTD_PIN6  */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_7  },          /* idx 27: DIGOUT_PORTD_PIN7  */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_8  },          /* idx 28: DIGOUT_PORTD_PIN8  */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_9  },          /* idx 29: DIGOUT_PORTD_PIN9  */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_10 },          /* idx 30: DIGOUT_PORTD_PIN10 */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_11 },          /* idx 31: DIGOUT_PORTD_PIN11 */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_12 },          /* idx 32: DIGOUT_PORTD_PIN12 */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_13 },          /* idx 33: DIGOUT_PORTD_PIN13 */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_14 },          /* idx 34: DIGOUT_PORTD_PIN14 */
  { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_15 },          /* idx 35: DIGOUT_PORTD_PIN15 */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_0  },          /* idx 36: DIGOUT_PORTE_PIN0  */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_1  },          /* idx 37: DIGOUT_PORTE_PIN1  */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_3  },          /* idx 38: DIGOUT_PORTE_PIN3  */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_7  },          /* idx 39: DIGOUT_PORTE_PIN7  */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_8  },          /* idx 40: DIGOUT_PORTE_PIN8  */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_9  },          /* idx 41: DIGOUT_PORTE_PIN9  */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_10 },          /* idx 42: DIGOUT_PORTE_PIN10 */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_11 },          /* idx 43: DIGOUT_PORTE_PIN11 */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_12 },          /* idx 44: DIGOUT_PORTE_PIN12 */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_13 },          /* idx 45: DIGOUT_PORTE_PIN13 */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_14 },          /* idx 46: DIGOUT_PORTE_PIN14 */
  { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_15 },          /* idx 47: DIGOUT_PORTE_PIN15 */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_0  },          /* idx 48: DIGOUT_PORTF_PIN0  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_1  },          /* idx 49: DIGOUT_PORTF_PIN1  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_2  },          /* idx 50: DIGOUT_PORTF_PIN2  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_3  },          /* idx 51: DIGOUT_PORTF_PIN3  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_4  },          /* idx 52: DIGOUT_PORTF_PIN4  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_5  },          /* idx 53: DIGOUT_PORTF_PIN5  */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_11 },          /* idx 54: DIGOUT_PORTF_PIN11 */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_12 },          /* idx 55: DIGOUT_PORTF_PIN12 */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_13 },          /* idx 56: DIGOUT_PORTF_PIN13 */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_14 },          /* idx 57: DIGOUT_PORTF_PIN14 */
  { RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_15 },          /* idx 58: DIGOUT_PORTF_PIN15 */
  { RCC_AHB1Periph_GPIOG, GPIOG, GPIO_Pin_0  },          /* idx 59: DIGOUT_PORTG_PIN0  */
  { RCC_AHB1Periph_GPIOG, GPIOG, GPIO_Pin_1  },          /* idx 60: DIGOUT_PORTG_PIN1  */
  { RCC_AHB1Periph_GPIOG, GPIOG, GPIO_Pin_2  },          /* idx 61: DIGOUT_PORTG_PIN2  */
  { RCC_AHB1Periph_GPIOG, GPIOG, GPIO_Pin_3  },          /* idx 62: DIGOUT_PORTG_PIN3  */
  { RCC_AHB1Periph_GPIOG, GPIOG, GPIO_Pin_4  },          /* idx 63: DIGOUT_PORTG_PIN4  */
  { RCC_AHB1Periph_GPIOG, GPIOG, GPIO_Pin_5  },          /* idx 64: DIGOUT_PORTG_PIN5  */
  { RCC_AHB1Periph_GPIOG, GPIOG, GPIO_Pin_6  },          /* idx 65: DIGOUT_PORTG_PIN6  */
  { RCC_AHB1Periph_GPIOG, GPIOG, GPIO_Pin_9  },          /* idx 66: DIGOUT_PORTG_PIN9  */
  { RCC_AHB1Periph_GPIOG, GPIOG, GPIO_Pin_10 },          /* idx 67: DIGOUT_PORTG_PIN10 */
  { RCC_AHB1Periph_GPIOG, GPIOG, GPIO_Pin_11 },          /* idx 68: DIGOUT_PORTG_PIN11 */
  { RCC_AHB1Periph_GPIOG, GPIOG, GPIO_Pin_13 },          /* idx 69: DIGOUT_PORTG_PIN13 */
  { RCC_AHB1Periph_GPIOG, GPIOG, GPIO_Pin_14 },          /* idx 70: DIGOUT_PORTG_PIN14 */
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_13 }           /* idx 71: DIGOUT_PORTC_PIN13 */
};


/************************************************************************************//**
** \brief     Configures a digital output pin.
** \param     id Pin identifier.
** \param     config Pin configuration (DIGOUT_CFG_xxx).
** \return    none.
**
****************************************************************************************/
void DigoutConfigure(uint8_t id, tDigoutCfg config)
{
  GPIO_InitTypeDef gpio_init;

  /* make sure the id is valid before using it as an array indexer */
  if (!(id < sizeof(pinMapping)/sizeof(pinMapping[0])))
  {
    ErrCodesSetError(ER_CODE_DIGOUT_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* enable the port's peripheral clock */
  RCC_AHB1PeriphClockCmd(pinMapping[id].peripheral, ENABLE);
  /* prepare pin configuration */
  gpio_init.GPIO_Pin = pinMapping[id].pin;
  gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
  gpio_init.GPIO_Mode = GPIO_Mode_OUT;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  if (config == DIGOUT_CFG_OPENDRAIN)
  {
    gpio_init.GPIO_OType = GPIO_OType_OD;
  }
  else
  {
    gpio_init.GPIO_OType = GPIO_OType_PP;
  }
  /* initialize the pin */
  GPIO_Init(pinMapping[id].port, &gpio_init);
} /*** end of DigoutConfigure ***/


/************************************************************************************//**
** \brief     Sets the state of a digital output pin.
** \param     id Pin identifier.
** \param     state New state for the digital output pin.
** \return    none.
**
****************************************************************************************/
void DigoutSet(uint8_t id, tDigoutState state)
{
  BitAction bitVal = Bit_RESET;

  /* make sure the id is valid before using it as an array indexer */
  if (!(id < sizeof(pinMapping)/sizeof(pinMapping[0])))
  {
    ErrCodesSetError(ER_CODE_DIGOUT_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* determine the pin state */
  if (state == DIGOUT_HIGH)
  {
    bitVal = Bit_SET;
  }
  /* set the new pin state */
  GPIO_WriteBit(pinMapping[id].port, pinMapping[id].pin, bitVal);
} /*** end of DigoutSet ***/


/************************************ end of digout.c **********************************/


