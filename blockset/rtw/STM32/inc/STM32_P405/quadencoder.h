/************************************************************************************//**
* \file         quadencoder.h
* \brief        Quadrature encoder inputs driver header file.
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
 \endinternal
****************************************************************************************/
#ifndef QUADENC_H
#define QUADENC_H

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>                                   /* ANSI C types                  */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/* quadrature module identifiers. note that these are used as array indexers so make
 * sure the first one has value 0 and the others are increments. Note that the pins
 * are 5V tolerant.
 */
#define QUADENC_TIM1_PA8_PA9     (0)             /* Connector PA8 / UEXT-3             */
#define QUADENC_TIM3_PC6_PC7     (1)             /* Connector PC6 / PC7                */
#define QUADENC_TIM4_PB6_PB7     (2)             /* Connector UEXT-5 / UEXT-6         */


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief List of quad encoder input pin configurations. */
typedef enum
{
  QUADENC_CFG_FLOATING,                         /**< floating pin configuration        */
  QUADENC_CFG_PULLUP,                           /**< enable internal pullup resistor   */
  QUADENC_CFG_PULLDOWN                          /**< enable internal pulldown resistor */
} tQuadEncCfg;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void    QuadEncConfigure(uint8_t id, tQuadEncCfg config);
int32_t QuadEncGetCounter(uint8_t id);
void    QuadEncResetCounter(uint8_t id);
void    QuadEncUpdate(void);


#endif /* QUADENC_H */
/********************************* end of quadencoder.h ********************************/


