/************************************************************************************//**
* \file         digin.h
* \brief        Digital inputs driver header file.
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
#ifndef DIGIN_H
#define DIGIN_H

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>                                   /* ANSI C types                  */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/* digital input port identifiers. note that these are used as array indexers so make
 * sure the first one has value 0 and the others are increments.
 */
#define DIGIN_PORTA_PIN1    (0)                        /* PA1                          */
#define DIGIN_PORTA_PIN8    (1)                        /* PA8                          */
#define DIGIN_PORTB_PIN0    (2)                        /* PB0                          */
#define DIGIN_PORTB_PIN1    (3)                        /* PB1                          */
#define DIGIN_PORTB_PIN2    (4)                        /* PB2                          */
#define DIGIN_PORTB_PIN5    (5)                        /* PB5                          */
#define DIGIN_PORTB_PIN8    (6)                        /* PB8                          */
#define DIGIN_PORTB_PIN9    (7)                        /* PB9                          */
#define DIGIN_PORTB_PIN10   (8)                        /* PB10                         */
#define DIGIN_PORTB_PIN11   (9)                        /* PB11                         */
#define DIGIN_PORTB_PIN12   (10)                       /* PB12                         */
#define DIGIN_PORTB_PIN13   (11)                       /* PB13                         */
#define DIGIN_PORTB_PIN14   (12)                       /* PB14                         */
#define DIGIN_PORTB_PIN15   (13)                       /* PB15                         */
#define DIGIN_PORTC_PIN0    (14)                       /* PC0                          */
#define DIGIN_PORTC_PIN1    (15)                       /* PC1                          */
#define DIGIN_PORTC_PIN2    (16)                       /* PC2                          */
#define DIGIN_PORTC_PIN3    (17)                       /* PC3                          */
#define DIGIN_PORTC_PIN4    (18)                       /* PC4                          */
#define DIGIN_PORTC_PIN5    (19)                       /* PC5                          */
#define DIGIN_PORTC_PIN6    (20)                       /* PC6                          */
#define DIGIN_PORTC_PIN7    (21)                       /* PC7                          */
#define DIGIN_PORTC_PIN8    (22)                       /* PC8                          */
#define DIGIN_PORTC_PIN9    (23)                       /* PC9                          */
#define DIGIN_PORTC_PIN10   (24)                       /* PC10                         */
#define DIGIN_PORTC_PIN11   (25)                       /* PC11                         */
#define DIGIN_PORTC_PIN12   (26)                       /* PC12                         */
#define DIGIN_PORTC_PIN13   (27)                       /* PC13                         */
#define DIGIN_PORTD_PIN2    (28)                       /* PD2                          */
#define DIGIN_PORTA_PIN0    (29)                       /* PA0/BUTTON                   */


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief List of digital input pin configurations. */
typedef enum
{
  DIGIN_CFG_FLOATING,                           /**< floating pin configuration        */
  DIGIN_CFG_PULLUP,                             /**< enable internal pullup resistor   */
  DIGIN_CFG_PULLDOWN                            /**< enable internal pulldown resistor */
} tDiginCfg;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void    DiginConfigure(uint8_t id, tDiginCfg config);
uint8_t DiginGet(uint8_t id);


#endif /* DIGIN_H */
/********************************* end of digin.h **************************************/


