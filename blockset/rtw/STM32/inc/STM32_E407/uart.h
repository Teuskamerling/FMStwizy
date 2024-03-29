/************************************************************************************//**
* \file         uart.h
* \brief        UART driver header file.
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
#ifndef UART_H
#define UART_H

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>                                   /* ANSI C types                  */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/* uart peripheral identifiers. note that these are used as array indexers so make
 * sure the first one has value 0 and the others are increments.
 */
#define UART_CHANNEL1_PB6_PB7     (0)            /* UART 1 - Con. 3-D1   & Con. 3-D0   */
#define UART_CHANNEL2_PD5_PD6     (1)            /* UART 2 - Con. PD-8   & Con. PD-9   */
#define UART_CHANNEL3_PD8_PD9     (2)            /* UART 3 - Con. PD-11  & Con. PD-12  */
#define UART_CHANNEL6_PC6_PC7     (3)            /* UART 6 - Con. UEXT-3 & Con. UEXT-4 */


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Configuration values for the data bits. */
typedef enum
{
  DATA_BITS_8,                                  /**< 8 data bits                       */
  DATA_BITS_9                                   /**< 9 data bits                       */
} tUartDataBitsCfg;

/** \brief Configuration values for the stop bits. */
typedef enum
{
  STOP_BITS_1,                                  /**< 1 stop bit                        */
  STOP_BITS_2                                   /**< 2 stop bits                       */
} tUartStopBitsCfg;

/** \brief Configuration values for the parity. */
typedef enum
{
  PARITY_NONE,                                  /**< no parity                         */
  PARITY_EVEN,                                  /**< even parity                       */
  PARITY_ODD                                    /**< odd parity                        */
} tUartParityCfg;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
uint8_t  UartInit(uint8_t channel, uint32_t baudrate, tUartDataBitsCfg databits,
		          tUartParityCfg parity, tUartStopBitsCfg stopbits, uint16_t txBufferSize,
		          uint16_t rxBufferSize);
uint8_t  UartPutByte(uint8_t channel, uint8_t c);
uint8_t  UartPutString(uint8_t channel, char *text);
uint8_t  UartGetByte(uint8_t channel, uint8_t *rxByte);
uint16_t UartGetNumReceivedBytes(uint8_t channel);
void     Uart1Interrupt(void);
void     Uart2Interrupt(void);
void     Uart3Interrupt(void);
void     Uart6Interrupt(void);


#endif /* UART_H */
/********************************* end of uart.h ***************************************/


