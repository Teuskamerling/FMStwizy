/************************************************************************************//**
* \file         uart.c
* \brief        UART driver source file.
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
#include "os.h"                                       /* for operating system          */
#include "uart.h"                                     /* UART driver header file       */
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */
#include "stm32f10x.h"                                /* STM32 registers               */
#include "stm32f10x_conf.h"                           /* STM32 peripheral drivers      */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief UART interrupt priority */
#define UART_NVIC_IRQ_PRIO               (configLIBRARY_KERNEL_INTERRUPT_PRIORITY)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Structure type with channel mapping information. */
typedef struct
{
  USART_TypeDef* peripheral;
  uint32_t uart_clk;
  uint8_t uart_apb_number;
  uint8_t uart_nvic_channel;
  uint32_t tx_port_clk;
  GPIO_TypeDef* tx_port;
  uint16_t tx_pin;
  uint32_t rx_port_clk;
  GPIO_TypeDef* rx_port;
  uint16_t rx_pin;
} tUartChannelMapping;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static portBASE_TYPE UartInterrupt(uint8_t channel);


/****************************************************************************************
* Constant data declarations
****************************************************************************************/
const static tUartChannelMapping channelMapping[] =
{
  /* idx 0: UART_CHANNEL1_PA9_PA10 */
  { USART1, RCC_APB2Periph_USART1, 2, USART1_IRQn, RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_9, RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_10, },
  /* idx 1: UART_CHANNEL2_PA2_PA3 */
  { USART2, RCC_APB1Periph_USART2, 1, USART2_IRQn, RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_2, RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_3, },
  /* idx 2: UART_CHANNEL3_PB10_PB11 */
  { USART3, RCC_APB1Periph_USART3, 1, USART3_IRQn, RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_10, RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_11, }
};


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Queue handle for the UART transmit queue(s). */
static xQueueHandle uartTransmitQ[sizeof(channelMapping)/sizeof(channelMapping[0])];

/** \brief Queue handle for the UART reception queue(s). */
static xQueueHandle uartReceptionQ[sizeof(channelMapping)/sizeof(channelMapping[0])];

/** \brief Callback functions when the UART received a byte. */
static tCallbackDataReceived callbackDataReceived[3] = {NULL, NULL, NULL};

/************************************************************************************//**
** \brief     Initializes the UART driver with the desired communication speed.
** \param     channel UART channel identifier.
** \param     baudrate Communication speed in bits per second.
** \param     databits Configuration for the number of data bits.
** \param     parity Configuration for the parity.
** \param     stopbits Configuration for the number of stop bits.
** \param     txBufferSize size of the transmit buffer in bytes.
** \param     rxBufferSize size of the reception buffer in bytes.
** \return    TRUE is successful, FALSE otherwise.
**
****************************************************************************************/
uint8_t UartInit(uint8_t channel, uint32_t baudrate, tUartDataBitsCfg databits,
		         tUartParityCfg parity, tUartStopBitsCfg stopbits, uint16_t txBufferSize,
                 uint16_t rxBufferSize)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  USART_InitTypeDef USART_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  uint8_t result = TRUE;

  /* make sure the channel is valid before using it as an array indexer */
  if (!(channel < sizeof(channelMapping)/sizeof(channelMapping[0])))
  {
    ErrCodesSetError(ER_CODE_UART_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* enable UART peripheral clock */
  if (channelMapping[channel].uart_apb_number == 1)
  {
    RCC_APB1PeriphClockCmd(channelMapping[channel].uart_clk, ENABLE);
  }
  else
  {
    RCC_APB2PeriphClockCmd(channelMapping[channel].uart_clk, ENABLE);
  }
  /* enable GPIO peripheral clock for transmitter pin */
  RCC_APB2PeriphClockCmd(channelMapping[channel].tx_port_clk | RCC_APB2Periph_AFIO, ENABLE);
  /* enable GPIO peripheral clock for receiver pin */
  RCC_APB2PeriphClockCmd(channelMapping[channel].rx_port_clk | RCC_APB2Periph_AFIO, ENABLE);
  /* configure USART Tx as alternate function push-pull */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStruct.GPIO_Pin = channelMapping[channel].tx_pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(channelMapping[channel].tx_port, &GPIO_InitStruct);
  /* Configure USART Rx as alternate function input floating */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStruct.GPIO_Pin = channelMapping[channel].rx_pin;
  GPIO_Init(channelMapping[channel].rx_port, &GPIO_InitStruct);
  /* configure UART communcation parameters */
  USART_InitStruct.USART_BaudRate = baudrate;
  /* set databits configuration */
  if (databits == DATA_BITS_8)
  {
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  }
  else
  {
    USART_InitStruct.USART_WordLength = USART_WordLength_9b;
  }
  /* set stopbits configuration */
  if (stopbits == STOP_BITS_1)
  {
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
  }
  else
  {
    USART_InitStruct.USART_StopBits = USART_StopBits_2;
  }
  /* set parity configuration */
  if (parity == PARITY_NONE)
  {
    USART_InitStruct.USART_Parity = USART_Parity_No;
  }
  else if (parity == PARITY_EVEN)
  {
    USART_InitStruct.USART_Parity = USART_Parity_Even;
  }
  else
  {
    USART_InitStruct.USART_Parity = USART_Parity_Odd;
  }

  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(channelMapping[channel].peripheral, &USART_InitStruct);
  /* create the uart transmit buffer queue */
  uartTransmitQ[channel] = xQueueCreate(txBufferSize, sizeof(uint8_t));
  if (!(uartTransmitQ[channel] != NULL))
  {
    result = FALSE;
  }
  /* create the uart reception buffer queue */
  uartReceptionQ[channel] = xQueueCreate(rxBufferSize, sizeof(uint8_t));
  if (!(uartReceptionQ[channel] != NULL))
  {
    result = FALSE;
  }
  /* configure the UART interrupt */
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = UART_NVIC_IRQ_PRIO;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  /* enable the CAN1 RX interrupts */
  NVIC_InitStruct.NVIC_IRQChannel = channelMapping[channel].uart_nvic_channel;
  NVIC_Init(&NVIC_InitStruct);
  /* enable UART */
  USART_Cmd(channelMapping[channel].peripheral, ENABLE);
  /* enable the reception interrupt */
  USART_ITConfig(channelMapping[channel].peripheral, USART_IT_RXNE, ENABLE);
  return result;
} /*** end of UartInit ***/


/************************************************************************************//**
** \brief     Outputs a byte on the UART channel in a non-blocking manner. Note that this
**            function should be called from task level, not interrupt level.
** \param     channel UART channel identifier.
** \param     byte Byte to output.
** \return    TRUE is transmit request was successfully submitted, FALSE otherwise.
**
****************************************************************************************/
uint8_t UartPutByte(uint8_t channel, uint8_t byte)
{
  /* make sure the channel is valid before using it as an array indexer */
  if (!(channel < sizeof(channelMapping)/sizeof(channelMapping[0])))
  {
	ErrCodesSetError(ER_CODE_UART_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* add the byte to the queue */
  if (xQueueSendToBack(uartTransmitQ[channel], &byte, 0) == errQUEUE_FULL)
  {
	return FALSE;
  }
  /* make sure that the transmitted empty interrupt is enabled, which kicks off the
   * data transmission.
   */
  USART_ITConfig(channelMapping[channel].peripheral, USART_IT_TXE, ENABLE);
  /* transmit request successfully transmitted */
  return TRUE;
} /*** end of UartPutByte ***/


/************************************************************************************//**
** \brief     Outputs a line of text on the UART channel. Note that this function should
**            be called from task level, not interrupt level.
** \param     channel UART channel identifier.
** \param     text Pointer to the NUL-terminated character array.
** \return    none
**
****************************************************************************************/
uint8_t UartPutString(uint8_t channel, char *text)
{
  uint8_t status = TRUE;
    
  /* make sure the channel is valid before using it as an array indexer */
  if (!(channel < sizeof(channelMapping)/sizeof(channelMapping[0])))
  {
    ErrCodesSetError(ER_CODE_UART_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    status = FALSE;
  }
  /* output the characters of the string one-by-one */
  while (*text != '\0')
  {
    if (UartPutByte(channel, *text++) == FALSE)
    {
      ErrCodesSetError(ER_CODE_UART_TRANSMIT_QUEUE_FULL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
      status = FALSE;
    }
  }
  return status;
} /*** end of UartPutString ***/

/************************************************************************************//**
** \brief     Outputs an array of bytes on the UART channel. Note that this function
**            shouldbe called from task level, not interrupt level.
** \param     channel UART channel identifier.
** \param     text Pointer to the NUL-terminated character array.
** \param     n The number of bytes that must be transmitted.
** \return    none
**
****************************************************************************************/
void UartPutBytes(uint8_t channel, char *bytes, const uint16_t n)
{
  /* make sure the channel is valid before using it as an array indexer */
  if (!(channel < sizeof(channelMapping)/sizeof(channelMapping[0])))
  {
    ErrCodesSetError(ER_CODE_UART_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* output the bytes in the array one-by-one */
  uint16_t i=0;
  while (i < n)
  {
    if (UartPutByte(channel, bytes[i]) == FALSE)
    {
      ErrCodesSetError(ER_CODE_UART_TRANSMIT_QUEUE_FULL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }
    
    ++i;
  }
} /*** end of UartPutString ***/

/************************************************************************************//**
** \brief     Obtains the oldest byte from the reception buffer.
** \param     channel UART channel identifier.
** \param     rxByte Pointer to where the read byte should be stored.
** \return    TRUE is a byte was read from the buffer, FALSE if the buffer was empty.
**
****************************************************************************************/
uint8_t UartGetByte(uint8_t channel, uint8_t *rxByte)
{
  /* make sure the channel is valid before using it as an array indexer */
  if (!(channel < sizeof(channelMapping)/sizeof(channelMapping[0])))
  {
    ErrCodesSetError(ER_CODE_UART_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* read the next byte from the queue */
  if (xQueueReceive(uartReceptionQ[channel], (void *)rxByte, 0) == pdTRUE)
  {
	/* successfully read byte from the buffer */
    return TRUE;
  }
  /* still here so buffer was empty */
  return FALSE;


} /*** end of UartGetByte ***/


/************************************************************************************//**
** \brief     Obtains the number of received bytes currently stored in the internal
**            reception buffer. Note that this function should be called from task level,
**            not interrupt level.
** \param     channel UART channel identifier.
** \return    The number of received bytes that are currently stored in the internal
**            reception buffer.
**
****************************************************************************************/
uint16_t UartGetNumReceivedBytes(uint8_t channel)
{
  unsigned portBASE_TYPE numBytesInQueue;

  /* make sure the channel is valid before using it as an array indexer */
  if (!(channel < sizeof(channelMapping)/sizeof(channelMapping[0])))
  {
    ErrCodesSetError(ER_CODE_UART_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* read the number of items in the buffer */
  numBytesInQueue = uxQueueMessagesWaitingFromISR(uartReceptionQ[channel]);
  /* return the results */
  return (uint16_t)numBytesInQueue;
} /*** end of UartGetNumReceivedBytes ***/


/************************************************************************************//**
** \brief     UART channel generic interrupt service routine.
** \param     channel UART channel identifier.
** \return    pdTRUE is a higher priority task was woken, pdFALSE otherwise.
**
****************************************************************************************/
static portBASE_TYPE UartInterrupt(uint8_t channel)
{
  portBASE_TYPE xHigherPrioTaskWoken = pdFALSE;
  portBASE_TYPE result;
  uint8_t uartByte;

  /* check if this interrupt was caused by the transmitter */
  if (USART_GetITStatus(channelMapping[channel].peripheral, USART_IT_TXE) == SET)
  {
    /* read the next byte from the queue */
    result = xQueueReceiveFromISR(uartTransmitQ[channel], (void *)&uartByte, &xHigherPrioTaskWoken);
    /* in case the read fails, the last byte from the buffer was just sent and the transmit
     * interrupt should be disabled.
     */
    if (!(result == pdTRUE))
    {
      /* disable the transmit interrupt */
      USART_ITConfig(channelMapping[channel].peripheral, USART_IT_TXE, DISABLE);
    }
    else
    {
      /* transmit the byte */
      USART_SendData(channelMapping[channel].peripheral, uartByte);
    }
  }
  /* check if this interrupt was caused by the receiver */
  else if (USART_GetITStatus(channelMapping[channel].peripheral, USART_IT_RXNE) == SET)
  {
    /* read the newly received byte */
	uartByte = (uint8_t)USART_ReceiveData(channelMapping[channel].peripheral);
    
    /* add the byte to the queue */
    if (xQueueSendToBackFromISR(uartReceptionQ[channel], &uartByte, &xHigherPrioTaskWoken) == errQUEUE_FULL)
    {
      ErrCodesSetError(ER_CODE_UART_RECEPTION_QUEUE_FULL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }
    
    /* Call the data received callback function                    */
    /* no parameters are provided, the callback function must use  */
    /* UartGetNumReceivedBytes() to check if sufficient bytes were */
    /* receivedand and UartGetByte() to get the bytes.             */
    
    /* Only call the callback function if it has been registered   */
    if(callbackDataReceived[channel] != NULL)
    {
      callbackDataReceived[channel]();
    }
  }

  /* pass on info to the caller about whether or not a higher priority task was woken */
  return xHigherPrioTaskWoken;
} /*** end of UartInterrupt ***/


/************************************************************************************//**
** \brief     UART1 Interrupt Service Routine.
** \return    none.
**
****************************************************************************************/
void Uart1Interrupt(void)
{
  portBASE_TYPE xHigherPrioTaskWoken;

  /* call the interrupt handler for this channel */
  xHigherPrioTaskWoken = UartInterrupt(UART_CHANNEL1_PA9_PA10);
  /* should the context be switched so the ISR returns to a different task? */
  portEND_SWITCHING_ISR(xHigherPrioTaskWoken);
} /*** end of Uart1Interrupt ***/


/************************************************************************************//**
** \brief     UART2 Interrupt Service Routine.
** \return    none.
**
****************************************************************************************/
void Uart2Interrupt(void)
{
  portBASE_TYPE xHigherPrioTaskWoken;

  /* call the interrupt handler for this channel */
  xHigherPrioTaskWoken = UartInterrupt(UART_CHANNEL2_PA2_PA3);
  /* should the context be switched so the ISR returns to a different task? */
  portEND_SWITCHING_ISR(xHigherPrioTaskWoken);
} /*** end of Uart2Interrupt ***/


/************************************************************************************//**
** \brief     UART3 Interrupt Service Routine.
** \return    none.
**
****************************************************************************************/
void Uart3Interrupt(void)
{
  portBASE_TYPE xHigherPrioTaskWoken;

  /* call the interrupt handler for this channel */
  xHigherPrioTaskWoken = UartInterrupt(UART_CHANNEL3_PB10_PB11);
  /* should the context be switched so the ISR returns to a different task? */
  portEND_SWITCHING_ISR(xHigherPrioTaskWoken);
} /*** end of Uart3Interrupt ***/

/************************************************************************************//**
** \brief     Registers the callback function that is to be called whenever new data is
**            received by the UART.
** \param     callbackPtr Pointer to the callback function.
** \return    none.
**
****************************************************************************************/
void UartRegisterDataReceivedCallback(uint8_t channel, tCallbackDataReceived callbackPtr)
{
  /* set the function callback pointer */
  callbackDataReceived[channel] = callbackPtr;
} /*** end of UartRegisterDataReceivedCallback ***/

/************************************ end of uart.c ************************************/


