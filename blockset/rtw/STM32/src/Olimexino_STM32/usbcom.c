/************************************************************************************//**
* \file         usbcom.c
* \brief        USB virtual COM-port source file.
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
#include "usbcom.h"                                   /* USB COM-port header file      */
#include "usb_lib.h"                                  /* USB library driver header     */
#include "usb_desc.h"                                 /* USB descriptor header         */
#include "usb_pwr.h"                                  /* USB power management header   */
#include "usb_istr.h"                                 /* USB interrupt routine header  */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Size of the buffer for storing data that is to be sent to the PC */
#define USART_RX_DATA_SIZE        (512)

/* usb interrupt priority */
#define USB_COM_NVIC_IRQ_PRIO     (configLIBRARY_KERNEL_INTERRUPT_PRIORITY)


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void UsbComIntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);


/****************************************************************************************
* Global data declarations
****************************************************************************************/
/** \brief Rx buffer. Basically the virtual COM-port buffer for storing data that
 *         is to be sent to the PC. The name might be confusing because it came from
 *         an example that was a UART-USB gateway.
 */
uint8_t  USART_Rx_Buffer[USART_RX_DATA_SIZE];

/** \brief Rx buffer write pointer. */
uint32_t USART_Rx_ptr_in = 0;

/** \brief Rx buffer read pointer. */
uint32_t USART_Rx_ptr_out = 0;

/** \brief Number of bytes currently stored in the Rx buffer */
uint32_t USART_Rx_length  = 0;

/** \brief State of the USB transmit path. */
uint8_t  USB_Tx_State = 0;


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Function pointer to the data reception callback queue. */
static tUsbComCallbackDataReceived usbComCallbackDataReceived = NULL;

/** \brief Variable to keep track if this module is initialized. Needed for the ISR
 *         muxing with the CAN module.
 */
static uint8_t usbInitialized = FALSE;


/************************************************************************************//**
** \brief     Initializes the USB virtual COM-port driver.
** \return    none.
**
****************************************************************************************/
void UsbComInit(void)
{
  GPIO_InitTypeDef  gpio_init;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* reset the function callback pointer */
  usbComCallbackDataReceived = NULL;
  /* enable clock for PC12 pin peripheral (GPIOC) */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  /* configure DISC (GPIOC12) as open drain digital output */
  gpio_init.GPIO_Pin   = GPIO_Pin_12;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init.GPIO_Mode  = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOC, &gpio_init);
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
  /* configure interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USB_COM_NVIC_IRQ_PRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* set module to initialized */
  usbInitialized = TRUE;
  /* init the USB communication stack */
  USB_Init();
} /*** end of UsbComInit ***/


/************************************************************************************//**
** \brief     Submit data for transmission to the USB host.
** \param     data Pointer to the byte array with data to transmit.
** \param     len  Number of bytes to transmit.
** \return    none.
**
****************************************************************************************/
void UsbComTransmit(uint8_t *data, uint32_t len)
{
  uint32_t idx;
  uint32_t saved_cs_state;

  /* enter critical section */
  saved_cs_state = OsEnterCriticalSection();
  /* copy the data to the ring buffer */
  for (idx=0; idx<len; idx++)
  {
    /* write byte */
    USART_Rx_Buffer[USART_Rx_ptr_in] = data[idx];
    /* increment buffer indexer */
    USART_Rx_ptr_in++;
    /* prevent buffer overflow */
    if(USART_Rx_ptr_in == USART_RX_DATA_SIZE)
    {
      USART_Rx_ptr_in = 0;
    }
  }
  /* leave critical section */
  OsLeaveCriticalSection(saved_cs_state);
} /*** end of UsbComTransmit ***/


/************************************************************************************//**
** \brief     Registers the callback function that is to be called whenever new data is
**            received from the USB host.
** \param     callbackPtr Pointer to the callback function.
** \return    none.
**
****************************************************************************************/
void UsbComRegisterDataReceivedCallback(tUsbComCallbackDataReceived callbackPtr)
{
  /* set the function callback pointer */
  usbComCallbackDataReceived = callbackPtr;
} /*** end of UsbComRegisterDataReceivedCallback ***/


/************************************************************************************//**
** \brief     Handles the reception of new data from the USB host. Note that this
**            function is called at ISR level.
** \param     data Pointer to the byte array with received data.
** \param     len  Number of bytes that were received.
** \return    none.
**
****************************************************************************************/
void UsbComHandleReception(uint8_t *data, uint32_t len)
{
  /* call callback function */
  if (usbComCallbackDataReceived != NULL)
  {
    usbComCallbackDataReceived(data, len);
  }
} /*** end of UsbComHandleReception ***/


/************************************************************************************//**
** \brief     Send data to USB.
** \return    none.
**
****************************************************************************************/
void UsbComHandleAsynchXfer(void)
{

  uint16_t USB_Tx_ptr;
  uint16_t USB_Tx_length;

  if(USB_Tx_State != 1)
  {
    if (USART_Rx_ptr_out == USART_RX_DATA_SIZE)
    {
      USART_Rx_ptr_out = 0;
    }

    if(USART_Rx_ptr_out == USART_Rx_ptr_in)
    {
      USB_Tx_State = 0;
      return;
    }

    if(USART_Rx_ptr_out > USART_Rx_ptr_in) /* rollback */
    {
      USART_Rx_length = USART_RX_DATA_SIZE - USART_Rx_ptr_out;
    }
    else
    {
      USART_Rx_length = USART_Rx_ptr_in - USART_Rx_ptr_out;
    }

    if (USART_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE)
    {
      USB_Tx_ptr = USART_Rx_ptr_out;
      USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;

      USART_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;
      USART_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;
    }
    else
    {
      USB_Tx_ptr = USART_Rx_ptr_out;
      USB_Tx_length = USART_Rx_length;

      USART_Rx_ptr_out += USART_Rx_length;
      USART_Rx_length = 0;
    }
    USB_Tx_State = 1;

    UserToPMABufferCopy(&USART_Rx_Buffer[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
    SetEPTxCount(ENDP1, USB_Tx_length);
    SetEPTxValid(ENDP1);
  }
} /*** end of UsbComHandleAsynchXfer ***/



/************************************************************************************//**
** \brief     Callback that gets called whenever the USB device should be connected
**            to the USB bus.
** \param     connect TRUE to connect and FALSE to disconnect.
** \return    none.
**
****************************************************************************************/
void UsbComConnect(uint8_t connect)
{
  /* determine if the USB should be connected or disconnected */
  if (connect == TRUE)
  {
    /* the GPIO has a pull-up so to connect to the USB bus the pin needs to go low */
    GPIO_ResetBits(GPIOC, GPIO_Pin_12);
  }
  else
  {
    /* the GPIO has a pull-up so to disconnect to the USB bus the pin needs to go high */
    GPIO_SetBits(GPIOC, GPIO_Pin_12);
  }
} /*** end of UsbComConnect ***/


/************************************************************************************//**
** \brief     Callback that gets called whenever the USB host requests the device
**            to enter a low power mode.
** \return    none.
**
****************************************************************************************/
void UsbComEnterLowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
} /*** end of UsbComEnterLowPowerMode ***/


/************************************************************************************//**
** \brief     Callback that gets called whenever the USB host requests the device to
**            exit low power mode.
** \return    none.
**
****************************************************************************************/
void UsbComLeaveLowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
} /*** end of UsbComEnterLowPowerMode ***/



/************************************************************************************//**
** \brief     Create the serial number string descriptor.
** \return    none.
**
****************************************************************************************/
void UsbComGetSerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
  Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
  Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);

  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    UsbComIntToUnicode(Device_Serial0, &Virtual_Com_Port_StringSerial[2] , 8);
    UsbComIntToUnicode(Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
  }
} /*** end of UsbComGetSerialNum ***/


/************************************************************************************//**
** \brief     Convert Hex 32Bits value into char.
** \return    none.
**
****************************************************************************************/
static void UsbComIntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;

  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[ 2* idx + 1] = 0;
  }
} /*** end of UsbComIntToUnicode ***/


/************************************************************************************//**
** \brief     Handles the USB full speed interrupt.
** \return    none.
**
****************************************************************************************/
void UsbComIRQHandler(void)
{
  /* only run the ISR handler is we are actually initialized */
  if (usbInitialized == TRUE)
  {
    USB_Istr();
  }
} /*** end of UsbComIRQHandler ***/


/********************************* end of usbcom.c *************************************/


