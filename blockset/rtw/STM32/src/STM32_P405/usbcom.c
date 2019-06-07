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
#include "usb_core.h"                                 /* USB library core              */
#include "usb_conf.h"                                 /* USB descriptor                */
#include "usb_dcd_int.h"                              /* USB core interrupts           */
#include "usbd_core.h"                                /* USB driver core               */
#include "usbd_usr.h"                                 /* USB driver configuration      */
#include "usbd_desc.h"                                /* USB driver descriptor         */
#include "usbd_cdc_core.h"                            /* USB driver CDC class core     */
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */


/****************************************************************************************
* External data declarations
****************************************************************************************/
/** \brief Write CDC received data in this buffer. These data will be sent over USB IN
 *         endpoint in the CDC core functions.
 */
extern uint8_t  APP_Rx_Buffer[];

/** \brief Increment this pointer or roll it back to start address when writing received
 *         data in the buffer APP_Rx_Buffer.
 */
extern uint32_t APP_Rx_ptr_in;


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Function pointer to the data reception callback queue. */
static tUsbComCallbackDataReceived usbComCallbackDataReceived = NULL;

/** \brief USB device handler. Must be 32-bit aligned. */
static USB_OTG_CORE_HANDLE USB_OTG_dev __attribute__((aligned(4)));


/************************************************************************************//**
** \brief     Initializes the USB virtual COM-port driver.
** \return    none.
**
****************************************************************************************/
void UsbComInit(void)
{
  /* reset the function callback pointer */
  usbComCallbackDataReceived = NULL;
  /* initialize the USB device for the CDC class */
  USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
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
    APP_Rx_Buffer[APP_Rx_ptr_in] = data[idx];
    /* increment buffer indexer */
    APP_Rx_ptr_in++;
    /* prevent buffer overflow */
    if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
    {
      APP_Rx_ptr_in = 0;
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
** \brief     Handles the USB wake up interrupt.
** \return    none.
**
****************************************************************************************/
void UsbComWakeUpIRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line18);
} /*** end of UsbComWakeUpIRQHandler ***/


/************************************************************************************//**
** \brief     Handles the USB full speed interrupt.
** \return    none.
**
****************************************************************************************/
void UsbComIRQHandler(void)
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
} /*** end of UsbComIRQHandler ***/


/********************************* end of usbcom.c *************************************/


