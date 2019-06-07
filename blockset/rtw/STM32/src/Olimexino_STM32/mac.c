/************************************************************************************//**
* \file         mac.c
* \brief        Measurement and calibration source file.
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*  Copyright 2019 (c)  by HAN Automotive  http://www.han.nl           All rights reserved
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
#include "can.h"                                      /* CAN driver                    */
#include "canio.h"                                    /* CAN I/O module                */
#include "usbcom.h"                                   /* USB COM-port header file      */
#include "uart.h"                                     /* UART driver                   */
#include "XcpTargetSpecific.h"						  /* XCP functionality			   */
#include "errorHandling.h"
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */
#include <string.h>                                   /* for memcpy                    */

#include "stm32f10x.h"

/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Maximum XCP CTO packet length for CAN. */
#define MAC_XCP_MAX_CTO_CAN        (8)

/** \brief Maximum XCP DTO packet length for CAN. */
#define MAC_XCP_MAX_DTO_CAN        (8)

/** \brief Maximum XCP CTO packet length for USB and UART. */
#define MAC_XCP_MAX_CTO_UART       (32)

/** \brief Maximum XCP DTO packet length for USB and UART. */
#define MAC_XCP_MAX_DTO_UART       (32)

/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void MacCroMsgReceived(uint8_t dlc, uint8_t *data, uint32_t timestamp);
static void XcpEventTimerCallback(xTimerHandle pxTimer);
static void MacUsbComCallbackDataReceived(uint8_t *data, uint32_t len);
static void MacUartCallbackDataReceived(void);

/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Variable to hold the maximum number of bytes in a CTO packet. Having this as
 *         a variable makes it possible to change this dynamically based on the used 
 *         communication interface.
 */
static uint8_t xcpMaxCTO = MAC_XCP_MAX_CTO_CAN;

/** \brief Variable to hold the maximum number of bytes in a DTO packet. Having this as
 *         a variable makes it possible to change this dynamically based on the used 
 *         communication interface.
 */
static uint8_t xcpMaxDTO = MAC_XCP_MAX_DTO_CAN;

/** \brief Event timer for XCP */
static xTimerHandle xcpEventTimer;

/************************************************************************************//**
** \brief     Initializes the MAC module for the USB Virtual COM-port. Should be called
**            before MacInit().
** \return    none.
**
****************************************************************************************/
void MacUsbComInit(void)
{
  /* initialize the USB virtual COM port driver */
  UsbComInit();
  /* register the data reception handler */
  UsbComRegisterDataReceivedCallback(MacUsbComCallbackDataReceived);
  /* configures the XCP stack according user demands */
  /* param 1 = checksum (0 = off 1 = on) */
  XcpDynamicConfigurator (0x01, MAC_XCP_MAX_CTO_UART, MAC_XCP_MAX_DTO_UART);
  XcpUsbConfigureParamaters();
} /*** end of MacUsbComInit ***/


/************************************************************************************//**
** \brief     Initializes the MAC module for CAN. Should be called before MacInit().
** \param     channel CAN channel, which is don't care for this target.
** \param     croID XCP CRO message identifier for master->slave communication.
** \param     dtoID XCP DTO message identifier for slave->master communication.
** \return    none.
**
****************************************************************************************/
void MacCanInit(uint8_t channel, uint32_t croID, uint32_t dtoID)
{
  /* register the CRO message reception callback */
  CanIoRegisterMessageReceivedCallback(0, croID, MacCroMsgReceived);
  /* configures the XCP stack according user demands */
  /* param 1 = checksum (0 = off 1 = on) */
  XcpDynamicConfigurator (0x00, MAC_XCP_MAX_CTO_CAN, MAC_XCP_MAX_DTO_CAN);
  XcpCanConfigureParamaters(dtoID, channel);
} /*** end of MacCanInit ***/

/************************************************************************************//**
** \brief     Initializes the MAC module for the UART. Should be called
**            before MacInit().
** \return    none.
**
****************************************************************************************/
void MacUartInit(uint8_t channel, uint32_t baudrate)
{
  /* initialize the UART driver */
  UartInit(channel, baudrate, DATA_BITS_8, PARITY_NONE, STOP_BITS_1, 36, 36);
 
  /* register the data reception handler */
  UartRegisterDataReceivedCallback(channel, MacUartCallbackDataReceived);

  /* configures the XCP stack according user demands */
  /* param 1 = checksum (0 = off 1 = on) */
  XcpDynamicConfigurator (0x01, MAC_XCP_MAX_CTO_UART, MAC_XCP_MAX_DTO_UART);
  XcpUartConfigureParamaters(channel);
} /*** end of MacUartComInit ***/


/************************************************************************************//**
** \brief     Initializes the MAC module.
** \return    none.
**
****************************************************************************************/
void MacInit(void)
{
  static uint8_t initialized = FALSE;
  portBASE_TYPE result;
  
  /* add protection in case this function is called more than once */
  if (initialized == TRUE)
  {
    /* already initialized so nothing left to do */
    return;
  }
  
  /* still here, so first time this function is called */
  initialized = TRUE;

  /* create the event channel timers */
  xcpEventTimer = xTimerCreate("EvChnl1",
                               (((portTickType)10000)/portTICK_PERIOD_US),
                               pdTRUE, (void *) 0,
                               XcpEventTimerCallback);
  if (!(xcpEventTimer != NULL))
  {
    ErrCodesSetError(ER_CODE_MAC_EVENT_CHANNEL_TIMER_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* start the timers. note that block time must be 0 since scheduler hasn't been started
   * yet.
   */
  result = xTimerStart(xcpEventTimer, 0 );
  if (!(result == pdPASS))
  {
    ErrCodesSetError(ER_CODE_MAC_EVENT_CHANNEL_TIMER_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

} /*** end of MacInit ***/


/************************************************************************************//**
** \brief     Utility function for the XCP driver to determine the maximum number of
**            bytes in an XCP CTO packets.
** \return    none.
**
****************************************************************************************/
uint8_t MacGetXcpMaxCTO(void)
{
  return xcpMaxCTO;
} /*** end of MacGetXcpMaxCTO ***/


/************************************************************************************//**
** \brief     Utility function for the XCP driver to determine the maximum number of
**            bytes in an XCP DTO packets.
** \return    none.
**
****************************************************************************************/
uint8_t MacGetXcpMaxDTO(void)
{
  return xcpMaxDTO;
} /*** end of MacGetXcpMaxDTO ***/


/************************************************************************************//**
** \brief     Callback function to process newly received data from the USB host that
**            acts as a virtual COM-port.
** \param     data Pointer to the byte array with received data.
** \param     len  Number of bytes that were received.
** \return    none.
**
****************************************************************************************/
static void MacUsbComCallbackDataReceived(uint8_t *data, uint32_t len)
{
  /* Entry point of USB received XCP data for XCP stack */
  XcpUsbReceive (data, len);
} /*** end of MacUsbComCallbackDataReceived ***/

/************************************************************************************//**
** \brief     Callback function to process newly received data from the UART
** \param     data Pointer to the byte array with received data.
** \param     len  Number of bytes that were received.
** \return    none.
**
****************************************************************************************/
static void MacUartCallbackDataReceived(void)
{
  /* Entry point of UART received XCP data for XCP stack */
  XcpUartReceive();
} /*** end of MacUartCallbackDataReceived ***/

/************************************************************************************//**
** \brief     Callback function that gets called each time the XCP CRO message was
**            received.
** \param     dlc Number of bytes in the message.
** \param     data Byte array with message data
** \param     timestamp Message timestamp in ms.
** \return    none.
**
****************************************************************************************/
static void MacCroMsgReceived(uint8_t dlc, uint8_t *data, uint32_t timestamp)
{
  /* Entry point of CAN received XCP data for XCP stack */
  XcpCanReceive (data, dlc);
} /*** end of MacCroMsgReceived ***/


/************************************************************************************//**
** \brief     Callback function that gets called by the OS each time the period for
**            XCP event channel expired.
** \param     pxTimer Handler of the timer that expired.
** \return    none.
**
****************************************************************************************/
static void XcpEventTimerCallback(xTimerHandle pxTimer)
{
  /* process the correct XCP event channel */
  if (pxTimer == xcpEventTimer)
  {
    XcpDataTransmission();
  }
} /*** end of XcpEventTimerCallback ***/

/************************************ end of mac.c *************************************/