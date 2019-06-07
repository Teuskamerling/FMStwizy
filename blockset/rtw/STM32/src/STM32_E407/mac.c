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
#include "enet.h"                                     /* for lwIP/ethernet             */
#include "can.h"                                      /* CAN driver                    */
#include "canio.h"                                    /* CAN I/O module                */
#include "XcpTargetSpecific.h"						  /* XCP functionality			   */
#include "errorHandling.h"
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */
#include "usbcom.h"                                   /* USB COM-port header file      */
#include <string.h>                                   /* for memcpy                    */
#include "tcp.h"									  /* for tcp definitions		   */
#include "enet.h"									  /* for enet definitions		   */

/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Priority of the MAC task. */
#define MAC_TASK_PRIO                    (tskIDLE_PRIORITY + 4)

/** \brief Maximum number of DTO packets that can be stored in the buffer. */
#define MAC_NET_DTO_PACKET_BUFFER_SIZE   (16)

/** \brief Maximum XCP CTO packet length for CAN. */
#define MAC_XCP_MAX_CTO_CAN        (8)

/** \brief Maximum XCP DTO packet length for CAN. */
#define MAC_XCP_MAX_DTO_CAN        (8)

/** \brief Maximum XCP CTO packet length for Ethernet. */
#define MAC_XCP_MAX_CTO_ENET       (255)

/** \brief Maximum XCP DTO packet length for Ethernet. */
#define MAC_XCP_MAX_DTO_ENET       (255)

/** \brief Maximum XCP CTO packet length for UART. */
#define MAC_XCP_MAX_CTO_UART       (32)

/** \brief Maximum XCP DTO packet length for UART. */
#define MAC_XCP_MAX_DTO_UART       (32)

/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief List of digital input pin configurations. */
typedef enum
{
  MAC_IF_CAN,                                   /**< XCP on CAN                        */
  MAC_IF_ENET,                                  /**< XCP on TCP/IP                     */
  MAC_IF_USB                                    /**< XCP on USB (OTG1 connector        */
} tMacIfType;


/** \brief XCP CTO packet type with TCP/IP header  */
typedef union
{
  struct
  {
    uint16_t len;
    uint16_t counter;
    uint8_t data[MAC_XCP_MAX_CTO_ENET];
  } s;
  uint8_t raw[4+MAC_XCP_MAX_CTO_ENET];
} __attribute__((packed)) tMacNetXcpCtoPacket;

/** \brief XCP DTO packet type with TCP/IP header  */
typedef union
{
  struct
  {
    uint16_t len;
    uint16_t counter;
    uint8_t data[MAC_XCP_MAX_DTO_ENET];
  } s;
  uint8_t raw[4+MAC_XCP_MAX_DTO_ENET];
} __attribute__((packed)) tMacNetXcpDtoPacket;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void MacUsbComCallbackDataReceived(uint8_t *data, uint32_t len);
static void MacNetDtoBufferInit(void);
static uint8_t MacNetDtoBufferRead(tMacNetXcpDtoPacket *packet);
static uint16_t MacNetDtoBufferScan(void);
static void MacNetTask(void *pvParameters);
static void MacNetServeConnection(struct netconn *conn);
static void MacCroMsgReceived(uint8_t dlc, uint8_t *data, uint32_t timestamp);
static void XcpEventTimerCallback(xTimerHandle pxTimer);


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

/** \brief XCP DTO buffer to hold multiple DTO packets with TCP/IP header  */
static struct
{
  tMacNetXcpDtoPacket packet[MAC_NET_DTO_PACKET_BUFFER_SIZE];
  uint16_t writeIdx;
  uint16_t readIdx;
  uint16_t entries;
  uint16_t txCounter;
} macNetXcpDtoBuffer;

/** \brief The TCP/IP port to listen for XCP commands. */
static uint16_t macNetPort = 1000;

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
** \brief     Initializes the MAC module for Ethernet. Should be called before MacInit()
**            but after EnetInit().
** \return    none.
**
****************************************************************************************/
void MacEnetInit(uint16_t port)
{
  portBASE_TYPE    result;
  
  /* store the port to listen on */
  macNetPort = port;
  /* create the application task */
  result = xTaskCreate(MacNetTask, "MacNetTask", configMINIMAL_STACK_SIZE + 128,
              NULL, MAC_TASK_PRIO, NULL);
  /* Set error if the task couldn't be created */
  if (!(result == pdPASS))
  {
    ErrCodesSetError(ER_CODE_MAC_NET_TASK_INVALID, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
    /* configures the XCP stack according user demands */
    /* param 1 = checksum (0 = off 1 = on) */
    XcpDynamicConfigurator (0x00, MAC_XCP_MAX_CTO_ENET, MAC_XCP_MAX_DTO_ENET);
	XcpEthConfigureParamaters();
} /*** end of MacEnetInit ***/


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
  CanIoRegisterMessageReceivedCallback(channel, croID, MacCroMsgReceived);
  /* configures the XCP stack according user demands */
  /* param 1 = checksum (0 = off 1 = on) */
  XcpDynamicConfigurator (0x00, MAC_XCP_MAX_CTO_CAN, MAC_XCP_MAX_DTO_CAN);
  XcpCanConfigureParamaters(dtoID, channel);
} /*** end of MacCanInit ***/


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

// TODO check if these utility functions are neccesarry for other functionalities
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
** \brief     Initializes the DTO packet buffer.
** \return    none.
**
****************************************************************************************/
static void MacNetDtoBufferInit(void)
{
  /* initialize the buffer management variables */
  macNetXcpDtoBuffer.writeIdx = 0;
  macNetXcpDtoBuffer.readIdx = 0;
  macNetXcpDtoBuffer.entries = 0;
  macNetXcpDtoBuffer.txCounter = 0;
} /*** end of MacNetDtoBufferInit ***/


/************************************************************************************//**
** \brief     Writes a packet to the DTO packet buffer and adds the TCP/IP header
**            overhead.
** \param     data Pointer to the packet data bytes.
** \param     len Number of data bytes in the packet.
** \return    TRUE is the packet was written, FALSE otherwise.
**
****************************************************************************************/
uint8_t MacNetDtoBufferWrite(uint8_t *data, uint16_t len)
{
  tMacNetXcpDtoPacket *pDtoPacket;
  uint16_t cnt;

  taskENTER_CRITICAL();
  if (macNetXcpDtoBuffer.entries == MAC_NET_DTO_PACKET_BUFFER_SIZE)
  {
    taskEXIT_CRITICAL();
    /* full */
    return FALSE;
  }

  /* get a pointer to the next free DTO packet in the buffer */
  pDtoPacket = &macNetXcpDtoBuffer.packet[macNetXcpDtoBuffer.writeIdx];

  /* write the packet data */
  pDtoPacket->s.len = len;
  pDtoPacket->s.counter = macNetXcpDtoBuffer.txCounter;
  for (cnt=0; cnt<len; cnt++)
  {
    pDtoPacket->s.data[cnt] = data[cnt];
  }

  /* update buffer management */
  macNetXcpDtoBuffer.writeIdx++;
  if (macNetXcpDtoBuffer.writeIdx >= MAC_NET_DTO_PACKET_BUFFER_SIZE)
  {
    macNetXcpDtoBuffer.writeIdx = 0;
  }
  macNetXcpDtoBuffer.entries++;

  /* increment overall DTO packet counter used for the TCP/IP header */
  macNetXcpDtoBuffer.txCounter++;
  taskEXIT_CRITICAL();
  return TRUE;
} /*** end of MacNetDtoBufferWrite ***/


/************************************************************************************//**
** \brief     Reads a packet from the DTO packet buffer.
** \param     packet Pointer to where the packet should be stored.
** \return    TRUE is a packet was read, FALSE otherwise.
**
****************************************************************************************/
static uint8_t MacNetDtoBufferRead(tMacNetXcpDtoPacket *packet)
{
  uint16_t packetLen;
  uint16_t cnt;

  taskENTER_CRITICAL();
  if (macNetXcpDtoBuffer.entries == 0)
  {
    taskEXIT_CRITICAL();
    return FALSE;
  }

  /* determine total length of the packet including TCP/IP overhead */
  packetLen = macNetXcpDtoBuffer.packet[macNetXcpDtoBuffer.readIdx].s.len + 4;

  /* read the data */
  for (cnt=0; cnt<packetLen; cnt++)
  {
    packet->raw[cnt] = macNetXcpDtoBuffer.packet[macNetXcpDtoBuffer.readIdx].raw[cnt];
  }

  /* update buffer management */
  macNetXcpDtoBuffer.readIdx++;
  if (macNetXcpDtoBuffer.readIdx >= MAC_NET_DTO_PACKET_BUFFER_SIZE)
  {
    macNetXcpDtoBuffer.readIdx = 0;
  }
  macNetXcpDtoBuffer.entries--;

  taskEXIT_CRITICAL();
  return TRUE;
} /*** end of MacNetDtoBufferRead ***/


/************************************************************************************//**
** \brief     Determines how many packets are stored in the DTO packet buffer.
** \return    Number of packets.
**
****************************************************************************************/
static uint16_t MacNetDtoBufferScan(void)
{
  uint16_t entries;

  taskENTER_CRITICAL();
  entries = macNetXcpDtoBuffer.entries;
  taskEXIT_CRITICAL();

  return entries;
} /*** end of MacNetDtoBufferScan ***/


/************************************************************************************//**
** \brief     Mac main task.
** \param     pvParamters Pointer to task parameters data structure.
** \return    none.
**
****************************************************************************************/
static void MacNetTask(void *pvParameters)
{
  struct netconn *conn, *newconn;
  err_t accept_err;

  /* create a new TCP connection handle */
  conn = netconn_new(NETCONN_TCP);
  /* disable Nagle's algorithm */
  tcp_nagle_disable(conn->pcb.tcp);
  /* bind to port with default IP address */
  netconn_bind(conn, NULL, macNetPort);
  /* put the connection into LISTEN state */
  netconn_listen(conn);

  for( ;; )
  {
    /* accept any incoming connection */
    accept_err = netconn_accept(conn, &newconn);
    if(accept_err == ERR_OK)
    {
	//if( xSemaphoreTake( LwIPsemaphore, 10 ) == pdTRUE )
    //{
      /* (re)initialize the DTO packet buffer */
      MacNetDtoBufferInit();

      /* set the reception timeout so that pending transmissions can be handled by the
       * connection as well even if no data was received.
       */
      newconn->recv_timeout = 1;
	  
	    /* disable Nagle's algorithm */
	    tcp_nagle_disable(newconn->pcb.tcp);

      /* serve the connection until a fatal error occurs. fatal errors include
       * connections being closed, reset, aborted, etc.
       */
      while(!ERR_IS_FATAL(newconn->last_err))
      {
        /* serve connection */
        MacNetServeConnection(newconn);
      }
      /* delete connection */
      netconn_delete(newconn);
	//xSemaphoreGive(LwIPsemaphore);
	//}
    }
    else
    {
      /* delete connection */
      netconn_delete(newconn);
    }
  }
} /*** end of MacNetTask ***/


/************************************************************************************//**
** \brief     Service function for the MAC network connection.
** \param     conn Pointer to the network connection to serve.
** \return    none.
**
****************************************************************************************/
static void MacNetServeConnection(struct netconn *conn)
{
  struct netbuf *inbuf;
  err_t recv_err;
  uint8_t* buf;
  u16_t buflen;
  tMacNetXcpCtoPacket ctoPacket;
  tMacNetXcpDtoPacket dtoPacket;
  uint16_t idx;

  /* read the data from the port, blocking if nothing yet there. */
  recv_err = netconn_recv(conn, &inbuf);

  /* check if data was received */
  if (recv_err == ERR_OK)
  {
    /* check if no errors occurred */
    if (netconn_err(conn) == ERR_OK)
    {
      /* get access to the newly received data */
      netbuf_data(inbuf, (void**)&buf, &buflen);

      /* loop here because multiple CTO commands might be embedded in 1 TCP/IP packet */
      while (buflen > 0)
      {
        /* copy the first 4 bytes into the CTO packet buffer, which is the XCP header with
         * the length of the counter.
         */
        ctoPacket.raw[0] = *buf;
        buf++;
        ctoPacket.raw[1] = *buf;
        buf++;
        ctoPacket.raw[2] = *buf;
        buf++;
        ctoPacket.raw[3] = *buf;
        buf++;
        /* now copy the data of the CTO packet */
        for (idx=0; idx<ctoPacket.s.len; idx++)
        {
          ctoPacket.s.data[idx] = *buf;
          buf++;
        }
		
		/* Entry point of ethernet received XCP data for XCP stack */
		XcpEthReceive (&ctoPacket.s.data[0], ctoPacket.s.len);
		
        /* decrement the len parameter. note the +4 to also subtract the XCP header bytes */
        buflen -= (ctoPacket.s.len + 4);
      }
    }
  }
  /* delete the buffer. netconn_recv gives us ownership, so we have to make sure to
   * deallocate the buffer.
   */
  netbuf_delete(inbuf);

  /* check if there are pending transmissions */
  while (MacNetDtoBufferScan() > 0)
  {
    if (MacNetDtoBufferRead(&dtoPacket) == FALSE)
    {
      configASSERT(FALSE);
    }
    /* submit the packet for transmission */
    netconn_write(conn, &dtoPacket.raw[0], (u16_t) dtoPacket.s.len + 4, NETCONN_COPY );
  }
} /*** end of MacNetServeConnection ***/

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