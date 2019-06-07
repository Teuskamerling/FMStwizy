/************************************************************************************//**
* \file         udp_server.c
* \brief        UDP server for the lwIP stack source file.
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
#include "udp_server.h"                               /* module header                 */
#include "lwip/api.h"                                 /* for lwIP netconn API          */
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Stack size of the UDP server thread. */
#define UDP_SERVER_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE)
/** \brief Priority of the UDP server thread. */
#define UDP_SERVER_TASK_PRIORITY            (tskIDLE_PRIORITY + 4)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Type to group all members related to the data buffer management. */
typedef struct t_udp_server_buffer_info
{
  uint16_t totalSize;     /**< Maximum byte size of the data buffer.                   */
  uint16_t byteCount;     /**< Current number of data bytes stored in the data buffer. */
  uint8_t  newFlag;       /**< New data flag.                                          */
  uint8_t *data;          /**< Pointer to byte array with the actual data.             */
} tUdpServerBufferInfo;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void UdpServerThread(void *arg);


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Stores the port that the server should listen on. */
static uint16_t udpServerListenPort;
/** \brief Holds all the information regarding the data reception buffer. */
static tUdpServerBufferInfo udpServerBufferInfo;


/************************************************************************************//**
** \brief     Initializes the UDP server for receiving UPD packets from  a specific
**            IP address/port. Use IP address 0.0.0.0 to listen for UDP packets from
**            all IP addresses on the network.
** \param     port Port to listen on for UDP packets.
** \param     rxBufLen Size on bytes of the UDP reception buffer.
** \return    none.
**
****************************************************************************************/
void UdpServerInit(uint16_t port, uint16_t rxBufLen)
{
  /* initialize locals */
  udpServerListenPort = port;
  udpServerBufferInfo.data = pvPortMalloc(rxBufLen);
  udpServerBufferInfo.totalSize = rxBufLen;
  udpServerBufferInfo.byteCount = 0;
  udpServerBufferInfo.newFlag = FALSE;
  /* check successful allocation of the reception buffer on the heap */
  if (!(udpServerBufferInfo.data != NULL))
  {
    ErrCodesSetError(ER_CODE_UDPSERVER_RXBUF_ALLOC_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* create the server thread */
  sys_thread_new("UdpServer", UdpServerThread, NULL, UDP_SERVER_TASK_STACK_SIZE, UDP_SERVER_TASK_PRIORITY);
} /*** end of UdpServerInit ***/


/************************************************************************************//**
** \brief     Obtains the number of data bytes currently stored in the reception buffer.
** \return    byte count.
**
****************************************************************************************/
uint16_t UdpServerGetRxCount(void)
{
  return udpServerBufferInfo.byteCount;
} /*** end of UdpServerGetRxCount ***/


/************************************************************************************//**
** \brief     Obtains the value of the new data flag. This flag is set to TRUE each time
**            new data was received and written to the reception buffer by this server.
**            The flag is reset to FALSE when the user reads data out of the reception
**            buffer using UdpServerGetRxData().
** \return    The start of the new data flag. This is either TRUE or FALSE.
**
****************************************************************************************/
uint8_t UdpServerIsRxNewData(void)
{
  return udpServerBufferInfo.newFlag;
} /*** end of UdpServerIsRxNewData ***/


/************************************************************************************//**
** \brief     Reads bytes for the reception buffer and stores them in the byte array
**            that the data-parameter points to.
** \param     count The number of bytes to read from the data buffer.
** \param     startIdx Zero based index in the data buffer to start reading from.
** \param     data Pointer to byte array where the bytes will be written to.
** \return    TRUE is the data was successfully read, FALSE otherwise.
**
****************************************************************************************/
uint8_t UdpServerGetRxData(uint16_t count, uint16_t startIdx, uint8_t *data)
{
  uint16_t idx;
  uint32_t oldCsState;

  /* check to make sure we don't read outside of the reception buffer */
  if ((startIdx + count) > udpServerBufferInfo.totalSize)
  {
    /* cannot read this many bytes from the reception buffer because there are not that
     * many bytes available in the reception buffer.
     */
    return FALSE;
  }

  /* enter critical section for reading out the received data bytes */
  oldCsState = OsEnterCriticalSection();
  /* copy the bytes from the reception buffer */
  for (idx=0; idx<count; idx++)
  {
    data[idx] = udpServerBufferInfo.data[idx + startIdx];
  }
  /* reset new data flag because the buffer was read at least once */
  udpServerBufferInfo.newFlag = FALSE;
  /* leave critical section */
  OsLeaveCriticalSection(oldCsState);

  /* data successfully read from the reception buffer */
  return TRUE;
} /*** end of UdpServerGetRxData ***/


/************************************************************************************//**
** \brief     UDP server thread responsible for received UDP packets.
** \param     arg Pointer to thread parameters.
** \return    none.
**
****************************************************************************************/
static void UdpServerThread(void *arg)
{
  struct netconn *conn;
  struct netbuf *buf;
  err_t result;

  /* create a new netconn connection object for UDP communication */
  conn = netconn_new(NETCONN_UDP);
  /* check successful allocation of the connection object on the heap */
  if (!(conn != NULL))
  {
    ErrCodesSetError(ER_CODE_UDPSERVER_CONN_ALLOC_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* bind to the port to listen on */
  result = netconn_bind(conn, NULL, udpServerListenPort);
  if (!(result == ERR_OK))
  {
    ErrCodesSetError(ER_CODE_UDPSERVER_CONN_BIND_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* start thread body */
  for (;;)
  {
    result = netconn_recv(conn, &buf);
    if (result == ERR_OK)
    {
      /* discard UDP packets that are too large for the reception buffer */
      if (buf->p->tot_len <= udpServerBufferInfo.totalSize)
      {
        /* copy the newly received UDP packet data to the reception buffer */
        netbuf_copy(buf, udpServerBufferInfo.data, buf->p->tot_len);
        /* store the data length */
        udpServerBufferInfo.byteCount = buf->p->tot_len;
        /* set the new data flag to indicate that new data is available */
        udpServerBufferInfo.newFlag = TRUE;
        /* release the netbuff so it can be re-used for the next data reception */
        netbuf_delete(buf);
      }
    }
  }
} /*** end of UdpServerThread ***/


/************************************ end of udp_server.c ******************************/


