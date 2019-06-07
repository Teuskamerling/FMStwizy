/************************************************************************************//**
* \file         udp_client.c
* \brief        UDP client for the lwIP stack source file.
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
#include "udp_client.h"                               /* module header                 */
#include "lwip/api.h"                                 /* for lwIP netconn API          */


/************************************************************************************//**
** \brief     Sends data via UDP to a UDP server at a specific IP address/port.
** \param     data Pointer to a byte array with the data to send.
** \param     len Number of data bytes to send.
** \param     port Port to send the UDP packet on.
** \param     ip1 first part of the destination IP address.
** \param     ip2 second part of the destination IP address.
** \param     ip3 third part of the destination IP address.
** \param     ip4 fourth part of the destination IP address.
** \return    TRUE is successful, FALSE otherwise.
**
****************************************************************************************/
uint8_t UdpClientSend(uint8_t *data, uint8_t len, uint16_t port, 
                      uint8_t ip1, uint8_t ip2, uint8_t ip3, uint8_t ip4)
{
  static struct netbuf *buf = NULL;
  struct netconn *conn;
  err_t result;
  struct ip_addr ip_address;

  /* create a new netbuf */
  buf = netbuf_new();
  if (buf == NULL)
  {
    /* could not allocate netbuf object */ //TODO Jason: set error?
    return FALSE;
  }
  /* init the netbuf */
  if (netbuf_ref(buf, data, len) != ERR_OK)
  {
    /* free the netbuf object */
    netbuf_delete(buf);
    /* could not allocate netbuf object */
    return FALSE;
  }

  /* convert the requested 4 IP address bytes to an IP address */
  IP4_ADDR(&ip_address, ip1, ip2, ip3, ip4);
  
  /* create a new connection */
  conn = netconn_new(NETCONN_UDP);
  if (conn == NULL)
  {
	/* free the netbuf object */
    netbuf_delete(buf);
    /* could not allocate netconn object */
    return FALSE;
  }

  /* connect the connection to the remote host */
  result = netconn_connect(conn, &ip_address, port);
  if (result != ERR_OK)
  {
	/* free the netbuf object */
    netbuf_delete(buf);
    /* free the netconn object */
    netconn_delete(conn);
    /* could not connect to the IP address / port */
    return FALSE;
  }

  /* update the netbuf reference information with the data that is to be send. note that
   * during sending this calling task is suspended until the transmission is complete. This
   * means there are no data consistency problems by referencing the data as opposed to
   * copying it, but it is a lot faster.
   */
  buf->p->payload = (void*)data;
  buf->p->len = buf->p->tot_len = len;
  buf->ptr = buf->p;

  /* now send the data */
  result = netconn_send(conn, buf);

  /* free the netconn object */
  netconn_delete(conn);
  
  /* free the buffer object */
  netbuf_delete(buf);

  /* process result */
  if (result != ERR_OK)
  {
    return FALSE;
  } else {
  /* still here so sending was successful */
  return TRUE;
  }
} /*** end of UdpClientSend ***/


/************************************ end of udp_client.c ******************************/


