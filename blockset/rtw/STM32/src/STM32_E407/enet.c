/************************************************************************************//**
* \file         enet.c
* \brief        Ethernet module source file.
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
#include "os.h"                                      /* for operating system           */
#include "enet.h"                                    /* ethernet module header         */
#include "ethernetif.h"
#include "stm32f4x7_eth_bsp.h"



/****************************************************************************************
* Global data declarations
****************************************************************************************/
/** \brief Network interface structure. */
struct netif xnetif;


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief IP address part 1 */
static uint8_t enetIp1 = 169;
/** \brief IP address part 2 */
static uint8_t enetIp2 = 254;
/** \brief IP address part 3 */
static uint8_t enetIp3 = 19;
/** \brief IP address part 4 */
static uint8_t enetIp4 = 63;

/** \brief Netmask part 1 */
static uint8_t enetNm1 = 255;
/** \brief Netmask part 2 */
static uint8_t enetNm2 = 255;
/** \brief Netmask part 3 */
static uint8_t enetNm3 = 0;
/** \brief Netmask part 4 */
static uint8_t enetNm4 = 0;

/** \brief Gateway part 1 */
static uint8_t enetGw1 = 169;
/** \brief Gateway part 2 */
static uint8_t enetGw2 = 254;
/** \brief Gateway part 3 */
static uint8_t enetGw3 = 1;
/** \brief Gateway part 4 */
static uint8_t enetGw4 = 1;


/****************************************************************************************
* External data declarations
****************************************************************************************/
/** \brief Ethernet status. */
extern volatile uint32_t EthStatus;


/************************************************************************************//**
** \brief     Initialize the ethernet application and interface.
** \return    none.
**
****************************************************************************************/
void EnetInit(void)
{
  struct ip_addr ipaddr;
  struct ip_addr netmask;
  struct ip_addr gw;

  /* configure PHY  */
  ETH_BSP_Config();

  /* create tcp_ip stack thread */
  tcpip_init( NULL, NULL );

  /* IP address setting */
  IP4_ADDR(&ipaddr, enetIp1, enetIp2, enetIp3, enetIp4);
  IP4_ADDR(&netmask, enetNm1, enetNm2 , enetNm3, enetNm4);
  IP4_ADDR(&gw, enetGw1, enetGw2, enetGw3, enetGw4);

  /* Adds your network interface to the netif_list. Allocate a struct
   * netif and pass a pointer to this structure as the first argument.
   * Give pointers to cleared ip_addr structures when using DHCP,
   * or fill them with sane numbers otherwise. The state pointer may be NULL.
   * The init function pointer must point to a initialization function for
   * your ethernet netif interface. The following code illustrates it's use.
   */
  netif_add(&xnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

  /* Registers the default network interface. */
  netif_set_default(&xnetif);

  if (EthStatus == (ETH_INIT_FLAG | ETH_LINK_FLAG))
  {
    /* Set Ethernet link flag */
    xnetif.flags |= NETIF_FLAG_LINK_UP;
    /* When the netif is fully configured this function must be called.*/
    netif_set_up(&xnetif);
  }
  else
  {
    /*  When the netif link is down this function must be called.*/
    netif_set_down(&xnetif);
  }

  /* Set the link callback function, this function is called on change of link status*/
  netif_set_link_callback(&xnetif, ETH_link_callback);
} /*** end of EnetInit ***/


/************************************************************************************//**
** \brief     Called by the ethernet driver whenever the network links goes up.
** \return    none.
**
****************************************************************************************/
void EnetLinkUpHook(void)
{
  /* could be used to indicate to the user that the network link is up */
} /*** end of EnetLinkUpHook ***/


/************************************************************************************//**
** \brief     Called by the ethernet driver whenever the network links goes down.
** \return    none.
**
****************************************************************************************/
void EnetLinkDownHook(void)
{
  /* could be used to indicate to the user that the network link is down */
} /*** end of EnetLinkDownHook ***/


/************************************************************************************//**
** \brief     Configures the IP address. Should be called before EnetInit().
** \return    none.
**
****************************************************************************************/
void EnetSetIpAddress(uint8_t ip1, uint8_t ip2, uint8_t ip3, uint8_t ip4)
{
  enetIp1 = ip1;
  enetIp2 = ip2;
  enetIp3 = ip3;
  enetIp4 = ip4;
} /*** end of EnetSetIpAddress ***/


/************************************************************************************//**
** \brief     Configures the network mask. Should be called before EnetInit().
** \return    none.
**
****************************************************************************************/
void EnetSetNetMask(uint8_t nm1, uint8_t nm2, uint8_t nm3, uint8_t nm4)
{
  enetNm1 = nm1;
  enetNm2 = nm2;
  enetNm3 = nm3;
  enetNm4 = nm4;
} /*** end of EnetSetNetMask ***/


/************************************************************************************//**
** \brief     Configures the gateway address. Should be called before EnetInit().
** \return    none.
**
****************************************************************************************/
void EnetSetGateway(uint8_t gw1, uint8_t gw2, uint8_t gw3, uint8_t gw4)
{
  enetGw1 = gw1;
  enetGw2 = gw2;
  enetGw3 = gw3;
  enetGw4 = gw4;
} /*** end of EnetSetGateway ***/


/************************************************************************************//**
** \brief     Obtains the configured IP address.
** \return    none.
**
****************************************************************************************/
void EnetGetIpAddress(uint8_t *ip1, uint8_t *ip2, uint8_t *ip3, uint8_t *ip4)
{
  *ip1 = enetIp1;
  *ip2 = enetIp2;
  *ip3 = enetIp3;
  *ip4 = enetIp4;
} /*** end of EnetGetIpAddress ***/


/************************************************************************************//**
** \brief     Obtains the configured network mask.
** \return    none.
**
****************************************************************************************/
void EnetGetNetMask(uint8_t *nm1, uint8_t *nm2, uint8_t *nm3, uint8_t *nm4)
{
  *nm1 = enetNm1;
  *nm2 = enetNm2;
  *nm3 = enetNm3;
  *nm4 = enetNm4;
} /*** end of EnetGetNetMask ***/


/************************************************************************************//**
** \brief     Obtains the configured gateway address.
** \return    none.
**
****************************************************************************************/
void EnetGetGateway(uint8_t *gw1, uint8_t *gw2, uint8_t *gw3, uint8_t *gw4)
{
  *gw1 = enetGw1;
  *gw2 = enetGw2;
  *gw3 = enetGw3;
  *gw4 = enetGw4;
} /*** end of EnetGetGateway ***/


/*********************************** end of enet.c *************************************/
