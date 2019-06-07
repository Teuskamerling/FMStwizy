/************************************************************************************//**
* \file         enet.h
* \brief        Ethernet module header file.
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
#ifndef ENET_H
#define ENET_H

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>                                   /* ANSI C types                  */
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/tcpip.h"


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void EnetInit(void);
void EnetSetIpAddress(uint8_t ip1, uint8_t ip2, uint8_t ip3, uint8_t ip4);
void EnetSetNetMask(uint8_t nm1, uint8_t nm2, uint8_t nm3, uint8_t nm4);
void EnetSetGateway(uint8_t gw1, uint8_t gw2, uint8_t gw3, uint8_t gw4);
void EnetGetIpAddress(uint8_t *ip1, uint8_t *ip2, uint8_t *ip3, uint8_t *ip4);
void EnetGetNetMask(uint8_t *nm1, uint8_t *nm2, uint8_t *nm3, uint8_t *nm4);
void EnetGetGateway(uint8_t *gw1, uint8_t *gw2, uint8_t *gw3, uint8_t *gw4);


/****************************************************************************************
* Hook function prototypes
****************************************************************************************/
void EnetLinkUpHook(void);
void EnetLinkDownHook(void);


#endif /* ENET_H */
/********************************* end of enet.h ***************************************/


