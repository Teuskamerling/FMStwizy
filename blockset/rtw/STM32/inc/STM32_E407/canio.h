/************************************************************************************//**
* \file         canio.h
* \brief        CAN message input/output header file. This module is an optional add-on
*               to the CAN driver, which enables message specific event callbacks and
*               storage of newly received message data.
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
#ifndef CANIO_H
#define CANIO_H

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>                                   /* ANSI C types                  */
#include "can.h"


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Function type for the id specific message received callback handler. */
typedef void (* tCanIoCallbackMessageReceived)(uint8_t dlc, uint8_t *data, uint32_t timestamp);

/** \brief Function type for the message unmatched callback handler. */
typedef void (* tCanIoCallbackUnmatchedMessageReceived)(uint32_t id, uint8_t dlc, uint8_t *data, uint32_t timestamp);

/** \brief Function type for the id specific message transmitted callback handler. */
typedef void (* tCanIoCallbackMessageTransmitted)(void);

/** \brief CAN message structure type. */
typedef struct
{
  uint32_t id;                                        /**< message identifier          */
  uint8_t  len;                                       /**< message data length code    */
  uint8_t  data[CAN_MAX_DATA_LEN];                    /**< message data byte array     */
  uint32_t timestamp;                                 /**< message timestamp [ms]      */
} tCanIoMessage;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void    CanIoInit(uint8_t channel);
void    CanIoCreateMessageReceivedStorage(uint8_t channel, uint32_t id);
int8_t  CanIoGetMessage(uint8_t channel, uint32_t id, uint8_t *dlc, uint8_t *data, uint32_t *timestamp);
void    CanIoRegisterMessageTransmittedCallback(uint8_t channel, uint32_t id, tCanIoCallbackMessageTransmitted callbackPtr);
void    CanIoRegisterMessageReceivedCallback(uint8_t channel, uint32_t id, tCanIoCallbackMessageReceived callbackPtr);
void    CanIoRegisterUnmatchedMessageReceivedCallback(uint8_t channel, tCanIoCallbackUnmatchedMessageReceived callbackPtr);


#endif /* CANIO_H */
/********************************* end of canio.h **************************************/


