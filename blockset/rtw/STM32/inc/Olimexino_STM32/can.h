/************************************************************************************//**
* \file         can.h
* \brief        CAN driver header file.
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
#ifndef CAN_H
#define CAN_H


/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>                                   /* ANSI C types                  */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Whenever this bit is set in a can identifier, it means it is an 29-bit extended
 *         message identifier.
 */
#define CAN_EXTID_MASK      ((uint32_t)0x80000000)
/** \brief Whenever this bit is set in a can identifier, it means it is an remote frame*/
#define CAN_RTR_FRAME_MASK  ((uint32_t)0x40000000)
/** \brief Maximum number of bytes in a CAN message */
#define CAN_MAX_DATA_LEN    ((uint8_t)8)
/** \brief Total number of supported CAN controllers by this driver. */
#define CAN_CHANNELS_MAX                (1)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief CAN error status values. */
typedef enum
{
  CAN_ERR_STAT_RX_OVERFLOW = 0,              /**< reception buffer overflow occurred   */
  CAN_ERR_STAT_BUS_OFF                       /**< bus off event occurred               */
} tCanErrStatus;

/** \brief CAN error counters values. */
typedef enum
{
  CAN_ERR_COUNT_RECEPTION = 0,               /**< receive error counter                */
  CAN_ERR_COUNT_TRANSMISSION                 /**< transmit error counter               */
} tCanErrCounter;

/** \brief CAN acceptance filter mode. */
typedef enum 
{                     
  CAN_FILTER_MODE_STDID_ONLY = 0,            /**< receive only 11-bit CAN id's         */
  CAN_FILTER_MODE_EXTID_ONLY,                /**< receive only 29-bit CAN id's         */
  CAN_FILTER_MODE_MIXED_ID                   /**< receive both 11- and 29-bit CAN id's */
} tCanFilterMode;

/** \brief CAN acceptance filter configuration.
 *  \details The mask part of the filter determines which bits in the received message
 *           identifiers should be matched to the corresponding bits in the code part.
 *           A mask bit value of 0 means don't care. The code part of the filter
 *           determines what bit values to match in the received message identifier.
 *    Example 1 - receive all 11-bit and 29-bit message identifiers.
 *       filter = %X XXXX XXXX XXXX XXXX XXXX XXXX XXXX
 *       .mask = 0x00000000   
 *       .code = 0x00000000 (irrelevant in this case)
 *       .mode = CAN_FILTER_MODE_MIXED_ID
 *
 *    Example 2 - receive only 11-bit message identifiers where bits 0..3 are 0.
 *       filter = %X XXXX XXXX XXXX XXXX XXXX XXXX 0000
 *       .mask = 0x0000000f   
 *       .code = 0x00000000
 *       .mode = CAN_FILTER_MODE_STDID_ONLY
 *
 *    Example 3 - receive only the 29-bit message with identifier 0x0000400e
 *       filter = %0 0000 0000 0000 0100 0000 0000 1110
 *       .mask = 0x1fffffff   
 *       .code = 0x0000400e
 *       .mode = CAN_FILTER_MODE_EXTID_ONLY
 */
typedef struct
{
  uint32_t       mask;                       /**< filter mask setting                  */
  uint32_t       code;                       /**< filter code setting                  */
  tCanFilterMode mode;                       /**< filter mode setting STD/EXT/mixed    */
} tCanFilter;

/** \brief Function type for the generic message transmitted callback handler. */
typedef void (* tCanCallbackGenericTransmitted)(uint32_t id);

/** \brief Function type for the generic message received callback handler. */
typedef void (* tCanCallbackGenericReceived)(uint32_t id, uint8_t dlc, uint8_t *data, uint32_t timestamp);

/** \brief Function type for the error callback handler. */
typedef void (* tCanCallbackError)(tCanErrStatus error);


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void    CanInit(uint16_t transmitBufferSize, uint16_t eventBufferSize);
uint8_t CanConnect(uint8_t channel, uint32_t baud, tCanFilter *filter);
uint8_t CanIsConnected(uint8_t channel);
void    CanDisconnect(uint8_t channel);
uint8_t CanTransmit(uint8_t channel, uint32_t id, uint8_t dlc, uint8_t *data);
void    CanCancelTransmit(uint8_t channel);
uint8_t CanGetErrorCount(uint8_t channel, tCanErrCounter counter);
void    CanRegisterGenericTransmittedCallback(uint8_t channel, tCanCallbackGenericTransmitted callbackPtr);
void    CanRegisterGenericReceivedCallback(uint8_t channel, tCanCallbackGenericReceived callbackPtr);
void    CanRegisterErrorCallback(uint8_t channel, tCanCallbackError callbackPtr);
void    CanRx0Interrupt(void);
void    CanRx1Interrupt(void);
void    CanTxInterrupt(void);
void    CanErrorInterrupt(void);


#endif /* CAN_H */
/********************************* end of can.h ****************************************/


