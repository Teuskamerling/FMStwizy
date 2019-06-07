/****************************************************************************************
* \file         canio.c
* \brief        CAN message input/output source file. This module is an optional add-on
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

/****************************************************************************************
* Include files
****************************************************************************************/
#include "os.h"                                       /* for operating system          */
#include "can.h"                                      /* CAN driver header file        */
#include "canio.h"                                    /* CAN I/O header file           */
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Structure type for the message transmitted callback queue entry. */
typedef struct t_tx_msg_callback_q_entry
{
  uint32_t msgId;                                    /**< message identifier           */
  tCanIoCallbackMessageTransmitted callbackFct;      /**< callback function pointer    */
  struct t_tx_msg_callback_q_entry *nextQEntry;      /**< pointer to next queue entry  */
} tCanIoTransmitMessageCallbackQEntry;

/** \brief Structure type for the message received callback queue entry. */
typedef struct t_rx_msg_callback_q_entry
{
  uint32_t msgId;                                    /**< message identifier           */
  tCanIoCallbackMessageReceived callbackFct;         /**< callback function pointer    */
  struct t_rx_msg_callback_q_entry *nextQEntry;      /**< pointer to next queue entry  */
} tCanIoReceiveMessageCallbackQEntry;

/** \brief Structure type for the message received storage queue entry. */
typedef struct t_rx_msg_storage_q_entry
{
  uint32_t msgId;                                    /**< message identifier           */
  uint8_t  len;                                      /**< message data length code     */
  uint8_t  data[CAN_MAX_DATA_LEN];                   /**< message data byte array      */
  uint32_t timestamp;                                /**< message timestamp            */
  uint8_t  overrunCount;                             /**< message overrun count        */      
  struct t_rx_msg_storage_q_entry *nextQEntry;       /**< pointer to next queue entry  */
} tCanIoReceiveMessageStorageQEntry;


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Linked list that holds all the elements of the transmit callback queue. */
static tCanIoTransmitMessageCallbackQEntry *pCanIoTransmitCallbackQueue;

/** \brief Linked list that holds all the elements of the reception callback queue. */
static tCanIoReceiveMessageCallbackQEntry *pCanIoReceiveCallbackQueue;

/** \brief Linked list that holds all the elements of the reception storage queue. */
static tCanIoReceiveMessageStorageQEntry *pCanIoReceiveMessageStorageQueue;

/** \brief Function pointer for the unmatched message received callback handler. */
static tCanIoCallbackUnmatchedMessageReceived canIoCallbackUnmatchedMessageReceived;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void CanIoGenericMessageTransmittedCallback(uint32_t id);
static void CanIoGenericMessageReceivedCallback(uint32_t id, uint8_t dlc, uint8_t *data, uint32_t timestamp);
static void CanIoAddMessageToStorageQueue(uint32_t id, uint8_t dlc, uint8_t *data, uint32_t timestamp);


/****************************************************************************************
** \brief     Initializes the CAN I/O module.
** \param     channel CAN channel, which is don't care for this target.
** \return    none.
**
****************************************************************************************/
void CanIoInit(uint8_t channel)
{
  /* empty the queue's */
  pCanIoTransmitCallbackQueue = NULL;
  pCanIoReceiveCallbackQueue = NULL;
  pCanIoReceiveMessageStorageQueue = NULL;
  /* invalidate function pointers */
  canIoCallbackUnmatchedMessageReceived = NULL;
  /* register the callback that gets called each time a new message was transmitted */
  CanRegisterGenericTransmittedCallback(0, CanIoGenericMessageTransmittedCallback);
  /* register the callback that gets called each time a new message was received */
  CanRegisterGenericReceivedCallback(0, CanIoGenericMessageReceivedCallback);
} /*** end of CanIoInit ***/


/****************************************************************************************
** \brief     Allocates a RAM buffer for storing the message with this particular
**            identifier. Afterwards, each time a CAN message with this identifier was
**            received, it will automatically be copied to the RAM buffer. The
**            application can retrieve the most recent message related data through
**            function CanIoGetMessageGeneric. Note that only one RAM buffer can be
**            assigned for a specific message identifier.
** \param     channel CAN channel, which is don't care for this target.
** \param     id Message identifier.
** \return    none.
**
****************************************************************************************/
void CanIoCreateMessageReceivedStorage(uint8_t channel, uint32_t id)
{
  tCanIoReceiveMessageStorageQEntry *newQEntry;
  tCanIoReceiveMessageStorageQEntry *lastQEntry;

  /* allocate memory for the queue entry on the heap */
  newQEntry = pvPortMalloc(sizeof(tCanIoReceiveMessageStorageQEntry));
  /* check if the heap wasn't full */
  if (!(newQEntry != NULL))
  {
    ErrCodesSetError(ER_CODE_CANIO_RX_STORAGE_ALLOCATION, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* configure the entry */
  newQEntry->msgId = id;
  newQEntry->overrunCount = 0;
  newQEntry->nextQEntry = NULL;
  /* is this the first queue entry? */
  if (pCanIoReceiveMessageStorageQueue == NULL)
  {
    /* add the new entry at the start of the linked list */
    pCanIoReceiveMessageStorageQueue = newQEntry;
  }
  else
  {
    /* find the last element in the list */
    lastQEntry = pCanIoReceiveMessageStorageQueue;
    while (lastQEntry->nextQEntry != NULL)
    {
      lastQEntry = lastQEntry->nextQEntry;
    }
    /* add the new entry at the end of the linked list */
    lastQEntry->nextQEntry = newQEntry;
  }
} /*** end of CanIoCreateMessageReceivedStorage ***/


/****************************************************************************************
** \brief     Searches through the message storage queue to find the message data for
**            this particular message identifier. If an entry was found, the message
**            data will be copied. Note that this will only work if a RAM buffer was
**            created for a message with this identifier using function
**            CanIoCreateMessageReceivedStorage.
** \param     channel CAN channel, which is don't care for this target.
** \param     id Message identifier to get the message data from.
** \param     dlc Pointer to where the length info of the message will be stored.
** \param     data Pointer to where the data of the message will be stored.
** \param     timestamp Pointer to where the timestamp of the message will be stored.
** \return    0 if no new data is present, -1 in case of error, > 0 in case new data is 
**            present. If the number is > 1 then the message was received multiple times
**            before the data was read out with this function. In this case the most 
**            recent data copied.
**
****************************************************************************************/
int8_t CanIoGetMessage(uint8_t channel, uint32_t id, uint8_t *dlc, uint8_t *data, uint32_t *timestamp)
{
  tCanIoReceiveMessageStorageQEntry *pQEntry;
  tCanIoReceiveMessageStorageQEntry *foundQEntry = NULL;
  uint8_t byteIdx;
  uint8_t result;
  uint32_t saved_cs_state;
  

  /* only continue if the storage queue is not completely empty */
  if (pCanIoReceiveMessageStorageQueue == NULL)
  {
    /* queue empty so nothing to get */
    return -1;
  }

  /* search through the linked list to find a message specific storage entry for the
   * message with this identifier. start by getting the pointer to the first element
   * in the list
   */
  pQEntry = pCanIoReceiveMessageStorageQueue;
  /* is this the one we are looking for? */
  if (pQEntry->msgId == id)
  {
    /* store the entry */
    foundQEntry = pQEntry;
  }
  /* search through the entire list */
  else
  {
    while (pQEntry->nextQEntry != NULL)
    {
      /* move on to the next entry in the linked list */
      pQEntry = pQEntry->nextQEntry;
      /* is this the one we are looking for? */
      if (pQEntry->msgId == id)
      {
        /* store the entry */
        foundQEntry = pQEntry;
        /* element found so all done */
        break;
      }
    }
  }
  /* check if an entry was found */
  if (foundQEntry != NULL)
  {
    /* enter critical section */
    saved_cs_state = OsEnterCriticalSection();
    /* copy the message length */
    if (dlc != NULL)
    {
      *dlc = foundQEntry->len;
    }
    /* copy the message timestamp */
    if (timestamp != NULL)
    {
      *timestamp = foundQEntry->timestamp;
    }
    /* copy the message data */
    if (data != NULL)
    {
      for (byteIdx=0; byteIdx<foundQEntry->len; byteIdx++)
      {
        data[byteIdx] = foundQEntry->data[byteIdx];
      }
    }
    /* read and decrement the overrun counter */
    result = foundQEntry->overrunCount;
    if (foundQEntry->overrunCount > 0)
    {
      foundQEntry->overrunCount--;
    }
    /* leave critical section */
    OsLeaveCriticalSection(saved_cs_state);
    /* message successfully copied */
    return result;
  }
  else
  {
    /* no data found for a message with this identifier */
    return -1;
  }
} /*** end of CanIoGetMessage ***/


/****************************************************************************************
** \brief     Registers a callback function that gets called when a message with the
**            specified identifier was successfully transmitted. Note that only one
**            callback per message identifier can be registered.
** \param     channel CAN channel, which is don't care for this target.
** \param     id Message identifier.
** \param     callbackPtr Pointer to the callback function.
** \return    none.
**
****************************************************************************************/
void CanIoRegisterMessageTransmittedCallback(uint8_t channel, uint32_t id, tCanIoCallbackMessageTransmitted callbackPtr)
{
  tCanIoTransmitMessageCallbackQEntry *newQEntry;
  tCanIoTransmitMessageCallbackQEntry *lastQEntry;

  /* allocate memory for the queue entry on the heap */
  newQEntry = pvPortMalloc(sizeof(tCanIoTransmitMessageCallbackQEntry));
  /* check if the heap wasn't full */
  if (!(newQEntry != NULL))
  {
    ErrCodesSetError(ER_CODE_CANIO_TX_CALLBACK_ALLOCATION, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* configure the entry */
  newQEntry->msgId = id;
  newQEntry->callbackFct = callbackPtr;
  newQEntry->nextQEntry = NULL;
  /* is this the first queue entry? */
  if (pCanIoTransmitCallbackQueue == NULL)
  {
    /* add the new entry at the start of the linked list */
    pCanIoTransmitCallbackQueue = newQEntry;
  }
  else
  {
    /* find the last element in the list */
    lastQEntry = pCanIoTransmitCallbackQueue;
    while (lastQEntry->nextQEntry != NULL)
    {
      lastQEntry = lastQEntry->nextQEntry;
    }
    /* add the new entry at the end of the linked list */
    lastQEntry->nextQEntry = newQEntry;
  }
} /*** end of CanIoRegisterMessageTransmittedCallback ***/


/****************************************************************************************
** \brief     Registers a callback function that gets called when a message with the
**            specified identifier was received. NOte that only one callback per message
**            identifier can be registered.
** \param     channel CAN channel, which is don't care for this target.
** \param     id Message identifier.
** \param     callbackPtr Pointer to the callback function.
** \return    none.
**
****************************************************************************************/
void CanIoRegisterMessageReceivedCallback(uint8_t channel, uint32_t id, tCanIoCallbackMessageReceived callbackPtr)
{
  tCanIoReceiveMessageCallbackQEntry *newQEntry;
  tCanIoReceiveMessageCallbackQEntry *lastQEntry;

  /* allocate memory for the queue entry on the heap */
  newQEntry = pvPortMalloc(sizeof(tCanIoReceiveMessageCallbackQEntry));
  /* check if the heap wasn't full */
  if (!(newQEntry != NULL))
  {
    ErrCodesSetError(ER_CODE_CANIO_RX_CALLBACK_ALLOCATION, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* configure the entry */
  newQEntry->msgId = id;
  newQEntry->callbackFct = callbackPtr;
  newQEntry->nextQEntry = NULL;
  /* is this the first queue entry? */
  if (pCanIoReceiveCallbackQueue == NULL)
  {
    /* add the new entry at the start of the linked list */
    pCanIoReceiveCallbackQueue = newQEntry;
  }
  else
  {
    /* find the last element in the list */
    lastQEntry = pCanIoReceiveCallbackQueue;
    while (lastQEntry->nextQEntry != NULL)
    {
      lastQEntry = lastQEntry->nextQEntry;
    }
    /* add the new entry at the end of the linked list */
    lastQEntry->nextQEntry = newQEntry;
  }
} /*** end of CanIoRegisterMessageReceivedCallback ***/


/****************************************************************************************
** \brief     Registers a callback function that gets called when a message was received
**            for which no specific reception callback was registered with function
**            CanIoRegisterMessageReceivedCallback. This callback can be used as a catch
**            all for unregistered messages.
** \param     channel CAN channel, which is don't care for this target.
** \param     callbackPtr Pointer to the callback function.
** \return    none.
**
****************************************************************************************/
void CanIoRegisterUnmatchedMessageReceivedCallback(uint8_t channel, tCanIoCallbackUnmatchedMessageReceived callbackPtr)
{
  canIoCallbackUnmatchedMessageReceived = callbackPtr;
} /*** end of CanIoRegisterMessageReceivedCallback ***/



/****************************************************************************************
** \brief     Callback hander that gets called by the CAN driver each time a message
**            was transmitted. This function then locates a message identifier specific
**            callback handler.
** \param     id Message identifier.
** \return    none.
**
****************************************************************************************/
static void CanIoGenericMessageTransmittedCallback(uint32_t id)
{
  tCanIoTransmitMessageCallbackQEntry *pQEntry;
  tCanIoTransmitMessageCallbackQEntry *foundQEntry = NULL;

  /* search through the linked list to find a message specific callback for the message
   * with this identifier.
   */
  if (pCanIoTransmitCallbackQueue != NULL)
  {
    /* get pointer to the first element in the linked list */
    pQEntry = pCanIoTransmitCallbackQueue;
    /* is this the one we are looking for? */
    if (pQEntry->msgId == id)
    {
      /* store the entry */
      foundQEntry = pQEntry;
    }
    /* search through the entire list */
    else
    {
      while (pQEntry->nextQEntry != NULL)
      {
        /* move on to the next entry in the linked list */
        pQEntry = pQEntry->nextQEntry;
        /* is this the one we are looking for? */
        if (pQEntry->msgId == id)
        {
          /* store the entry */
          foundQEntry = pQEntry;
          /* element found so all done */
          break;
        }
      }
    }
    /* check if an entry was found */
    if (foundQEntry != NULL)
    {
      /* invoke the callback */
      if (foundQEntry->callbackFct != NULL)
      {
        foundQEntry->callbackFct();
      }
    }
  }
} /*** end of CanIoGenericMessageTransmittedCallback ***/


/****************************************************************************************
** \brief     Callback hander that gets called by the CAN driver each time a message
**            was received. This function then locates a message identifier specific
**            callback handler.
** \param     id Message identifier.
** \param     dlc Message data length code.
** \param     data Byte array with message data.
** \param     timestamp Timestamp of the received message in ms.
** \return    none.
**
****************************************************************************************/
static void CanIoGenericMessageReceivedCallback(uint32_t id, uint8_t dlc, uint8_t *data, uint32_t timestamp)
{
  tCanIoReceiveMessageCallbackQEntry *pQEntry;
  tCanIoReceiveMessageCallbackQEntry *foundQEntry = NULL;

  /* add the message data to the storage queue */
  CanIoAddMessageToStorageQueue(id, dlc, data, timestamp);

  /* search through the linked list to find a message specific callback for the message
   * with this identifier.
   */
  if (pCanIoReceiveCallbackQueue != NULL)
  {
    /* get pointer to the first element in the linked list */
    pQEntry = pCanIoReceiveCallbackQueue;
    /* is this the one we are looking for? */
    if (pQEntry->msgId == id)
    {
      /* store the entry */
      foundQEntry = pQEntry;
    }
    /* search through the entire list */
    else
    {
      while (pQEntry->nextQEntry != NULL)
      {
        /* move on to the next entry in the linked list */
        pQEntry = pQEntry->nextQEntry;
        /* is this the one we are looking for? */
        if (pQEntry->msgId == id)
        {
          /* store the entry */
          foundQEntry = pQEntry;
          /* element found so all done */
          break;
        }
      }
    }
    /* check if an entry was found */
    if (foundQEntry != NULL)
    {
      /* invoke the callback */
      if (foundQEntry->callbackFct != NULL)
      {
        foundQEntry->callbackFct(dlc, data, timestamp);
      }
    }
  }
  /* no callback function found? */
  if (foundQEntry == NULL)
  {
    /* no callback was registered for a message with this identifier so invoke the
     * message unmatch callback, if valid.
     */
    if (canIoCallbackUnmatchedMessageReceived != NULL)
    {
      canIoCallbackUnmatchedMessageReceived(id, dlc, data, timestamp);
    }
  }
} /*** end of CanIoGenericMessageReceivedCallback ***/


/****************************************************************************************
** \brief     This function attempts to located the storage buffer for a message with
**            this identifier and overwrites the old data with the newly received data.
**            If no storage buffer is available, a new one will be allocated on the heap.
** \param     id Message identifier.
** \param     dlc Message data length code.
** \param     data Byte array with message data.
** \param     timestamp Timestamp of the received message in ms.
** \return    none.
**
****************************************************************************************/
static void CanIoAddMessageToStorageQueue(uint32_t id, uint8_t dlc, uint8_t *data, uint32_t timestamp)
{
  tCanIoReceiveMessageStorageQEntry *pQEntry;
  uint8_t byteIdx;
  uint32_t saved_cs_state;

  /* is the storage queue completely empty? */
  if (pCanIoReceiveMessageStorageQueue != NULL)
  {
    /* get pointer to the first element in the linked list */
    pQEntry = pCanIoReceiveMessageStorageQueue;
    /* is this the one we are looking for? */
    if (pQEntry->msgId == id)
    {
      /* enter critical section */
      saved_cs_state = OsEnterCriticalSection();
      /* store the new message data */
      pQEntry->len = dlc;
      for (byteIdx=0; byteIdx<dlc; byteIdx++)
      {
        pQEntry->data[byteIdx] = data[byteIdx];
      }
      pQEntry->timestamp = timestamp;
      /* increment the overrun counter */
      if (pQEntry->overrunCount < 127)
      {
        pQEntry->overrunCount++;
      }
      /* leave critical section */
      OsLeaveCriticalSection(saved_cs_state);
    }
    /* search through the entire list */
    else
    {
      while (pQEntry->nextQEntry != NULL)
      {
        /* move on to the next entry in the linked list */
        pQEntry = pQEntry->nextQEntry;
        /* is this the one we are looking for? */
        if (pQEntry->msgId == id)
        {
          /* enter critical section */
          saved_cs_state = OsEnterCriticalSection();
          /* store the new message data */
          pQEntry->len = dlc;
          for (byteIdx=0; byteIdx<dlc; byteIdx++)
          {
            pQEntry->data[byteIdx] = data[byteIdx];
          }
          pQEntry->timestamp = timestamp;
          /* increment the overrun counter */
          if (pQEntry->overrunCount < 127)
          {
            pQEntry->overrunCount++;
          }
          /* leave critical section */
          OsLeaveCriticalSection(saved_cs_state);
          /* element found so all done */
          break;
        }
      }
    }
  }
} /*** end of CanIoAddMessageToStorageQueue ***/


/************************************ end of canio.c ***********************************/


