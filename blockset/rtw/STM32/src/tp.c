/************************************************************************************//**
* \file         tp.c
* \brief        Transport protocol source file. 
* \details      This module enables transmission/reception of up to 4095 data bytes on 
*               the CAN network, including the reporting of transmission/reception 
*               completion (or failure), according to ISO 15765-2:2011.
*
*               Implementation notes and limitations
*               ------------------------------------
*                - Supports physical addressing and not functional.
*                - Supports normal addressing and not normal fixed or extended.
*                - Does data padding so the DLC of all TP related CAN messages is 8.
*
*                Example API usage
*                -----------------
*                - Initialization:
*                  * The following example initializes the module to use CAN channel 0
*                    of the CAN driver for TP related communication. The module needs
*                    to know the CAN identifier for transmitting and receiving TP
*                    related messages. 0x603 and 0x614 are used for this respectively.
*                    During segmented reception, a block size (BS) of 4 is demanded
*                    and a minimum separation time (STmin) of 40 milliseconds. For the
*                    internal reception buffer, 1024 bytes are reserved. So the largest
*                    data reception that this module can handle is 1024 bytes.
*                      TpInit(0, 0x603, 0x6F1, 4, 40, 1024);
*
*                - Data transmission:
*                  * The following example show how 32 data are transmitted using the
*                    TP module:
*
*                     uint8_t testData[] = {
*                                 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
*                                 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
*                                 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
*                                 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F };
*
*                     TpTransfer(testData, 32);
*
*                   * It is possible to register a function that gets called by this
*                     module after the transmission completed, or was aborted due to
*                     an error. Here follows an example of such a function and how
*                     it is registered:
*
*                      void TestTpCallbackTransferComplete(tTpResult result)
*                      {
*                        if (result == TP_RESULT_OK)
*                        {
*                          // handle transfer completion..
*                        }
*                        else
*                        {
*                          // handle transfer error..
*                        }
*                      }
*
*                      TpRegisterTransferCompleteCallback(TestTpCallbackTransferComplete);
*
*                - Data reception:
*                  * All received data is stored in the internal reception buffer and
*                    handled internally by this module. The moment a reception completed,
*                    a callback function is called to pass the received data on to a
*                    higher layer. Here follows an example of such a function and how
*                    it is registered:
*
*                     void TestTpCallbackDataReceived(uint8_t *data, uint16_t len,
*                                                     tTpResult result)
*                     {
*                       uint16_t idx;
*
*                       if (result == TP_RESULT_OK)
*                       {
*                         // process newly received data
*                       }
*                     }
*
*                     TpRegisterDataReceivedCallback(TestTpCallbackDataReceived);
*
*                  * A segmented reception is started when a First Frame message was
*                    received. An optional callback function can be used when a higher
*                    layer should be informed of this event. Here follows an example of
*                    such a function and how it is registered:
*
*                     void TestTpCallbackFirstFrameReceived(uint16_t len)
*                     {
*                       // process start of segmented reception event
*                     }
*
*                     TpRegisterFirstFrameReceivedCallback(TestTpCallbackFirstFrameReceived);
*
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
 \endinternal
****************************************************************************************/


/****************************************************************************************
* Include files
****************************************************************************************/
#include "tp.h"                                       /* for transport protocol        */
#include "os.h"                                       /* for operating system          */
#include "canio.h"                                    /* for CAN I/O interface         */
#include "errorcodes.h"                               /* error codes module            */
#include "errorList.h"                                /* for error list                */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Maximum data that can be transferred by the transport protocol. */
#define TP_MAX_TRANSFER_LEN       (4095u)

/** \brief Number of data bytes that can fit in a single frame message. */
#define TP_MAX_SINGLE_FRAME_DATA  (CAN_MAX_DATA_LEN - 1u)

/** \brief Number of data bytes that can fit in a first frame message. */
#define TP_MAX_FIRST_FRAME_DATA   (CAN_MAX_DATA_LEN - 2u)

/** \brief Number of data bytes that can fit in a consecutive frame message. */
#define TP_MAX_CONSEC_FRAME_DATA  (CAN_MAX_DATA_LEN - 1u)

/** \brief Single frame protocol control information specification. */
#define TP_PCI_SINGLE_FRAME       (0u)

/** \brief First frame protocol control information specification. */
#define TP_PCI_FIRST_FRAME        (1u)

/** \brief Consecutive frame protocol control information specification. */
#define TP_PCI_CONSECUTIVE_FRAME  (2u)

/** \brief Flow control protocol control information specification. */
#define TP_PCI_FLOW_CONTROL       (3u)

/** \brief Continue to send flow status specification. */
#define TP_FS_CTS                 (0u)

/** \brief Wait flow status specification. */
#define TP_FS_WAIT                (1u)

/** \brief Buffer overflow flow status specification. */
#define TP_FS_OVFLW               (2u)

/** \brief Protocol defined timeout time for the reception of a flow control. */
#define TP_BS_TIMER_TIMEOUT_MS    (1000u)

/** \brief Protocol defined timeout time for the transmission of a CAN frame on the
 *         sender side.
 */
#define TP_AS_TIMER_TIMEOUT_MS    (1000u)

/** \brief Protocol defined timeout time for the transmission of a CAN frame on the
 *         receiver side.
 */
#define TP_AR_TIMER_TIMEOUT_MS    (1000u)

/** \brief Protocol defined timeout time for the reception of a consecutive frame. */
#define TP_CR_TIMER_TIMEOUT_MS    (1000u)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Structure type for the transport protocol packet data units (PDU). */
typedef struct
{
  /** \brief Raw byte-by-byte access of the data. */
  uint8_t data[CAN_MAX_DATA_LEN];
} tTpPDU;

/** \brief Possible states for the transmit path. */
typedef enum
{
  TP_TX_STATE_IDLE,                    /**< transmit path is idle                      */
  TP_TX_STATE_EXPECT_SINGLEFRAME_CONF, /**< waiting for single frame confirmation      */
  TP_TX_STATE_EXPECT_FIRSTFRAME_CONF,  /**< waiting for first frame confirmation       */
  TP_TX_STATE_EXPECT_FLOWCONTROL_IND,  /**< waiting for flow control indication        */
  TP_TX_STATE_CLEAR_TO_SEND            /**< sending consecutive frames.                */
} tTpTxState;

/** \brief Possible states for the reception path. */
typedef enum
{
  TP_RX_STATE_IDLE,                    /**< reception path is idle                     */
  TP_RX_STATE_EXPECT_FLOWCONTROL_CONF, /**< waiting for flow control confirmation      */
  TP_RX_STATE_EXPECT_CONS_FRAME_IND,   /**< waiting for consecutive frame indication   */
} tTpRxState;

/** \brief Structure type for the transport protocol communication settings. */
typedef struct
{
  uint8_t  channel;                    /**< CAN channel to use                         */
  uint32_t txId;                       /**< CAN transmit message identifier            */
  uint32_t rxId;                       /**< CAN receive message identifier             */
} tTpCommSettings;

/** \brief Structure type for the transport protocol transmit path settings. */
typedef struct
{
  tTpTxState state;                    /**< transmit state                             */
  uint16_t   transferLen;              /**< total length of the transfer               */
  uint16_t   transmitted;              /**< number of already transmitted bytes        */
  uint8_t   *transferData;             /**< pointer to byte array with transfer data   */
  tTpPDU     transmitPdu;              /**< transmit packet data unit (PDU)            */
  tTpPDU     receivePdu;               /**< receive packet data unit (PDU)             */
  uint8_t    stMin;                    /**< min. separation time to use (ms).          */
  uint8_t    blockSize;                /**< block size to use.                         */
  uint8_t    seqNum;                   /**< sequence number for consecutive frame      */
  uint8_t    cfCount;                  /**< counter of sent consecutive frames         */
  uint8_t    cfSepTimer;               /**< separation timer for consecutive frames    */
  uint16_t   bsTimer;                  /**< timeout timer for flow control reception   */
  uint16_t   asTimer;                  /**< timeout timer for frame transmission       */
} tTpTxSettings;

typedef struct
{
  uint16_t size;                       /**< total number of bytes that can fit         */
  uint16_t entries;                    /**< number of bytes stored in the buffer       */
  uint8_t *data;                       /**< pointer to the start of the buffer         */
} tTpRxBuffer;

/** \brief Structure type for the transport protocol reception path settings. */
typedef struct
{
  tTpRxState      state;               /**< reception state                            */
  uint8_t         blockSize;           /**< consecutive frames before flow control     */
  uint8_t         stMin;               /**< min. time (ms) between consecutive frames  */
  tTpRxBuffer     buffer;              /**< FIFO buffer for data reception             */
  tTpPDU          transmitPdu;         /**< transmit packet data unit (PDU)            */
  tTpPDU          receivePdu;          /**< receive packet data unit (PDU)             */
  uint16_t        transferLen;         /**< total length of the transfer               */
  uint16_t        received;            /**< number of already received bytes           */
  uint8_t         seqNum;              /**< sequence number for consecutive frame      */
  uint8_t         cfCount;             /**< counter of received consecutive frames     */
  uint16_t        arTimer;             /**< timeout timer for frame transmission       */
  uint16_t        crTimer;             /**< timeout timer for consecutive frame rx     */
} tTpRxSettings;


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Function pointer for the first frame received callback handler. */
static tTpCallbackFirstFrameReceived tpCallbackFirstFrameReceived;

/** \brief Function pointer for the data received callback handler. */
static tTpCallbackDataReceived tpCallbackDataReceived;

/** \brief Function pointer for the transfer complete callback handler. */
static tTpCallbackTransferComplete tpCallbackTransferComplete;

/** \brief Communication settings shared by data reception and transmission. */
static tTpCommSettings tpCommSettings;

/** \brief Transmit path settings. */
static tTpTxSettings tpTxSettings;

/** \brief Reception path settings. */
static tTpRxSettings tpRxSettings;

/** \brief Timer handle for the 1 millisecond task timer. */
static xTimerHandle taskTimer1msHandle;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void TpMillisecondTimerCallback(xTimerHandle pxTimer);
static void TpTxMsTask(void);
static void TpRxMsTask(void);
static uint8_t TpCanTransmit(uint8_t *data);
static void TpCanReceived(uint8_t dlc, uint8_t *data, uint32_t timestamp);
static void TpCanTransmitted(void);
static void TpClearPduData(uint8_t *data);
static void TpSetPciType(tTpPDU *pdu, uint8_t type);
static uint8_t TpGetPciType(tTpPDU *pdu);
static void TpSetSFLen(tTpPDU *pdu, uint8_t len);
static uint8_t TpGetSFLen(tTpPDU *pdu);
static void TpSetFFLen(tTpPDU *pdu, uint16_t len);
static uint16_t TpGetFFLen(tTpPDU *pdu);
static void TpSetCFSeqNum(tTpPDU *pdu, uint8_t sn);
static uint8_t TpGetCFSeqNum(tTpPDU *pdu);
static void TpSetFCStatus(tTpPDU *pdu, uint8_t status);
static uint8_t TpGetFCStatus(tTpPDU *pdu);
static void TpSetFCBlockSize(tTpPDU *pdu, uint8_t bs);
static uint8_t TpGetFCBlockSize(tTpPDU *pdu);
static void TpSetFCStMin(tTpPDU *pdu, uint8_t st_min);
static uint8_t TpGetFCStMin(tTpPDU *pdu);
static uint8_t TpCheckAndCorrectStMin(uint8_t stMin);
static void TpRxBufferInit(uint16_t bufferSize);
static void TpRxBufferFlush(void);
static uint16_t TpRxBufferScan(void);
static uint8_t *TpRxBufferGetDataPtr(void);
static uint8_t TpRxBufferWrite(uint8_t *data, uint16_t len);


/************************************************************************************//**
** \brief     Initializes the transport protocol module.
** \param     channel The CAN channel that is to be used for data transfers.
** \param     transmit_id The CAN identifier to use for transmitting transport protocol
**                        messages.
** \param     receive_id The CAN identifier to use for receiving transport protocol
**                       messages.
** \param     blockSize  The number of consecutive frames we expect to receive before we
**                       send a flow control message (data reception).
** \param     STmin Minimum time we demand in milliseconds between 2 consecutive frames
**                  (data reception).
** \param     receiveBufferSize Size of the internal buffer that is used for data
**                              reception. This also determines the maximum size of
**                              possible data transfer to this module.
** \return    none.
**
****************************************************************************************/
void TpInit(uint8_t channel, uint32_t transmit_id, uint32_t receive_id,
            uint8_t blockSize, uint8_t STmin, uint16_t receiveBufferSize)
{
  portBASE_TYPE result;

  /* reset callback function pointers */
  tpCallbackFirstFrameReceived = NULL;
  tpCallbackDataReceived = NULL;
  tpCallbackTransferComplete = NULL;
  /* store the generic communication settings */
  tpCommSettings.channel = channel;
  tpCommSettings.txId = transmit_id;
  tpCommSettings.rxId = receive_id;
  /* initialize the transmit settings */
  tpTxSettings.state = TP_TX_STATE_IDLE;
  /* initialize the reception settings */
  tpRxSettings.state = TP_RX_STATE_IDLE;
  tpRxSettings.blockSize = blockSize;
  tpRxSettings.stMin = STmin;
  TpRxBufferInit(receiveBufferSize);
  /* register callbacks for handling the message transmit completion and message
   * reception of the transport protocol related CAN messages.
   */
  CanIoRegisterMessageTransmittedCallback(tpCommSettings.channel, tpCommSettings.txId, TpCanTransmitted);
  CanIoRegisterMessageReceivedCallback(tpCommSettings.channel, tpCommSettings.rxId, TpCanReceived);

  /* create the 1 millisecond task timer */
  taskTimer1msHandle = xTimerCreate("tpTimer", (((portTickType)1000)/portTICK_PERIOD_US),
                              pdTRUE, NULL, TpMillisecondTimerCallback);
  if (taskTimer1msHandle == NULL)
  {
    ErrCodesSetError(ER_CODE_TP_TASK_TIMER_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* start the 1 millisecond task timer. note that block time must be 0 since it could
   * be that the scheduler hasn't been started yet.
   */
  result = xTimerStart(taskTimer1msHandle, 0);
  if (!(result == pdPASS))
  {
    ErrCodesSetError(ER_CODE_TP_TASK_TIMER_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
} /*** end of TpInit ***/


/************************************************************************************//**
** \brief     Initiates a data transfer.
** \param     data Pointer to the buffer with data bytes to transfer. Note that the data
**            that this pointer points to must stay valid during the (segmented)
**            transfer, so basically until the transfer complete callback is invoked.
** \param     len Total number of data bytes to transfer.
** \return    TRUE if the data transfer request was successfully started, FALSE
**            otherwise.
**
****************************************************************************************/
uint8_t TpTransfer(uint8_t *data, uint16_t len)
{
  uint32_t saved_cs_state;
  uint8_t result = TRUE;
  uint8_t idx;

  /* a transfer can only be started if the transmit path is available and if the
   * specified length is supported by the protocol.
   */
  saved_cs_state = OsEnterCriticalSection();
  if ( (tpTxSettings.state != TP_TX_STATE_IDLE) || (len == 0) || (len > TP_MAX_TRANSFER_LEN) )
  {
    OsLeaveCriticalSection(saved_cs_state);
    result = FALSE;
  }
  else
  {
    /* does all data fit in a single frame message? */
    if (len <= TP_MAX_SINGLE_FRAME_DATA)
    {
      /* init the transfer information and update the state */
      tpTxSettings.asTimer = 0;
      tpTxSettings.bsTimer = 0;
      tpTxSettings.transferLen = len;
      tpTxSettings.transmitted = 0;
      tpTxSettings.transferData = data;
      tpTxSettings.state = TP_TX_STATE_EXPECT_SINGLEFRAME_CONF;
      OsLeaveCriticalSection(saved_cs_state);
      /* clear the PDU data */
      TpClearPduData(tpTxSettings.transmitPdu.data);
      /* prepare the single frame PDU */
      TpSetPciType(&tpTxSettings.transmitPdu, TP_PCI_SINGLE_FRAME);
      TpSetSFLen(&tpTxSettings.transmitPdu, len);
      for (idx=0; idx<len; idx++)
      {
        tpTxSettings.transmitPdu.data[idx+1] = tpTxSettings.transferData[idx];
      }
      /* submit the single frame for transmission */
      if (TpCanTransmit(tpTxSettings.transmitPdu.data) == FALSE)
      {
        /* set state back to idle */
        tpTxSettings.state = TP_TX_STATE_IDLE;
        result = FALSE;
      }
      /* submit in progress, start the timer */
      tpTxSettings.asTimer = TP_AS_TIMER_TIMEOUT_MS;
    }
    /* data too large for single frame so start segmented transfer with a first frame */
    else
    {
      /* init the transfer information and update the state */
      tpTxSettings.asTimer = 0;
      tpTxSettings.bsTimer = 0;
      tpTxSettings.transferLen = len;
      tpTxSettings.transmitted = 0;
      tpTxSettings.transferData = data;
      tpTxSettings.state = TP_TX_STATE_EXPECT_FIRSTFRAME_CONF;
      OsLeaveCriticalSection(saved_cs_state);
      /* clear the PDU data */
      TpClearPduData(tpTxSettings.transmitPdu.data);
      /* prepare the first frame PDU */
      TpSetPciType(&tpTxSettings.transmitPdu, TP_PCI_FIRST_FRAME);
      TpSetFFLen(&tpTxSettings.transmitPdu, len);
      for (idx=0; idx<TP_MAX_FIRST_FRAME_DATA; idx++)
      {
        tpTxSettings.transmitPdu.data[idx+2] = tpTxSettings.transferData[idx];
      }
      /* submit the first frame for transmission */
      if (TpCanTransmit(tpTxSettings.transmitPdu.data) == FALSE)
      {
        /* set state back to idle */
        tpTxSettings.state = TP_TX_STATE_IDLE;
        result = FALSE;
      }
      /* submit in progress, start the timer */
      tpTxSettings.asTimer = TP_AS_TIMER_TIMEOUT_MS;
    }
  }
  /* all done */
  return result;
} /*** end of TpTransfer ***/


/************************************************************************************//**
** \brief     Utility function for dynamically changing the communication parameters.
** \param     id Identifier of the parameter to change (TP_PARAM_ID_xxx).
** \param     value New parameter value.
** \return    Result of the parameter change operation (TP_PARAM_CHANGE_xxx).
**
****************************************************************************************/
tTpParamChangeResult TpChangeParameter(tTpParamId id, uint8_t value)
{
  tTpParamChangeResult result = TP_PARAM_CHANGE_RX_ON;
  uint32_t saved_cs_state;

  /* only allow parameter changes if no segmented reception is in progress */
  saved_cs_state = OsEnterCriticalSection();
  if (tpRxSettings.state != TP_RX_STATE_IDLE)
  {
    /* filter on supported parameters */
    switch(id)
    {
      case TP_PARAM_ID_BLOCKSIZE:
        /* change the blocksize used during segmented data reception */
        tpRxSettings.blockSize = value;
        result = TP_PARAM_CHANGE_OK;
        break;

      case TP_PARAM_ID_STMIN:
        /* change the STmin time used during segmented data reception */
        tpRxSettings.stMin = TpCheckAndCorrectStMin(value);
        result = TP_PARAM_CHANGE_OK;
        break;

      default:
        result = TP_PARAM_CHANGE_WRONG_PARAMETER;
        break;
    }
  }
  OsLeaveCriticalSection(saved_cs_state);
  return result;
} /*** end of TpChangeParameter ***/


/************************************************************************************//**
** \brief     Registration of the callback function that gets called each time a first
**            frame is received.
** \param     callbackPtr Pointer to the callback function.
** \return    none.
**
****************************************************************************************/
void TpRegisterFirstFrameReceivedCallback(tTpCallbackFirstFrameReceived callbackPtr)
{
  /* store the callback's function pointer */
  tpCallbackFirstFrameReceived = callbackPtr;
} /*** end of TpRegisterFirstFrameReceivedCallback ***/


/************************************************************************************//**
** \brief     Registration of the callback function that gets called each time data was
**            received.
** \param     callbackPtr Pointer to the callback function.
** \return    none.
**
****************************************************************************************/
void TpRegisterDataReceivedCallback(tTpCallbackDataReceived callbackPtr)
{
  /* store the callback's function pointer */
  tpCallbackDataReceived = callbackPtr;
} /*** end of TpRegisterDataReceivedCallback ***/


/************************************************************************************//**
** \brief     Registration of the callback function that gets called each time a data
**            transfer was completed.
** \param     callbackPtr Pointer to the callback function.
** \return    none.
**
****************************************************************************************/
void TpRegisterTransferCompleteCallback(tTpCallbackTransferComplete callbackPtr)
{
  /* store the callback's function pointer */
  tpCallbackTransferComplete = callbackPtr;
} /*** end of TpRegisterTransferCompleteCallback ***/


/************************************************************************************//**
** \brief     Timer callback function that should be called once a milliseconds. This
**            function drives the data transfers and handles timeout monitoring.
** \param     pxTimer Handle of the timer that caused the callback to be called.
** \return    none.
**
****************************************************************************************/
static void TpMillisecondTimerCallback(xTimerHandle pxTimer)
{
  if (pxTimer == taskTimer1msHandle)
  {
    /* update the transmit path */
    TpTxMsTask();
    /* update the reception path */
    TpRxMsTask();
  }
} /*** end of TpMsTask ***/


/************************************************************************************//**
** \brief     Task function that should be called once a milliseconds for driving the
**            transmit path.
** \return    none.
**
****************************************************************************************/
static void TpTxMsTask(void)
{
  uint8_t cfDataLen;
  uint8_t idx;

  /* ============================= Transmit Path ===================================== */
  /* only need to run this task if the transmit path is not idle */
  if (tpTxSettings.state != TP_TX_STATE_IDLE)
  {
    /* ------------------- TP_TX_STATE_EXPECT_FLOWCONTROL_IND ------------------------ */
    if (tpTxSettings.state == TP_TX_STATE_EXPECT_FLOWCONTROL_IND)
    {
      /* update Bs timer which monitors timeout of flow control reception */
      if (tpTxSettings.bsTimer > 0)
      {
        /* 1 ms passed, so decrement it */
        tpTxSettings.bsTimer--;
        /* did it timeout? */
        if (tpTxSettings.bsTimer == 0)
        {
          /* set state back to idle */
          tpTxSettings.state = TP_TX_STATE_IDLE;
          /* invoke callback to inform about the transfer abort */
          if (tpCallbackTransferComplete != NULL)
          {
            tpCallbackTransferComplete(TP_RESULT_TIMEOUT_BS);
          }
        }
      }
    }
    /* ------------------- TP_TX_STATE_EXPECT_FLOWCONTROL_IND ------------------------ */
    else if (tpTxSettings.state == TP_TX_STATE_EXPECT_SINGLEFRAME_CONF)
    {
      /* update As timer which monitors timeout of CAN frame transmission */
      if (tpTxSettings.asTimer > 0)
      {
        /* 1 ms passed, so decrement it */
        tpTxSettings.asTimer--;
        /* did it timeout? */
        if (tpTxSettings.asTimer == 0)
        {
          /* set state back to idle */
          tpTxSettings.state = TP_TX_STATE_IDLE;
          /* invoke callback to inform about the transfer abort */
          if (tpCallbackTransferComplete != NULL)
          {
            tpCallbackTransferComplete(TP_RESULT_TIMEOUT_A);
          }
        }
      }
    }
    /* ------------------- TP_TX_STATE_EXPECT_FIRSTFRAME_CONF ------------------------ */
    else if (tpTxSettings.state == TP_TX_STATE_EXPECT_FIRSTFRAME_CONF)
    {
      /* update As timer which monitors timeout of CAN frame transmission */
      if (tpTxSettings.asTimer > 0)
      {
        /* 1 ms passed, so decrement it */
        tpTxSettings.asTimer--;
        /* did it timeout? */
        if (tpTxSettings.asTimer == 0)
        {
          /* set state back to idle */
          tpTxSettings.state = TP_TX_STATE_IDLE;
          /* invoke callback to inform about the transfer abort */
          if (tpCallbackTransferComplete != NULL)
          {
            tpCallbackTransferComplete(TP_RESULT_TIMEOUT_A);
          }
        }
      }
    }
    /* ------------------- TP_TX_STATE_CLEAR_TO_SEND --------------------------------- */
    else if (tpTxSettings.state == TP_TX_STATE_CLEAR_TO_SEND)
    {
      /* update As timer which monitors timeout of CAN frame transmission */
      if (tpTxSettings.asTimer > 0)
      {
        /* 1 ms passed, so decrement it */
        tpTxSettings.asTimer--;
        /* did it timeout? */
        if (tpTxSettings.asTimer == 0)
        {
          /* set state back to idle */
          tpTxSettings.state = TP_TX_STATE_IDLE;
          /* invoke callback to inform about the transfer abort */
          if (tpCallbackTransferComplete != NULL)
          {
            tpCallbackTransferComplete(TP_RESULT_TIMEOUT_A);
          }
        }
      }
      /* update separation timer */
      if (tpTxSettings.cfSepTimer > 0)
      {
        tpTxSettings.cfSepTimer--;
      }
      /* check if the separation timer timed out, meaning that we can send the next
       * consecutive frame.
       */
      if (tpTxSettings.cfSepTimer == 0)
      {
        /* increment the sequence number with overflow handling */
        tpTxSettings.seqNum++;
        if (tpTxSettings.seqNum > 0x0f)
        {
          tpTxSettings.seqNum = 0;
        }
        /* determine number of bytes for the consecutive frame */
        if ( (tpTxSettings.transferLen - tpTxSettings.transmitted) >= (CAN_MAX_DATA_LEN - 1) )
        {
          cfDataLen = CAN_MAX_DATA_LEN - 1;
        }
        else
        {
          cfDataLen = tpTxSettings.transferLen - tpTxSettings.transmitted;
        }
        /* clear the PDU data */
        TpClearPduData(tpTxSettings.transmitPdu.data);
        /* prepare the consecutive frame */
        TpSetPciType(&tpTxSettings.transmitPdu, TP_PCI_CONSECUTIVE_FRAME);
        TpSetCFSeqNum(&tpTxSettings.transmitPdu, tpTxSettings.seqNum);
        /* copy actual data */
        for (idx=0; idx<cfDataLen; idx++)
        {
          tpTxSettings.transmitPdu.data[idx+1] = tpTxSettings.transferData[tpTxSettings.transmitted + idx];
        }
        /* submit the consecutive frame for transmission */
        if (TpCanTransmit(tpTxSettings.transmitPdu.data) == FALSE)
        {
          /* set state back to idle */
          tpTxSettings.state = TP_TX_STATE_IDLE;
          /* invoke callback to inform about the transfer abort */
          if (tpCallbackTransferComplete != NULL)
          {
            tpCallbackTransferComplete(TP_RESULT_ERROR);
          }
        }
        /* submit in progress, start the timer */
        tpTxSettings.asTimer = TP_AS_TIMER_TIMEOUT_MS;
      }
    }
  }
} /*** end of TpTxMsTask ***/


/************************************************************************************//**
** \brief     Task function that should be called once a milliseconds for driving the
**            reception path.
** \return    none.
**
****************************************************************************************/
static void TpRxMsTask(void)
{
  /* ============================= Receive Path ====================================== */
  /* only need to run task if the receive path is not idle */
  if (tpRxSettings.state != TP_RX_STATE_IDLE)
  {
    /* ------------------- TP_RX_STATE_EXPECT_FLOWCONTROL_CONF ----------------------- */
    if (tpRxSettings.state == TP_RX_STATE_EXPECT_FLOWCONTROL_CONF)
    {
      /* update Ar timer which monitors timeout of CAN frame transmission */
      if (tpRxSettings.arTimer > 0)
      {
        /* 1 ms passed, so decrement it */
        tpRxSettings.arTimer--;
        /* did it timeout? */
        if (tpRxSettings.arTimer == 0)
        {
          /* invoke callback to inform about the reception abort */
          if (tpCallbackDataReceived != NULL)
          {
            tpCallbackDataReceived(TpRxBufferGetDataPtr(), 0, TP_RESULT_TIMEOUT_A);
          }
          /* set state back to idle */
          tpRxSettings.state = TP_RX_STATE_IDLE;
        }
      }
    }
    /* ------------------- TP_RX_STATE_EXPECT_CONS_FRAME_IND ------------------------- */
    else if (tpRxSettings.state == TP_RX_STATE_EXPECT_CONS_FRAME_IND)
    {
      /* update Cr timer which monitors timeout of consecutive frame reception */
      if (tpRxSettings.crTimer > 0)
      {
        /* 1 ms passed, so decrement it */
        tpRxSettings.crTimer--;
        /* did it timeout? */
        if (tpRxSettings.crTimer == 0)
        {
          /* invoke callback to inform about the reception abort */
          if (tpCallbackDataReceived != NULL)
          {
            tpCallbackDataReceived(TpRxBufferGetDataPtr(), 0, TP_RESULT_TIMEOUT_CR);
          }
          /* set state back to idle */
          tpRxSettings.state = TP_RX_STATE_IDLE;
        }
      }

    }
  }
} /*** end of TpRxMsTask ***/


/************************************************************************************//**
** \brief     Transmits a Transport Protocol CAN message.
** \param     data Pointer to the array with byte to transmit.
** \return    TRUE if successfully transmitted the CAN message, FALSE otherwise.
**
****************************************************************************************/
static uint8_t TpCanTransmit(uint8_t *data)
{
  /* submit the CAN message transmit request */
  return CanTransmit(tpCommSettings.channel, tpCommSettings.txId, CAN_MAX_DATA_LEN, data);
} /*** end of TpCanTransmit ***/


/************************************************************************************//**
** \brief     CAN I/O module callback function that gets called each time the transport
**            protocol CAN message was received.
** \param     dlc Number of data bytes in the CAN message.
** \param     data Pointer to the array with received data bytes.
** \param     timestamp Reception timestamp of the message (not used).
** \return    none.
**
****************************************************************************************/
static void TpCanReceived(uint8_t dlc, uint8_t *data, uint32_t timestamp)
{
  uint8_t pci_type;
  uint8_t status;
  uint8_t idx;
  uint8_t rxLen;

  /* ============================= Reception Path ==================================== */
  /* copy the newly received PDU data */
  for (idx=0; idx<CAN_MAX_DATA_LEN; idx++)
  {
    tpRxSettings.receivePdu.data[idx] = data[idx];
  }
  /* read out the PCI type of this newly received PDU */
  pci_type = TpGetPciType(&tpRxSettings.receivePdu);

  /* --------------------- TP_RX_STATE_IDLE ------------------------------------------ */
  /* reception of first frame and single frame should always be processed, regardless
   * of the state first check single frame.
   */
  if (pci_type == TP_PCI_SINGLE_FRAME)
  {
    /* if a segmented reception is currently in progress, protocol dictates that we abort
     * this one first.
     */
    if (tpRxSettings.state != TP_RX_STATE_IDLE)
    {
      /* invoke callback to inform about the reception being aborted */
      if (tpCallbackDataReceived != NULL)
      {
        tpCallbackDataReceived(TpRxBufferGetDataPtr(), 0, TP_RESULT_UNEXP_PDU);
      }
      /* go back to idle */
      tpRxSettings.state = TP_RX_STATE_IDLE;
    }
    /* only process if the number of data bytes is not zero */
    if (TpGetSFLen(&tpRxSettings.receivePdu) > 0)
    {
      /* invoke callback to pass the received data on */
      if (tpCallbackDataReceived != NULL)
      {
        tpCallbackDataReceived(&data[1], TpGetSFLen(&tpRxSettings.receivePdu), TP_RESULT_OK);
      }
    }
  }
  /* first frame received? */
  else if (pci_type == TP_PCI_FIRST_FRAME)
  {
    /* if a segmented reception is currently in progress, protocol dictates that we abort
     * this one first.
     */
    if (tpRxSettings.state != TP_RX_STATE_IDLE)
    {
      /* invoke callback to inform about the reception being aborted */
      if (tpCallbackDataReceived != NULL)
      {
        tpCallbackDataReceived(TpRxBufferGetDataPtr(), 0, TP_RESULT_UNEXP_PDU);
      }
      /* go back to idle */
      tpRxSettings.state = TP_RX_STATE_IDLE;
    }
    /* read and store the transfer length */
    tpRxSettings.transferLen = TpGetFFLen(&tpRxSettings.receivePdu);
    /* transfers that could fit in a single frame should be ignored */
    if (tpRxSettings.transferLen > TP_MAX_SINGLE_FRAME_DATA)
    {
      /* new transfer so no bytes yet received */
      tpRxSettings.received = 0;
      /* prepare the flow control PDU with STmin, blocksize */
      TpClearPduData(tpRxSettings.transmitPdu.data);
      TpSetPciType(&tpRxSettings.transmitPdu, TP_PCI_FLOW_CONTROL);
      TpSetFCBlockSize(&tpRxSettings.transmitPdu, tpRxSettings.blockSize);
      TpSetFCStMin(&tpRxSettings.transmitPdu, tpRxSettings.stMin);
      /* check that the reception buffer is large enough for this transfer */
      if (tpRxSettings.transferLen > tpRxSettings.buffer.size)
      {
        /* not enough space in the reception buffer. protocol dictates that
         * we signal this error condition by sending the flow control message
         * with the status set to buffer overflow and then abort the reception.
         */
        TpSetFCStatus(&tpRxSettings.transmitPdu, TP_FS_OVFLW);
        /* submit the flow control PDU for transmission */
        TpCanTransmit(tpRxSettings.transmitPdu.data);
      }
      else
      {
        /* clear the buffer at the start of a new segmented transfer */
        TpRxBufferFlush();
        /* place first data bytes received in the first frame in the buffer already.
         * it was just emptied so this will always work.
         */
        TpRxBufferWrite(&tpRxSettings.receivePdu.data[2], TP_MAX_FIRST_FRAME_DATA);
        tpRxSettings.received += TP_MAX_FIRST_FRAME_DATA;
        /* invoke the first frame received callback to inform that a segmented
         * reception started.
         */
        if (tpCallbackFirstFrameReceived != NULL)
        {
          tpCallbackFirstFrameReceived(tpRxSettings.transferLen);
        }
        /* set the flow status to clear-to-send */
        TpSetFCStatus(&tpRxSettings.transmitPdu, TP_FS_CTS);
        /* submit the flow control PDU for transmission. no action is needed if
         * this fails. the receiver of this message will time out and take action.
         */
        if (TpCanTransmit(tpRxSettings.transmitPdu.data) == TRUE)
        {
          /* first frame is considered sequence number 0 so set this value */
          tpRxSettings.seqNum = 0;
          /* submit in progress, start the timer */
          tpRxSettings.arTimer = TP_AR_TIMER_TIMEOUT_MS;
          /* update the state machine */
          tpRxSettings.state = TP_RX_STATE_EXPECT_FLOWCONTROL_CONF;
        }
      }
    }
  }
  /* only process further reception path related messages is a reception is actually
   * in progress.
   */
  else if (tpRxSettings.state != TP_RX_STATE_IDLE)
  {
    /* ------------------- TP_RX_STATE_EXPECT_CONS_FRAME_IND ------------------------- */
    if (tpRxSettings.state == TP_RX_STATE_EXPECT_CONS_FRAME_IND)
    {
      /* consecutive frame received? */
      if (pci_type == TP_PCI_CONSECUTIVE_FRAME)
      {
        /* stop the reception timeout timer */
        tpRxSettings.crTimer = 0;
        /* increment the sequence number so it contains the value we expect on the PDU */
        tpRxSettings.seqNum++;
        if (tpRxSettings.seqNum > 0x0f)
        {
          tpRxSettings.seqNum = 0;
        }
        /* check if the received sequence number matches */
        if (TpGetCFSeqNum(&tpRxSettings.receivePdu) != tpRxSettings.seqNum)
        {
          /* invoke callback to inform about the reception being aborted */
          if (tpCallbackDataReceived != NULL)
          {
            tpCallbackDataReceived(TpRxBufferGetDataPtr(), 0, TP_RESULT_WRONG_SN);
          }
          /* go back to idle */
          tpRxSettings.state = TP_RX_STATE_IDLE;
        }
        /* sequence number correct, continue with processing it */
        else
        {
          /* assume full length data */
          rxLen = CAN_MAX_DATA_LEN - 1;
          /* correct it in case this is the end of the transfer. in this case the
           * consecutive frame might have more data bytes then we need.
           */
          if ( (tpRxSettings.received + rxLen) > tpRxSettings.transferLen)
          {
            rxLen = tpRxSettings.transferLen - tpRxSettings.received;
          }
          /* place the data in the buffer. note that there is no need to check for
           * available space, because this check was done when the reception started,
           * while processing the first frame message.
           */
          TpRxBufferWrite(&tpRxSettings.receivePdu.data[1], rxLen);
          tpRxSettings.received += rxLen;
          /* is this the end of the transfer? */
          if (tpRxSettings.received == tpRxSettings.transferLen)
          {
            /* pass on whatever data is still in the buffer and signal the end of the transfer */
            if (tpCallbackDataReceived != NULL)
            {
              tpCallbackDataReceived(TpRxBufferGetDataPtr(), TpRxBufferScan(), TP_RESULT_OK);
            }
            /* transfer done so go back to idle state */
            tpRxSettings.state = TP_RX_STATE_IDLE;
          }
          /* still expecting to receive more data */
          else
          {
            /* increment the consecutive frame counter */
            tpRxSettings.cfCount++;
            /* full block received? */
            if (tpRxSettings.cfCount == tpRxSettings.blockSize)
            {
              /* prepare the flow control PDU with STmin, blocksize and clear-to-send */
              TpClearPduData(tpRxSettings.transmitPdu.data);
              TpSetPciType(&tpRxSettings.transmitPdu, TP_PCI_FLOW_CONTROL);
              TpSetFCStatus(&tpRxSettings.transmitPdu, TP_FS_CTS);
              TpSetFCBlockSize(&tpRxSettings.transmitPdu, tpRxSettings.blockSize);
              TpSetFCStMin(&tpRxSettings.transmitPdu, tpRxSettings.stMin);
              /* submit the flow control PDU for transmission */
              if (TpCanTransmit(tpRxSettings.transmitPdu.data) == TRUE)
              {
                /* submit in progress, start the timer */
                tpRxSettings.arTimer = TP_AR_TIMER_TIMEOUT_MS;
                /* update the state machine */
                tpRxSettings.state = TP_RX_STATE_EXPECT_FLOWCONTROL_CONF;
              }
              else
              {
                /* error sending flow status so abort */
                if (tpCallbackDataReceived != NULL)
                {
                  tpCallbackDataReceived(TpRxBufferGetDataPtr(), 0, TP_RESULT_ERROR);
                }
                /* transfer aborted so go back to idle state */
                tpRxSettings.state = TP_RX_STATE_IDLE;
              }
            }
            /* expecting the next consecutive frame */
            else
            {
              /* start the reception timeout timer */
              tpRxSettings.crTimer = TP_CR_TIMER_TIMEOUT_MS;
            }
          }
        }
      }
    }
  }

  /* ============================= Transmit Path ===================================== */
  /* only need to process this indication if the transmit path is not idle */
  else if (tpTxSettings.state != TP_TX_STATE_IDLE)
  {
    /* copy the newly received PDU data */
    for (idx=0; idx<CAN_MAX_DATA_LEN; idx++)
    {
      tpTxSettings.receivePdu.data[idx] = data[idx];
    }
    /* read out the PCI type of this newly received PDU */
    pci_type = TpGetPciType(&tpTxSettings.receivePdu);

    /* ------------------- TP_TX_STATE_EXPECT_FLOWCONTROL_IND ----------------------- */
    if (tpTxSettings.state == TP_TX_STATE_EXPECT_FLOWCONTROL_IND)
    {
      /* process flow control indication */
      if (pci_type == TP_PCI_FLOW_CONTROL)
      {
        /* read out the status information */
        status = TpGetFCStatus(&tpTxSettings.receivePdu);
        /* overflow reported? */
        if (status == TP_FS_OVFLW)
        {
          /* abort the transfer */
          tpTxSettings.state = TP_TX_STATE_IDLE;
          /* invoke callback to pass on this transfer complete (with error) event */
          if (tpCallbackTransferComplete != NULL)
          {
            tpCallbackTransferComplete(TP_RESULT_BUFFER_OVFLW);
          }
        }
        /* wait requested? */
        else if (status == TP_FS_WAIT)
        {
          /* if a wait is requested, we need to not send out the next consecutive frame.
           * instead we need to wait for the next flow control and continue if this one
           * contains the clear to send (CTS) specifier. we are currently already in a
           * state for expecting a flow control. we just need to reset the Bs timer
           * that monitors the timeout of a flow control reception.
           */
          tpTxSettings.bsTimer = TP_BS_TIMER_TIMEOUT_MS;
        }
        /* clear to send? */
        else if (status == TP_FS_CTS)
        {
          /* store the received block size and STmin values */
          tpTxSettings.blockSize = TpGetFCBlockSize(&tpTxSettings.receivePdu);
          tpTxSettings.stMin = TpGetFCStMin(&tpTxSettings.receivePdu);
          /* further validate the STmin time */
          tpTxSettings.stMin = TpCheckAndCorrectStMin(tpTxSettings.stMin);
          /* reset consecutive frame counter, needed for block size handling */
          tpTxSettings.cfCount = 0;
          /* restart the separation timer to transmit the next consecutive frame
           * after STmin. Going by the specification, this STmin wait time here
           * is not necessary, but some services tools seem to expect this.
           * It is within the specification to do this extra STmin time here.
           * Some service tools seem to complain if we send at exactly the STmin
           * time so add one more millisecond.
           */
          tpTxSettings.cfSepTimer = tpTxSettings.stMin + 1;
          /* transition to the clear to send state, which will send out the
           * data in consecutive frames at task level.
           */
          tpTxSettings.state = TP_TX_STATE_CLEAR_TO_SEND;
        }
        /* invalid and unsupported flowstatus received */
        else
        {
          /* abort the transfer */
          tpTxSettings.state = TP_TX_STATE_IDLE;
          /* invoke callback to pass on this transfer complete (with error) event */
          if (tpCallbackTransferComplete != NULL)
          {
            tpCallbackTransferComplete(TP_RESULT_INVALID_FS);
          }
        }
      }
    }
  }
} /*** end of TpCanReceived ***/


/************************************************************************************//**
** \brief     CAN I/O module callback function that gets called each time the transport
**            protocol CAN message was successfully transmitted.
** \return    none.
**
****************************************************************************************/
static void TpCanTransmitted(void)
{
  uint8_t pci_type;

  /* ============================= Receive Path ====================================== */
  /* read out the PCI type of the last PDU that was submitted for transmission */
  pci_type = TpGetPciType(&tpRxSettings.transmitPdu);

  /* only need to process this confirmation if the receive path is not idle */
  if (tpRxSettings.state != TP_RX_STATE_IDLE)
  {
    /* ------------------- TP_RX_STATE_EXPECT_FLOWCONTROL_CONF ----------------------- */
    if (tpRxSettings.state == TP_RX_STATE_EXPECT_FLOWCONTROL_CONF)
    {
      /* reset the consecutive frame count because a new block of consecutive frames
       * are now expected.
       */
      tpRxSettings.cfCount = 0;
      /* stop the timeout monitoring */
      tpRxSettings.arTimer = 0;
      /* start the reception timeout timer */
      tpRxSettings.crTimer = TP_CR_TIMER_TIMEOUT_MS;
      /* update the state */
      tpRxSettings.state = TP_RX_STATE_EXPECT_CONS_FRAME_IND;
    }
  }

  /* ============================= Transmit Path ===================================== */
  /* read out the PCI type of the last PDU that was submitted for transmission */
  pci_type = TpGetPciType(&tpTxSettings.transmitPdu);

  /* only need to process this confirmation if the transmit path is not idle */
  if (tpTxSettings.state != TP_TX_STATE_IDLE)
  {
    /* ------------------- TP_TX_STATE_EXPECT_SINGLEFRAME_CONF ----------------------- */
    if (tpTxSettings.state == TP_TX_STATE_EXPECT_SINGLEFRAME_CONF)
    {
      /* process single frame confirmation */
      if (pci_type == TP_PCI_SINGLE_FRAME)
      {
        /* stop the timer */
        tpTxSettings.asTimer = 0;
        /* transfer complete, so transition back to the idle state */
        tpTxSettings.state = TP_TX_STATE_IDLE;
        /* invoke callback to pass on this transfer complete event */
        if (tpCallbackTransferComplete != NULL)
        {
          tpCallbackTransferComplete(TP_RESULT_OK);
        }
      }
    }
    /* ------------------- TP_TX_STATE_EXPECT_FIRSTFRAME_CONF ------------------------ */
    else if (tpTxSettings.state == TP_TX_STATE_EXPECT_FIRSTFRAME_CONF)
    {
      /* process first frame confirmation */
      if (pci_type == TP_PCI_FIRST_FRAME)
      {
        /* stop the timer */
        tpTxSettings.asTimer = 0;
        /* update transfer information. first frame carried 6 data bytes */
        tpTxSettings.transmitted += 6;
        /* first frame is considered sequence number 0 so set this value */
        tpTxSettings.seqNum = 0;
        /* next we expect a flow control reception with blocksize and st_min info */
        tpTxSettings.state = TP_TX_STATE_EXPECT_FLOWCONTROL_IND;
        /* start timeout monitoring of the flow control */
        tpTxSettings.bsTimer = TP_BS_TIMER_TIMEOUT_MS;
      }
    }
    /* ------------------- TP_TX_STATE_CLEAR_TO_SEND --------------------------------- */
    else if (tpTxSettings.state == TP_TX_STATE_CLEAR_TO_SEND)
    {
      /* process consecutive frame confirmation */
      if (pci_type == TP_PCI_CONSECUTIVE_FRAME)
      {
        /* stop the timer */
        tpTxSettings.asTimer = 0;
        /* restart the STmin timer. Some service tools seem to complain if we send at
         * exactly the STmin time so add one more millisecond.
         */
        tpTxSettings.cfSepTimer = tpTxSettings.stMin + 1;
        /* update the number of bytes that were transmitted, assuming the max that fits
         * in a consecutive frame
         */
        tpTxSettings.transmitted += (CAN_MAX_DATA_LEN - 1);
        /* check if all data is now transferred. note that transmitted can be a few
         * byte more the transferLen because the last consecutive frame might not have
         * been filled with data bytes.
         */
        if (tpTxSettings.transmitted >= tpTxSettings.transferLen)
        {
          /* correct transmitted count. currently not use anywhere, just for
           * correctness and possible later usage.
           */
          tpTxSettings.transmitted = tpTxSettings.transferLen;
          /* set state back to idle */
          tpTxSettings.state = TP_TX_STATE_IDLE;
          /* invoke callback to inform about the transfer completion */
          if (tpCallbackTransferComplete != NULL)
          {
            tpCallbackTransferComplete(TP_RESULT_OK);
          }
        }
        /* still data left to send */
        else
        {
          /* increment the consecutive frame counter */
          tpTxSettings.cfCount++;
          /* block of consecutive frames now sent? note that a blockSize of 0 means
           * that consecutive frames can be sent without having to wait for a
           * flow control.
           */
          if ( (tpTxSettings.cfCount >= tpTxSettings.blockSize) && (tpTxSettings.blockSize > 0) )
          {
            /* a flow control is now expected. update the state for this */
            tpTxSettings.state = TP_TX_STATE_EXPECT_FLOWCONTROL_IND;
            /* start timeout monitoring of the flow control */
            tpTxSettings.bsTimer = TP_BS_TIMER_TIMEOUT_MS;
          }
        }
      }

    }
  }
} /*** end of TpCanTransmitted ***/


/************************************************************************************//**
** \brief     Utility function for setting all bytes of a packet data unit (PDU) to 0.
** \param     data Pointer to the PDU data bytes.
** \return    none.
**
****************************************************************************************/
static void TpClearPduData(uint8_t *data)
{
  uint8_t idx;

  /* clear all data bytes */
  for (idx=0; idx<CAN_MAX_DATA_LEN; idx++)
  {
    data[idx] = 0;
  }
} /*** end of TpClearPduData ***/


/************************************************************************************//**
** \brief     Utility function for setting the PCI type in the packet data unit (PDU).
** \param     pdu Pointer to the transmit PDU.
** \param     type PCI type.
** \return    none.
**
****************************************************************************************/
static void TpSetPciType(tTpPDU *pdu, uint8_t type)
{
  pdu->data[0] &= ~0xf0;
  pdu->data[0] |= (uint8_t)(type << 4);
} /*** end of TpSetPciType ***/


/************************************************************************************//**
** \brief     Utility function for reading the PCI type from a packet data unit (PDU).
** \param     pdu Pointer to the transmit PDU.
** \return    The PCI type code (TP_PCI_Xxx).
**
****************************************************************************************/
static uint8_t TpGetPciType(tTpPDU *pdu)
{
  return (pdu->data[0] >> 4) & 0x0f;
} /*** end of TpGetPciType ***/


/************************************************************************************//**
** \brief     Utility function for setting the PCI length in a single frame packet data
**            unit (PDU).
** \param     pdu Pointer to the transmit PDU.
** \param     len Number of data bytes (1..7).
** \return    none.
**
****************************************************************************************/
static void TpSetSFLen(tTpPDU *pdu, uint8_t len)
{
  pdu->data[0] &= ~0x0f;
  pdu->data[0] |= (uint8_t)(len & 0x0f);
} /*** end of TpSetSFLen ***/


/************************************************************************************//**
** \brief     Utility function for getting the PCI length from a single frame packet data
**            unit (PDU).
** \param     pdu Pointer to the transmit PDU.
** \return    none.
**
****************************************************************************************/
static uint8_t TpGetSFLen(tTpPDU *pdu)
{
  return (pdu->data[0] & 0x0f);
} /*** end of TpSetSFLen ***/


/************************************************************************************//**
** \brief     Utility function for setting the PCI length in a first frame packet data
**            unit (PDU), taking into account the correct bit ordering.
** \param     pdu Pointer to the transmit PDU.
** \param     len Number of data bytes (1..TP_MAX_TRANSFER_LEN).
** \return    none.
**
****************************************************************************************/
static void TpSetFFLen(tTpPDU *pdu, uint16_t len)
{
  pdu->data[0] &= ~0x0f;
  pdu->data[0] |= (uint8_t)((uint16_t)len >> 8);
  pdu->data[1] |= (uint8_t)len;
} /*** end of TpSetFFLen ***/


/************************************************************************************//**
** \brief     Utility function for getting the PCI length from a first frame packet data
**            unit (PDU), taking into account the correct bit ordering.
** \param     pdu Pointer to the transmit PDU.
** \return    Number of data bytes (1..TP_MAX_TRANSFER_LEN).
**
****************************************************************************************/
static uint16_t TpGetFFLen(tTpPDU *pdu)
{
  return (((uint16_t)(pdu->data[0] << 8) & 0x0f00) | pdu->data[1]);
} /*** end of TpGetFFLen ***/


/************************************************************************************//**
** \brief     Utility function for setting the sequence number in a consecutive frame
**            packet data unit (PDU).
** \param     pdu Pointer to the transmit PDU.
** \param     sn Sequence number (0x00..0x0F).
** \return    none.
**
****************************************************************************************/
static void TpSetCFSeqNum(tTpPDU *pdu, uint8_t sn)
{
  pdu->data[0] &= ~0x0f;
  pdu->data[0] |= (uint8_t)(sn & 0x0f);
} /*** end of TpSetCFSeqNum ***/


/************************************************************************************//**
** \brief     Utility function for getting the sequence number from a consecutive frame
**            packet data unit (PDU).
** \param     pdu Pointer to the transmit PDU.
** \return    sn Sequence number (0x00..0x0F).
**
****************************************************************************************/
static uint8_t TpGetCFSeqNum(tTpPDU *pdu)
{
  return (pdu->data[0] & 0x0f);
} /*** end of TpGetCFSeqNum ***/


/************************************************************************************//**
** \brief     Utility function for setting the status in a flow control packet data unit
**            (PDU).
** \param     pdu Pointer to the transmit PDU.
** \param     status flow control status.
** \return    none.
**
****************************************************************************************/
static void TpSetFCStatus(tTpPDU *pdu, uint8_t status)
{
  pdu->data[0] &= ~0x0f;
  pdu->data[0] |= (uint8_t)(status & 0x0f);
} /*** end of TpSetFCStatus ***/


/************************************************************************************//**
** \brief     Utility function for reading the status from a flow control packet data
**            unit(PDU).
** \param     pdu Pointer to the transmit PDU.
** \return    flow control status.
**
****************************************************************************************/
static uint8_t TpGetFCStatus(tTpPDU *pdu)
{
  return (pdu->data[0]) & 0x0f;
} /*** end of TpGetFCStatus ***/


/************************************************************************************//**
** \brief     Utility function for setting the blocksize in a flow control packet data
**            unit (PDU).
** \param     pdu Pointer to the transmit PDU.
** \param     bs block size.
** \return    none.
**
****************************************************************************************/
static void TpSetFCBlockSize(tTpPDU *pdu, uint8_t bs)
{
  pdu->data[1] = bs;
} /*** end of TpSetFCBlockSize ***/


/************************************************************************************//**
** \brief     Utility function for reading the blocksize from a flow control packet data
**            unit (PDU).
** \param     pdu Pointer to the transmit PDU.
** \return    block size.
**
****************************************************************************************/
static uint8_t TpGetFCBlockSize(tTpPDU *pdu)
{
  return pdu->data[1];
} /*** end of TpGetFCBlockSize ***/


/************************************************************************************//**
** \brief     Utility function for setting the STmin in a flow control packet data unit
**            (PDU).
** \param     pdu Pointer to the transmit PDU.
** \param     st_min Minimal separation time (ms).
** \return    none.
**
****************************************************************************************/
static void TpSetFCStMin(tTpPDU *pdu, uint8_t st_min)
{
  pdu->data[2] = st_min;
} /*** end of TpSetFCStMin ***/


/************************************************************************************//**
** \brief     Utility function for reading the STmin in a flow control packet data unit
**            (PDU).
** \param     pdu Pointer to the transmit PDU.
** \return    Minimal separation time (ms).
**
****************************************************************************************/
static uint8_t TpGetFCStMin(tTpPDU *pdu)
{
  return pdu->data[2];
} /*** end of TpGetFCStMin ***/


/************************************************************************************//**
** \brief     The STmin time has some reserved ranges. This function checks if a valid
**            in the reserved range is used and then defaults to the worst case STmin
**            time as an automatic error correction.
** \param     stMin Uncorrected STmin time value.
** \return    Corrected STmin time value.
**
****************************************************************************************/
static uint8_t TpCheckAndCorrectStMin(uint8_t stMin)
{
  uint8_t stMinCorrected = stMin;

  if ( (tpTxSettings.stMin >= 0xF1) && (tpTxSettings.stMin <= 0xF9) )
  {
    /* this module can't handle STmin in the 100 - 900 microsecond range,
     * so just use 1 ms.
     */
    stMinCorrected = 1;
  }
  else if ( (tpTxSettings.stMin >= 0x80) && (tpTxSettings.stMin <= 0xF0) )
  {
    /* reserved range. protocol specifies that 0x7F = 127 ms is used */
    stMinCorrected = 127;
  }
  else if ( (tpTxSettings.stMin >= 0xFA) && (tpTxSettings.stMin <= 0xFF) )
  {
    /* reserved range. protocol specifies that 0x7F = 127 ms is used */
    stMinCorrected = 127;
  }
  return stMinCorrected;
} /*** end of TpCheckAndCorrectStMin ***/


/************************************************************************************//**
** \brief     Initializes the buffer used for data reception.
** \param     bufferSize Total number of storage bytes for the buffer.
** \return    none.
**
****************************************************************************************/
static void TpRxBufferInit(uint16_t bufferSize)
{
  /* allocate memory for the reception buffer on the heap. this buffer is used for
   * segmented data reception and should therefore be at least larger than the amount of
   * bytes that fit in a single frame.
   */
  tpRxSettings.buffer.size = bufferSize;
  if (tpRxSettings.buffer.size <= TP_MAX_SINGLE_FRAME_DATA)
  {
    tpRxSettings.buffer.size = TP_MAX_SINGLE_FRAME_DATA + 1;
  }
  /* no need to reserve more buffer space than what is supported by the protocol */
  if (tpRxSettings.buffer.size > TP_MAX_TRANSFER_LEN)
  {
    tpRxSettings.buffer.size = TP_MAX_TRANSFER_LEN;
  }
  tpRxSettings.buffer.data = pvPortMalloc(tpRxSettings.buffer.size);
  /* make sure the allocation was successful */
  if (tpRxSettings.buffer.data == NULL)
  {
    ErrCodesSetError(ER_CODE_TP_RX_ALLOC_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* make sure the buffer is empty */
  tpRxSettings.buffer.entries = 0;
} /*** end of TpRxBufferInit ***/


/************************************************************************************//**
** \brief     Empties out the buffer by setting the read and write pointer to the
**            start.
** \return    none.
**
****************************************************************************************/
static void TpRxBufferFlush(void)
{
  uint32_t saved_cs_state;

  saved_cs_state = OsEnterCriticalSection();
  tpRxSettings.buffer.entries = 0;
  OsLeaveCriticalSection(saved_cs_state);
} /*** end of TpRxBufferFlush ***/


/************************************************************************************//**
** \brief     Determines how many bytes are currently stored in the data buffer.
** \return    Number of bytes in the buffer.
**
****************************************************************************************/
static uint16_t TpRxBufferScan(void)
{
  uint16_t entries;
  uint32_t saved_cs_state;

  saved_cs_state = OsEnterCriticalSection();
  entries = tpRxSettings.buffer.entries;
  OsLeaveCriticalSection(saved_cs_state);
  return entries;
} /*** end of TpRxBufferScan ***/


/************************************************************************************//**
** \brief     Obtains the location of the start of data pointer.
** \return    Location of the data pointer.
**
****************************************************************************************/
static uint8_t *TpRxBufferGetDataPtr(void)
{
  uint8_t *dataPtr;
  uint32_t saved_cs_state;

  saved_cs_state = OsEnterCriticalSection();
  dataPtr = tpRxSettings.buffer.data;
  OsLeaveCriticalSection(saved_cs_state);
  return dataPtr;
} /*** end of TpRxBufferGetDataPtr ***/


/************************************************************************************//**
** \brief     Writes data to the buffer if it still fits. If not, then nothing is
**            written to the buffer.
** \param     data Pointer to byte array with data to place in the buffer.
** \param     len Number of bytes to copy to the buffer.
** \return    TRUE if the data fit in the buffer, FALSE otherwise.
**
****************************************************************************************/
static uint8_t TpRxBufferWrite(uint8_t *data, uint16_t len)
{
  uint8_t result = FALSE;
  uint16_t cnt;

  uint32_t saved_cs_state;

  saved_cs_state = OsEnterCriticalSection();
  /* only write to the buffer if the data actually fits  */
  if (len <= (tpRxSettings.buffer.size - tpRxSettings.buffer.entries))
  {
    /* write byte for byte */
    for (cnt=0; cnt<len; cnt++)
    {
      /* store byte in buffer */
      tpRxSettings.buffer.data[tpRxSettings.buffer.entries] = data[cnt];
      tpRxSettings.buffer.entries++;
    }
    result = TRUE;
  }
  OsLeaveCriticalSection(saved_cs_state);
  return result;
} /*** end of TpRxBufferWrite ***/


/*********************************** end of tp.c ***************************************/
