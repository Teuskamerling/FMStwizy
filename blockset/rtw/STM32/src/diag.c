/************************************************************************************//**
* \file         diag.c
* \brief        Diagnostics module source file. 
* \details      This module implements basic UDS support according to ISO 14229:2006.
*               The following services are supported:
*                 - DiagnosticSessionControl (10h)
*                 - ECUReset (11h)
*                 - SecurityAccess(27h)
*                 - TesterPresent (3Eh)
*                 - ReadMemoryByAddress (23h)
*                 - WriteMemoryByAddress (3Dh)
*                 - ClearDiagnosticInformation (14h)
*                 - ReadDTCInformation (19h)
*               This module depends on the ISO 15765-2 transort protocol module. The
*               DTC access part is integrated with the HANcoder error code module.
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
#include "diag.h"                                     /* for diagnostics module        */
#include "tp.h"                                       /* for transport protocol        */
#include "os.h"                                       /* for operating system          */
#include "errorcodes.h"                               /* error codes module            */
#include "errorList.h"                                /* for error list                */
#include <string.h>                                   /* for memcpy                    */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Timeout time of the session (S3) timer in milliseconds. */
#define DIAG_S3_TIMER_TIMEOUT_MS                 (5000u)

/** \brief Bitmask for the suppress positive response bit. */
#define DIAG_BITMASK_SUPPRESS_POS_RES            (0x80)

/** \brief Diagnostics service negative response ID. */
#define DIAG_SERV_RES_ID_NEGATIVE                (0x7Fu)

/** \brief DiagnosticSessionControl service request ID. */
#define DIAG_SERV_REQ_ID_SESSION_CONTROL         (0x10u)

/** \brief ECUReset service request ID. */
#define DIAG_SERV_REQ_ID_ECU_RESET               (0x11u)

/** \brief ClearDiagnosticInformation service request ID. */
#define DIAG_SERV_REQ_ID_CLEAR_DIAG_INFO         (0x14u)

/** \brief ReadDTCInformation service request ID. */
#define DIAG_SERV_REQ_ID_READ_DTC_INFO           (0x19u)

/** \brief ReadMemoryByAddress service request ID. */
#define DIAG_SERV_REQ_ID_READ_MEM_BY_ADDR        (0x23u)

/** \brief SecurityAccess service request ID. */
#define DIAG_SERV_REQ_ID_SECURITY_ACCESS         (0x27u)

/** \brief WriteMemoryByAddress service request ID. */
#define DIAG_SERV_REQ_ID_WRITE_MEM_BY_ADDR       (0x3Du)

/** \brief TesterPresent service request ID. */
#define DIAG_SERV_REQ_ID_TESTER_PRESENT          (0x3Eu)

/** \brief DiagnosticSessionControl service positive response ID. */
#define DIAG_SERV_POS_RES_ID_SESSION_CONTROL     (0x50u)

/** \brief ECUReset service request ID. */
#define DIAG_SERV_POS_RES_ID_ECU_RESET           (0x51u)

/** \brief ClearDiagnosticInformation service positive response ID. */
#define DIAG_SERV_POS_RES_ID_CLEAR_DIAG_INFO     (0x54u)

/** \brief ReadDTCInformation service positive response ID. */
#define DIAG_SERV_POS_RES_ID_READ_DTC_INFO       (0x59u)

/** \brief ReadMemoryByAddress service positive response ID. */
#define DIAG_SERV_POS_RES_ID_READ_MEM_BY_ADDR    (0x63u)

/** \brief SecurityAccess service positive response ID. */
#define DIAG_SERV_POS_RES_ID_SECURITY_ACCESS     (0x67u)

/** \brief WriteMemoryByAddress service positive response ID. */
#define DIAG_SERV_POS_RES_ID_WRITE_MEM_BY_ADDR   (0x7Du)

/** \brief TesterPresent service positive response ID. */
#define DIAG_SERV_POS_RES_ID_TESTER_PRESENT      (0x7Eu)

/** \brief Negative response code for service not supported (SNS). */
#define DIAG_SERV_NRC_SERVICE_NOT_SUPPORTED      (0x11u)

/** \brief Negative response code for subfunction not supported (SFNS). */
#define DIAG_SERV_NRC_SUBFUNCTION_NOT_SUPPORTED  (0x12u)

/** \brief Negative response code for incorrect message length or invalid format
 *        (IMLOIF)
 */
#define DIAG_SERV_NRC_INVALID_FORMAT             (0x13u)

/** \brief Negative response code for a conditions not correct error. */
#define DIAG_SERV_NRC_INCORRECT_CONDITIONS       (0x22u)

/** \brief Negative response code for a sequence error. */
#define DIAG_SERV_NRC_SEQUENCE_ERROR             (0x24u)

/** \brief Negative response code for security access denied. */
#define DIAG_SERV_NRC_ACCESS_DENIED              (0x33u)

/** \brief Negative response code for an invalid key. */
#define DIAG_SERV_NRC_INVALID_KEY                (0x35u)

/** \brief Negative response code for request out of range (ROOR). */
#define DIAG_SERV_NRC_REQUEST_OUT_OF_RANGE       (0x31u)



/****************************************************************************************
* Type definitions
****************************************************************************************/
typedef struct
{
  uint16_t size;                       /**< total number of bytes that can fit         */
  uint16_t entries;                    /**< number of bytes stored in the buffer       */
  uint8_t *data;                       /**< pointer to the start of the buffer         */
} tDiagTxBuffer;

/** \brief Structure type for storing information related to the diagnostic server. */
typedef struct
{
  /** \brief Currently active diagnostic session. */
  tDiagSession session;
  /** \brief Boolean flag to determine if certain services, such as Read/WriteMemoryBy-
   *         address, are locked (extendedDiagnosticSession) or unlocked (defaultSession).
   */
  uint8_t requireSeedKey;
  /** \brief Flag to indicate if a seed/key sequence unlocked the resources related to
   *         the currently active session.
   */
  uint8_t sessionResourcesUnlocked;
  /** \brief Buffer object needed for transmitting responses using the transport
   *         protocol.
   */
  tDiagTxBuffer txBuffer;

  /** \brief Flag to determine if a positive response should be send or not. */
  uint8_t suppressPosRspMsg;
  /** \brief Session (S3) timer counter. A value of 0 means that the timer is disabled,
   *         a decrement to 0 means the timer timed out.
   */
  uint16_t s3_timer;
  /** \brief Flag to determine if an ECU reset should be performed upon completion of
   *         sending the positive response to the ECU reset service.
   */
  uint8_t ecuResetPending;
} tDiagServerInfo;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void     DiagMsTask(void);
static void     DiagMillisecondTimerCallback(xTimerHandle pxTimer);
static void     DiagSetSession(tDiagSession newSession);
static void     DiagHandleServiceRequest(uint8_t *data, uint16_t len);
/* transport protocol interface */
static uint8_t  DiagTpTransmitResponse(void);
static void     DiagTpFirstFrameReceived(uint16_t len);
static void     DiagTpDataReceived(uint8_t *data, uint16_t len, tTpResult result);
static void     DiagTpTransferComplete(tTpResult result);
/* session timer */
static void     DiagSessionTimerStart(void);
static void     DiagSessionTimerStop(void);
static void     DiagSessionTimerUpdate(void);
/* transmit buffer management */
static void     DiagTxBufferInit(uint16_t bufferSize);
static void     DiagTxBufferFlush(void);
static uint16_t DiagTxBufferScan(void);
static uint8_t *DiagTxBufferGetDataPtr(void);
static void     DiagTxBufferWrite(uint8_t *data, uint16_t len);
/* diagnostic service processors */
static void     DiagPrepareNegativeResponse(uint8_t serviceId, uint8_t responseCode);
static void     DiagProcessTesterPresent(uint8_t *data, uint16_t len);
static void     DiagProcessEcuReset(uint8_t *data, uint16_t len);
static void     DiagProcessSecurityAccess(uint8_t *data, uint16_t len);
static void     DiagProcessSessionControl(uint8_t *data, uint16_t len);
static void     DiagProcessReadMemoryByAddress(uint8_t *data, uint16_t len);
static void     DiagProcessWriteMemoryByAddress(uint8_t *data, uint16_t len);
static void     DiagProcessClearDiagInfo(uint8_t *data, uint16_t len);
static void     DiagProcessReadDTCInfo(uint8_t *data, uint16_t len);


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Holds group of variables related to the diagnostics server.  */
static tDiagServerInfo diagServerInfo;

/** \brief Timer handle for the 1 millisecond task timer. */
static xTimerHandle diagTimer1msHandle;

/** \brief Function pointer for the get seed callback handler. */
static tDiagCallbackGetSeed diagCallbackGetSeed;

/** \brief Function pointer for the verify key callback handler. */
static tDiagCallbackVerifyKey diagCallbackVerifyKey;

/** \brief Seed value to be used in the seed/key unlock sequence. */
static uint32_t diagSeed;

/** \brief Key value that was most recently received from the UDS tester during a
 *         seed/key unlock sequence.
 */
static uint32_t diagKey;


/************************************************************************************//**
** \brief     Initializes the diagnostics module. Certain supported services, such as
**            Read/WriteMemoryByAddress are allowed to be available by the defaultSession,
**            but the application can decide to place them in the extendedDiagnostic-
**            Session. In this latter case, access to these services must first be 
**            unlocked by a seed/key sequence.
** \attention This module relies on the transport protocol module for communication. 
**            Make sure the TpInit() is called prior to calling this function.
** \param     require_seedkey TRUE to lock services for the extendedDiagnosticSession,
**            FALSE to keep them unlocked and make them available in the defaultSession.
** \param     txBufferSize Size of the internal transmit buffer in bytes.
** \return    none.
**
****************************************************************************************/
void DiagInit(uint8_t require_seedkey, uint16_t txBufferSize)
{
  portBASE_TYPE result;

  /* set the default diagnostics session and lock the resources */
  diagServerInfo.session = DIAG_SESSION_DEFAULT;
  diagServerInfo.sessionResourcesUnlocked = FALSE;
  /* set the default seed to make sure it is not 0, because this means resources
   * unlocked.
   */
  diagSeed = 0x12345678;
  /* reset the key */
  diagKey = 0;
  /* store resource locked information */
  diagServerInfo.requireSeedKey = require_seedkey;
  /* disable session timer */
  diagServerInfo.s3_timer = 0;
  /* by default don't suppress the response message */
  diagServerInfo.suppressPosRspMsg = FALSE;
  /* negate the ECU reset pending flag */
  diagServerInfo.ecuResetPending = FALSE;
  /* reset callback function pointers */
  diagCallbackGetSeed = NULL;
  diagCallbackVerifyKey = NULL;
  /* initialize the transmit buffer */
  DiagTxBufferInit(txBufferSize);
  /* create the 1 millisecond task timer */
  diagTimer1msHandle = xTimerCreate("diagTimer", (((portTickType)1000)/portTICK_PERIOD_US),
                                    pdTRUE, NULL, DiagMillisecondTimerCallback);
  if (diagTimer1msHandle == NULL)
  {
    ErrCodesSetError(ER_CODE_DIAG_TASK_TIMER_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* register callback functions with the transport protocol module */
  TpRegisterFirstFrameReceivedCallback(DiagTpFirstFrameReceived);
  TpRegisterDataReceivedCallback(DiagTpDataReceived);
  TpRegisterTransferCompleteCallback(DiagTpTransferComplete);
  /* start the 1 millisecond task timer. note that block time must be 0 since it could
   * be that the scheduler hasn't been started yet.
   */
  result = xTimerStart(diagTimer1msHandle, 0);
  if (!(result == pdPASS))
  {
    ErrCodesSetError(ER_CODE_DIAG_TASK_TIMER_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
} /*** end of DiagInit ***/


/************************************************************************************//**
** \brief     Sets the value of the 32-bit seed that is to be used in the seed/key unlock
**            sequence. This function is typically called in the callback function that
**            was registered with the DiagRegisterGetSeedCallback() function.
** \param     seed Seed value to be used.
** \return    none.
**
****************************************************************************************/
void DiagSetSeed(uint32_t seed)
{
  diagSeed = seed;
} /*** end of DiagSetSeed ***/


/************************************************************************************//**
** \brief     Reads the value of the 32-bit seed that is to used in the seed/key unlock
**            sequence. This function is typically called in the callback function that
**            was registered with the DiagRegisterVerifyKeyCallback() function to help
**            determine if the received key was valid.
** \return    Seed value.
**
****************************************************************************************/
uint32_t DiagGetSeed(void)
{
  return diagSeed;
} /*** end of DiagGetSeed ***/


/************************************************************************************//**
** \brief     Reads the value of the 32-bit key that was recently received from the UDS
**            tester. Typically called in the callback function that was registered with
**            the DiagRegisterVerifyKeyCallback() function to be able to determine if
**            the received key was valid.
** \return    Key value.
**
****************************************************************************************/
uint32_t DiagGetKey(void)
{
  return diagKey;
} /*** end of DiagGetKey ***/


/************************************************************************************//**
** \brief     Determines is the key was valid or not. Typically called in the callback
**            function that was registered with the DiagRegisterVerifyKeyCallback()
**            function to be able to determine if the received key was valid.
** \return    > 0 if the key was valid, 0 otherwise.
**
****************************************************************************************/
void DiagSetKeyVerified(uint8_t keyOkay)
{
  if (keyOkay > 0)
  {
    /* unlock resources */
    diagServerInfo.sessionResourcesUnlocked = TRUE;
  }
  else
  {
    /* lock resources */
    diagServerInfo.sessionResourcesUnlocked = FALSE;
  }
} /*** end of DiagSetKeyVerified ***/


/************************************************************************************//**
** \brief     Registration of the callback function that gets called each time a seed is
**            needed for the extendedDiagnosticSession unlock sequence.
** \param     callbackPtr Pointer to the callback function.
** \return    none.
**
****************************************************************************************/
void DiagRegisterGetSeedCallback(tDiagCallbackGetSeed callbackPtr)
{
  diagCallbackGetSeed = callbackPtr;
} /*** end of DiagRegisterGetSeedCallback ***/


/************************************************************************************//**
** \brief     Registration of the callback function that gets called each time a key need
**            to be verified for the extendedDiagnosticSession unlock sequence.
** \param     callbackPtr Pointer to the callback function.
** \return    none.
**
****************************************************************************************/
void DiagRegisterVerifyKeyCallback(tDiagCallbackVerifyKey callbackPtr)
{
  diagCallbackVerifyKey = callbackPtr;
} /*** end of DiagRegisterVerifyKeyCallback ***/


/************************************************************************************//**
** \brief     Task function that drives time dependent parts of the diagnostics module.
**            This function should be called every 1 millisecond.
** \return    none.
**
****************************************************************************************/
static void DiagMsTask(void)
{
  /* update the session timer */
  DiagSessionTimerUpdate();
} /*** end of DiagMsTask ***/


/************************************************************************************//**
** \brief     Timer callback function that should be called once a milliseconds. This
**            function drives the time dependent parts of the diagnostics module.
** \param     pxTimer Handle of the timer that caused the callback to be called.
** \return    none.
**
****************************************************************************************/
static void DiagMillisecondTimerCallback(xTimerHandle pxTimer)
{
  if (pxTimer == diagTimer1msHandle)
  {
    /* call the diagnostics module task function */
    DiagMsTask();
  }
} /*** end of DiagMillisecondTimerCallback ***/


/************************************************************************************//**
** \brief     Setter function for the diagnostics session.
** \param     newSession Value for the new diagnostic session.
** \return    none.
**
****************************************************************************************/
static void DiagSetSession(tDiagSession newSession)
{
  /* upon change, make sure to lock the resources again so a seed/key sequence is
   * required for unlocking.
   */
  if (diagServerInfo.session != newSession)
  {
    diagServerInfo.sessionResourcesUnlocked = FALSE;
  }
  /* set the new session */
  diagServerInfo.session = newSession;
  /* stop the session timer when we are in the default session */
  if (diagServerInfo.session == DIAG_SESSION_DEFAULT)
  {
    DiagSessionTimerStop();
  }
} /*** end of DiagSetSession ***/


/************************************************************************************//**
** \brief     Transmits a positive or negative service response using the transport
**            protocol. This assumes that the response data is stored in the transmit
**            buffer.
** \return    TRUE if the data transfer request was successfully started, FALSE
**            otherwise.
**
****************************************************************************************/
static uint8_t DiagTpTransmitResponse(void)
{
  uint8_t result = TRUE;

  /* only actually transmit the response if this one isn't suppressed or doesn't have
   * any data.
   */
  if ( (diagServerInfo.suppressPosRspMsg == FALSE) && (DiagTxBufferScan() > 0) )
  {
    /* submit response for transmission with the transport protocol */
    result = TpTransfer(DiagTxBufferGetDataPtr(), DiagTxBufferScan());
  }
  else
  {
    /* no response need to be transmitted. so restart the session timer here instead
     * of the transmit confirmation.
     */
    DiagSessionTimerStart();
  }
  return result;
} /*** end of DiagTpTransmitResponse ***/


/************************************************************************************//**
** \brief     Transport protocol callback function that is called each time a first frame
**            was received.
** \param     len Total length of the data reception that is about to start.
** \return    none.
**
****************************************************************************************/
static void DiagTpFirstFrameReceived(uint16_t len)
{
  /* reception of a new diagnostic service request just started, so stop the session
   * timer.
   */
  DiagSessionTimerStop();
} /*** end of DiagTpFirstFrameReceived ***/


/************************************************************************************//**
** \brief     Transport protocol callback function that is called each time a data
**            reception completed. Use the result parameter to determine its status.
** \param     data Pointer to byte array with received data bytes.
** \param     len Total length of the data reception.
** \param     result TP_RESULT_OK if the data reception was successfully completed.
** \return    none.
**
****************************************************************************************/
static void DiagTpDataReceived(uint8_t *data, uint16_t len, tTpResult result)
{
  /* data reception successfully finished? */
  if (result == TP_RESULT_OK)
  {
    /* new diagnostic service request received so stop the session timer */
    DiagSessionTimerStop();
    /* handle reception of new service request */
    DiagHandleServiceRequest(data, len);
  }
  else
  {
    /* a reception error occurred, so no new diagnostic service request was received.
     * the session timer now needs to be restarted.
     */
    DiagSessionTimerStart();
  }
} /*** end of DiagTpDataReceived ***/


/************************************************************************************//**
** \brief     Transport protocol callback function that is called each time a data
**            transmission completed.
** \param     result TP_RESULT_OK if data transmission was successfully completed.
** \return    none.
**
****************************************************************************************/
static void DiagTpTransferComplete(tTpResult result)
{
  /* in case this was a positive response to ECU reset, then the actual reset should
   * be performed here.
   */
  if (diagServerInfo.ecuResetPending == TRUE)
  {
    /* reset flag (not really needed) */
    diagServerInfo.ecuResetPending = FALSE;
    /* perform the actual reset */
    OsSystemReset();
  }
  /* service response was sent, so restart the session timer */
  DiagSessionTimerStart();
} /*** end of DiagTpTransferComplete ***/


/************************************************************************************//**
** \brief     Start the S3 timer. According to the ISO spec:
**            - start when DiagnosticSessionControl positive response is sent for a
**              transition to a non-default session OR start when diagnostic session was
**              changed for a transition to a non-default session when no response is
**              required
**            - upon TP data rx indication with Error as the result AND session is
**              not default session
**            - confirmation of any service response message (pos and neg) OR
**              when the service was executed and it does not require a response to
**              be sent.
** \return    none.
**
****************************************************************************************/
static void DiagSessionTimerStart(void)
{
  /* S3 timer only needs to be active in a non-default session */
  if (diagServerInfo.session != DIAG_SESSION_DEFAULT)
  {
    /* start the timer with the max timeout time */
    diagServerInfo.s3_timer = DIAG_S3_TIMER_TIMEOUT_MS;
  }
  else
  {
    /* stop the timer */
    diagServerInfo.s3_timer = 0;
  }
} /*** end of DiagSessionTimerStart ***/


/************************************************************************************//**
** \brief     Stops the S3 timer. According to the ISO spec:
**            - when a transition from a non-default session to a default session takes
**              place.
**            - upon TP data rx indication for a single frame OR upon TP first frame rx
**              for a segmented transfer.
** \return    none.
**
****************************************************************************************/
static void DiagSessionTimerStop(void)
{
  /* stop the timer */
  diagServerInfo.s3_timer = 0;
} /*** end of DiagSessionTimerStop ***/


/************************************************************************************//**
** \brief     Updates the S3 timer. Should be called every millisecond. When the timer
**            times out, a switch to the default diagnostics session takes place.
** \return    none.
**
****************************************************************************************/
static void DiagSessionTimerUpdate(void)
{
  /* only update if active */
  if (diagServerInfo.s3_timer > 0)
  {
    /* 1 millisecond passed so decrement the timer */
    diagServerInfo.s3_timer--;
    /* did a timeout occurr? */
    if (diagServerInfo.s3_timer == 0)
    {
      /* transition back to the default session. note that the timer is already disabled
       * after a timeout.
       */
      DiagSetSession(DIAG_SESSION_DEFAULT);
    }
  }
} /*** end of DiagSessionTimerUpdate ***/


/************************************************************************************//**
** \brief     Initializes the buffer used for data transmission.
** \param     bufferSize Total number of storage bytes for the buffer.
** \return    none.
**
****************************************************************************************/
static void DiagTxBufferInit(uint16_t bufferSize)
{
  /* allocate memory for the buffer on the heap. */
  diagServerInfo.txBuffer.size = bufferSize;
  diagServerInfo.txBuffer.data = pvPortMalloc(diagServerInfo.txBuffer.size);
  /* make sure the allocation was successful */
  if (diagServerInfo.txBuffer.data == NULL)
  {
    ErrCodesSetError(ER_CODE_DIAG_TX_ALLOC_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* make sure the buffer is empty */
  diagServerInfo.txBuffer.entries = 0;
} /*** end of DiagTxBufferInit ***/


/************************************************************************************//**
** \brief     Empties out the buffer.
** \return    none.
**
****************************************************************************************/
static void DiagTxBufferFlush(void)
{
  uint32_t saved_cs_state;

  saved_cs_state = OsEnterCriticalSection();
  diagServerInfo.txBuffer.entries = 0;
  OsLeaveCriticalSection(saved_cs_state);
} /*** end of DiagTxBufferFlush ***/


/************************************************************************************//**
** \brief     Determines how many bytes are currently stored in the data buffer.
** \return    Number of bytes in the buffer.
**
****************************************************************************************/
static uint16_t DiagTxBufferScan(void)
{
  uint16_t entries;
  uint32_t saved_cs_state;

  saved_cs_state = OsEnterCriticalSection();
  entries = diagServerInfo.txBuffer.entries;
  OsLeaveCriticalSection(saved_cs_state);
  return entries;
} /*** end of DiagTxBufferScan ***/


/************************************************************************************//**
** \brief     Obtains the location of the start of data pointer.
** \return    Location of the data pointer.
**
****************************************************************************************/
static uint8_t *DiagTxBufferGetDataPtr(void)
{
  uint8_t *dataPtr;
  uint32_t saved_cs_state;

  saved_cs_state = OsEnterCriticalSection();
  dataPtr = diagServerInfo.txBuffer.data;
  OsLeaveCriticalSection(saved_cs_state);
  return dataPtr;
} /*** end of DiagTxBufferGetDataPtr ***/


/************************************************************************************//**
** \brief     Writes data to the buffer if it still fits. If not, then nothing is
**            written to the buffer.
** \param     data Pointer to byte array with data to place in the buffer.
** \param     len Number of bytes to copy to the buffer.
** \return    none.
**
****************************************************************************************/
static void DiagTxBufferWrite(uint8_t *data, uint16_t len)
{
  uint8_t result = FALSE;
  uint16_t cnt;

  uint32_t saved_cs_state;

  saved_cs_state = OsEnterCriticalSection();
  /* only write to the buffer if the data actually fits  */
  if (len <= (diagServerInfo.txBuffer.size - diagServerInfo.txBuffer.entries))
  {
    /* write byte for byte */
    for (cnt=0; cnt<len; cnt++)
    {
      /* store byte in buffer */
      diagServerInfo.txBuffer.data[diagServerInfo.txBuffer.entries] = data[cnt];
      diagServerInfo.txBuffer.entries++;
    }
    result = TRUE;
  }
  OsLeaveCriticalSection(saved_cs_state);

  /* trigger error if not all data fits in the buffer */
  if (result == FALSE)
  {
    ErrCodesSetError(ER_CODE_DIAG_TX_BUFFER_FULL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
} /*** end of DiagTxBufferWrite ***/


/************************************************************************************//**
** \brief     Handles the reception of new diagnostic service requests.
** \param     data Pointer to byte array with service request data,  including the
**            service identifier.
** \param     len Number of bytes in the data array.
** \return    none.
**
****************************************************************************************/
static void DiagHandleServiceRequest(uint8_t *data, uint16_t len)
{
  /* filter on the diagnostic service request */
  switch (data[0])
  {
    /* tester present */
    case DIAG_SERV_REQ_ID_TESTER_PRESENT:
      DiagProcessTesterPresent(&data[0], len);
      break;

    /* ECU reset */
    case DIAG_SERV_REQ_ID_ECU_RESET:
      DiagProcessEcuReset(&data[0], len);
      break;

    /* security access */
    case DIAG_SERV_REQ_ID_SECURITY_ACCESS:
      DiagProcessSecurityAccess(&data[0], len);
      break;

    /* security access */
    case DIAG_SERV_REQ_ID_SESSION_CONTROL:
      DiagProcessSessionControl(&data[0], len);
      break;

    /* read memory by address */
    case DIAG_SERV_REQ_ID_READ_MEM_BY_ADDR:
      DiagProcessReadMemoryByAddress(&data[0], len);
      break;

    /* write memory by address */
    case DIAG_SERV_REQ_ID_WRITE_MEM_BY_ADDR:
      DiagProcessWriteMemoryByAddress(&data[0], len);
      break;

    /* clear diagnostic information */
    case DIAG_SERV_REQ_ID_CLEAR_DIAG_INFO:
      DiagProcessClearDiagInfo(&data[0], len);
      break;

    /* read diagnostic trouble code (DTC) information */
    case DIAG_SERV_REQ_ID_READ_DTC_INFO:
      DiagProcessReadDTCInfo(&data[0], len);
      break;

    /* unknown service request */
    default:
      /* prepare the negative response data */
      DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_SERVICE_NOT_SUPPORTED);
      break;
  }
  /* transmit the response */
  DiagTpTransmitResponse();
} /*** end of DiagHandleServiceRequest ***/


/************************************************************************************//**
** \brief     Formats the data in the transmit buffer for sending the negative response.
** \param     serviceId Service identifier for which a negative response should be
**            prepared.
** \param     responseCode Negative response code.
** \return    none.
**
****************************************************************************************/
static void DiagPrepareNegativeResponse(uint8_t serviceId, uint8_t responseCode)
{
  uint8_t txByte;

  /* always send the response */
  diagServerInfo.suppressPosRspMsg = FALSE;
  /* start with an empty buffer */
  DiagTxBufferFlush();
  /* set response id to negative response */
  txByte = DIAG_SERV_RES_ID_NEGATIVE;
  DiagTxBufferWrite(&txByte, 1);
  /* set service id that isn't supported */
  txByte = serviceId;
  DiagTxBufferWrite(&txByte, 1);
  /* set negative response code */
  txByte = responseCode;
  DiagTxBufferWrite(&txByte, 1);
} /*** end of DiagPrepareNegativeResponse ***/


/************************************************************************************//**
** \brief     Diagnostic service processor for the service indicated by the function
**            name. Note that DiagTpTransmitResponse is automatically called after
**            this function runs so this function must place valid data in the transmit
**            buffer, or flush it to suppress a response.
** \param     data Pointer to byte array with service request data including the
**                 service identifier.
** \param     len Number of bytes in the data array.
** \return    none.
**
****************************************************************************************/
static void DiagProcessTesterPresent(uint8_t *data, uint16_t len)
{
  uint8_t txByte;
  uint8_t subFunction;

  /* this service supports sub-functions, so filter out the suppress positive response
   * bit.
   */
  if ( (data[1] & DIAG_BITMASK_SUPPRESS_POS_RES) == DIAG_BITMASK_SUPPRESS_POS_RES)
  {
    diagServerInfo.suppressPosRspMsg = TRUE;
  }
  else
  {
    diagServerInfo.suppressPosRspMsg = FALSE;
  }
  /* read out the sub function value */
  subFunction = data[1] & (uint8_t)(~DIAG_BITMASK_SUPPRESS_POS_RES);
  /* make sure the length is as expected */
  if (len != 2)
  {
    DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_INVALID_FORMAT);
  }
  /* make sure subfunction is supported. for tester present this is just 0x00,
   * zeroSubFunction.
   */
  else if (subFunction != 0x00)
  {
    DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_SUBFUNCTION_NOT_SUPPORTED);
  }
  /* valid tester present so prepare the positive response */
  else
  {
    /* start with an empty buffer */
    DiagTxBufferFlush();
    /* set positive response id */
    txByte = DIAG_SERV_POS_RES_ID_TESTER_PRESENT;
    DiagTxBufferWrite(&txByte, 1);
    /* set the zeroSubFunction */
    txByte = subFunction;
    DiagTxBufferWrite(&txByte, 1);
  }
} /*** end of DiagProcessTesterPresent ***/


/************************************************************************************//**
** \brief     Diagnostic service processor for the service indicated by the function
**            name.
** \param     data Pointer to byte array with service request data including the
**                 service identifier.
** \param     len Number of bytes in the data array.
** \return    none.
**
****************************************************************************************/
static void DiagProcessEcuReset(uint8_t *data, uint16_t len)
{
  uint8_t txByte;
  uint8_t subFunction;

  /* this service supports sub-functions, so filter out the suppress positive response
   * bit.
   */
  if ( (data[1] & DIAG_BITMASK_SUPPRESS_POS_RES) == DIAG_BITMASK_SUPPRESS_POS_RES)
  {
    diagServerInfo.suppressPosRspMsg = TRUE;
  }
  else
  {
    diagServerInfo.suppressPosRspMsg = FALSE;
  }
  /* read out the sub function value */
  subFunction = data[1] & (uint8_t)(~DIAG_BITMASK_SUPPRESS_POS_RES);
  /* make sure the length is as expected */
  if (len != 2)
  {
    DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_INVALID_FORMAT);
  }
  /* make sure the subfunction is supported. for ECU reset there are some reserved
   * ranged
   */
  else if ( (subFunction == 0x00) || (subFunction == 0x7F) ||
            ((subFunction >= 0x40) && (subFunction <= 0x5F)) )
  {
    DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_SUBFUNCTION_NOT_SUPPORTED);
  }
  /* valid tester present so prepare the positive response */
  else
  {
    /* start with an empty buffer */
    DiagTxBufferFlush();
    /* set positive response id */
    txByte = DIAG_SERV_POS_RES_ID_ECU_RESET;
    DiagTxBufferWrite(&txByte, 1);
    /* set the reset type */
    txByte = subFunction;
    DiagTxBufferWrite(&txByte, 1);
    /* set the powerdown time if the resettype is enableRapidPowershutdown (0x04) */
    if (subFunction == 0x04)
    {
      txByte = 0;
      DiagTxBufferWrite(&txByte, 1);
    }
    /* response prepared, but it won't be sent in case the positive response is
     * suppored. in this case the ECU reset can be performed immediately. otherwise
     * a flag should be set so that the ECU reset can be performed after the
     * response was confirmed
     */
    if (diagServerInfo.suppressPosRspMsg == TRUE)
    {
      /* perform ECU reset */
      OsSystemReset();
    }
    else
    {
      /* set reset pending flag */
      diagServerInfo.ecuResetPending = TRUE;
    }
  }

} /*** end of DiagProcessEcuReset ***/


/************************************************************************************//**
** \brief     Diagnostic service processor for the service indicated by the function
**            name.
** \param     data Pointer to byte array with service request data including the
**                 service identifier.
** \param     len Number of bytes in the data array.
** \return    none.
**
****************************************************************************************/
static void DiagProcessSecurityAccess(uint8_t *data, uint16_t len)
{
  static uint8_t seedSendFlag = FALSE;
  uint8_t txByte;
  uint8_t subFunction;
  uint32_t key;
  uint32_t seed;

  /* this service supports sub-functions, so filter out the suppress positive response
   * bit.
   */
  if ( (data[1] & DIAG_BITMASK_SUPPRESS_POS_RES) == DIAG_BITMASK_SUPPRESS_POS_RES)
  {
    diagServerInfo.suppressPosRspMsg = TRUE;
  }
  else
  {
    diagServerInfo.suppressPosRspMsg = FALSE;
  }
  /* read out the sub function value */
  subFunction = data[1] & (uint8_t)(~DIAG_BITMASK_SUPPRESS_POS_RES);
  /* the security access service is not available in the default session */
  if (diagServerInfo.session == DIAG_SESSION_DEFAULT)
  {
    DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_SERVICE_NOT_SUPPORTED);
  }
  else
  {
    /* is this a seed request? */
    if ( (subFunction == 0x01) || (subFunction == 0x03) || (subFunction == 0x05) )
    {
      /* note that an optional securityAccessDataRecord is allowed of unknown length,
       * this means that no check for request message length can be performed.
       */
      if (diagCallbackGetSeed != NULL)
      {
        /* invoke callback to get the seed. the application will typically use function
         * DiagSetSeed() in this callback function to configure the seed.
         */
        diagCallbackGetSeed();
      }
      /* read the configured seed */
      seed = DiagGetSeed();
      /* set the seed to 0 in case the session is already unlocked */
      if (diagServerInfo.sessionResourcesUnlocked == TRUE)
      {
        seed = 0;
      }
      /* set flag to be able to detect a sequence error */
      seedSendFlag = TRUE;
      /* prepare the response. start with an empty buffer */
      DiagTxBufferFlush();
      /* set positive response id */
      txByte = DIAG_SERV_POS_RES_ID_SECURITY_ACCESS;
      DiagTxBufferWrite(&txByte, 1);
      /* set the type */
      txByte = subFunction;
      DiagTxBufferWrite(&txByte, 1);
      /* set the seed, high byte first */
      txByte = (uint8_t)(seed >> 24);
      DiagTxBufferWrite(&txByte, 1);
      txByte = (uint8_t)(seed >> 16);
      DiagTxBufferWrite(&txByte, 1);
      txByte = (uint8_t)(seed >> 8);
      DiagTxBufferWrite(&txByte, 1);
      txByte = (uint8_t)(seed);
      DiagTxBufferWrite(&txByte, 1);
    }
    /* is this a send key? */
    else if ( (subFunction == 0x02) || (subFunction == 0x04) || (subFunction == 0x06) )
    {
      /* this implementation only supports 4-byte keys so check the length for this */
      if (len != 6)
      {
        DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_INVALID_FORMAT);
      }
      /* make sure the sequence is correct, so that we first sent the seed */
      else if (seedSendFlag != TRUE)
      {
        DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_SEQUENCE_ERROR);
      }
      else
      {
        /* read out the key, high byte first */
        key  = data[2] << 24;
        key |= data[3] << 16;
        key |= data[4] <<  8;
        key |= data[5];
        /* store the key */
        diagKey = key;
        if (diagCallbackVerifyKey != NULL)
        {
          /* invoke callback to verify the key. typically function DiagSetKeyVerified()
           * is used inside this callback function to verify if the key was valid or
           * not.
           */
          diagCallbackVerifyKey();
        }
        /* check key validity. if the key was valid, then sessionResourcesUnlocked will
         * have been set to TRUE by function DiagSetKeyVerified()
         */
        if (diagServerInfo.sessionResourcesUnlocked == FALSE)
        {
          DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_INVALID_KEY);
        }
        else
        {
          /* prepare the response. start with an empty buffer */
          DiagTxBufferFlush();
          /* set positive response id */
          txByte = DIAG_SERV_POS_RES_ID_SECURITY_ACCESS;
          DiagTxBufferWrite(&txByte, 1);
          /* set the type */
          txByte = subFunction;
          DiagTxBufferWrite(&txByte, 1);
        }
      }
      /* reset the flag to be able to properly detect the next seed/key sequence */
      seedSendFlag = FALSE;
    }
    else
    {
      /* unsupported subfunction */
      DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_SUBFUNCTION_NOT_SUPPORTED);
    }
  }
} /*** end of DiagProcessSecurityAccess ***/


/************************************************************************************//**
** \brief     Diagnostic service processor for the service indicated by the function
**            name.
** \param     data Pointer to byte array with service request data including the
**                 service identifier.
** \param     len Number of bytes in the data array.
** \return    none.
**
****************************************************************************************/
static void DiagProcessSessionControl(uint8_t *data, uint16_t len)
{
  uint8_t txByte;
  uint8_t subFunction;

  /* this service supports sub-functions, so filter out the suppress positive response
   * bit.
   */
  if ( (data[1] & DIAG_BITMASK_SUPPRESS_POS_RES) == DIAG_BITMASK_SUPPRESS_POS_RES)
  {
    diagServerInfo.suppressPosRspMsg = TRUE;
  }
  else
  {
    diagServerInfo.suppressPosRspMsg = FALSE;
  }
  /* read out the sub function value */
  subFunction = data[1] & (uint8_t)(~DIAG_BITMASK_SUPPRESS_POS_RES);
  /* make sure the length is as expected */
  if (len != 2)
  {
    DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_INVALID_FORMAT);
  }
  /* make sure subfunction is supported. for session control only the default session
   * and the extended session are currently supported.
   */
  else if ( (subFunction != 0x01) && (subFunction != 0x03) )
  {
    DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_SUBFUNCTION_NOT_SUPPORTED);
  }
  /* valid session control request, so process it */
  else
  {
    /* set the new session */
    if (subFunction == 0x03)
    {
      DiagSetSession(DIAG_SESSION_ECU_EXTENDED);
    }
    else
    {
      DiagSetSession(DIAG_SESSION_DEFAULT);
    }
    /* start with an empty buffer */
    DiagTxBufferFlush();
    /* set positive response id */
    txByte = DIAG_SERV_POS_RES_ID_SESSION_CONTROL;
    DiagTxBufferWrite(&txByte, 1);
    /* set the session SubFunction value */
    txByte = subFunction;
    DiagTxBufferWrite(&txByte, 1);
    /* set P2can_server_max high byte */
    txByte = (uint8_t)(50 >> 8);
    DiagTxBufferWrite(&txByte, 1);
    /* set P2can_server_max low byte */
    txByte = (uint8_t)(50);
    DiagTxBufferWrite(&txByte, 1);
    /* set P2*can_server_max high byte */
    txByte = (uint8_t)(5000 >> 8);
    DiagTxBufferWrite(&txByte, 1);
    /* set P2*can_server_max low byte */
    txByte = (uint8_t)(5000);
    DiagTxBufferWrite(&txByte, 1);
  }
} /*** end of DiagProcessSessionControl ***/


/************************************************************************************//**
** \brief     Diagnostic service processor for the service indicated by the function
**            name. Note that DiagTpTransmitResponse is automatically called after
**            this function runs so this function must place valid data in the transmit
**            buffer, or flush it to suppress a response.
** \param     data Pointer to byte array with service request data including the
**                 service identifier.
** \param     len Number of bytes in the data array.
** \return    none.
**
****************************************************************************************/
static void DiagProcessReadMemoryByAddress(uint8_t *data, uint16_t len)
{
  uint8_t txByte;
  uint8_t okToProcess;
  uint8_t numBytesAddr;
  uint8_t numBytesSize;
  uint32_t memAddr;
  uint32_t memSize;
  int8_t cnt;
  uint8_t bitShiftNum;

  /* this service does not support sub-functions, so make sure the positive response
   * is not suppressed.
   */
  diagServerInfo.suppressPosRspMsg = FALSE;

  /* check module configuration. if requireSeedKey == TRUE, then this service is not
   * part of the default session, but the extended session and the resources must
   * be unlocked
   */
  okToProcess = FALSE;
  if (diagServerInfo.requireSeedKey == TRUE)
  {
    /* must be in the extended session with unlocked resources */
    if ( (diagServerInfo.session == DIAG_SESSION_ECU_EXTENDED) &&
         (diagServerInfo.sessionResourcesUnlocked == TRUE) )
    {
      okToProcess = TRUE;
    }
  }
  else
  {
    /* this service is part of default session so always allow */
    okToProcess = TRUE;
  }
  /* read out the number of bytes used for the address and the size */
  numBytesAddr = data[1] & 0x0f;
  numBytesSize = (data[1] & 0xf0) >> 4;

  /* check processing state */
  if (okToProcess == FALSE)
  {
    /* not allow to process because we are in the default session? */
    if (diagServerInfo.session == DIAG_SESSION_DEFAULT)
    {
      DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_SERVICE_NOT_SUPPORTED);
    }
    /* the session is correct but the resource is still locked */
    else
    {
      DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_ACCESS_DENIED);
    }
  }
  /* check that address is not more than 4 bytes because of the 32-bit memory map.
   * check that the size is not more that 2 bytes because the transmit buffer
   * can only handle data size of 16-bits. also make sure that these values
   * are not zero, because those are invalid.
   * largest address we can store.
   */
  else if ( (numBytesAddr > 4) || (numBytesSize > 2) ||
            (numBytesAddr == 0) || (numBytesSize == 0) )
  {
    DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_REQUEST_OUT_OF_RANGE);
  }
  else
  {
    /* construct the address */
    memAddr = 0;
    bitShiftNum = 0;
    for (cnt=(numBytesAddr-1); cnt>=0; cnt--)
    {
      memAddr |= (data[2 + cnt] << (bitShiftNum));
      bitShiftNum += 8;
    }
    /* construct the size */
    memSize = 0;
    bitShiftNum = 0;
    for (cnt=(numBytesSize-1); cnt>=0; cnt--)
    {
      memSize |= (data[2 + numBytesAddr + cnt] << (bitShiftNum));
      bitShiftNum += 8;
    }
    /* prepare the response. start with an empty buffer */
    DiagTxBufferFlush();
    /* set positive response id */
    txByte = DIAG_SERV_POS_RES_ID_READ_MEM_BY_ADDR;
    DiagTxBufferWrite(&txByte, 1);
    /* read data from memory and write it to the transmit buffer */
    DiagTxBufferWrite((uint8_t *)memAddr, memSize);
  }
} /*** end of DiagProcessReadMemoryByAddress ***/


/************************************************************************************//**
** \brief     Diagnostic service processor for the service indicated by the function
**            name. Note that DiagTpTransmitResponse is automatically called after
**            this function runs so this function must place valid data in the transmit
**            buffer, or flush it to suppress a response.
** \param     data Pointer to byte array with service request data including the
**                 service identifier.
** \param     len Number of bytes in the data array.
** \return    none.
**
****************************************************************************************/
static void DiagProcessWriteMemoryByAddress(uint8_t *data, uint16_t len)
{
  uint8_t txByte;
  uint8_t okToProcess;
  uint8_t numBytesAddr;
  uint8_t numBytesSize;
  uint32_t memAddr;
  uint32_t memSize;
  int8_t cnt;
  uint8_t bitShiftNum;

  /* this service does not support sub-functions, so make sure the positive response
   * is not suppressed.
   */
  diagServerInfo.suppressPosRspMsg = FALSE;

  /* check module configuration. if requireSeedKey == TRUE, then this service is not
   * part of the default session, but the extended session and the resources must
   * be unlocked
   */
  okToProcess = FALSE;
  if (diagServerInfo.requireSeedKey == TRUE)
  {
    /* must be in the extended session with unlocked resources */
    if ( (diagServerInfo.session == DIAG_SESSION_ECU_EXTENDED) &&
         (diagServerInfo.sessionResourcesUnlocked == TRUE) )
    {
      okToProcess = TRUE;
    }
  }
  else
  {
    /* this service is part of default session so always allow */
    okToProcess = TRUE;
  }
  /* read out the number of bytes used for the address and the size */
  numBytesAddr = data[1] & 0x0f;
  numBytesSize = (data[1] & 0xf0) >> 4;

  /* check processing state */
  if (okToProcess == FALSE)
  {
    /* not allow to process because we are in the default session? */
    if (diagServerInfo.session == DIAG_SESSION_DEFAULT)
    {
      DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_SERVICE_NOT_SUPPORTED);
    }
    /* the session is correct but the resource is still locked */
    else
    {
      DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_ACCESS_DENIED);
    }
  }
  /* check that address is not more than 4 bytes because of the 32-bit memory map.
   * check that the size is not more that 2 bytes because the transmit buffer
   * can only handle data size of 16-bits. also make sure that these values
   * are not zero, because those are invalid.
   * largest address we can store.
   */
  else if ( (numBytesAddr > 4) || (numBytesSize > 2) ||
            (numBytesAddr == 0) || (numBytesSize == 0) )
  {
    DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_REQUEST_OUT_OF_RANGE);
  }
  else
  {
    /* construct the address */
    memAddr = 0;
    bitShiftNum = 0;
    for (cnt=(numBytesAddr-1); cnt>=0; cnt--)
    {
      memAddr |= (data[2 + cnt] << (bitShiftNum));
      bitShiftNum += 8;
    }
    /* construct the size */
    memSize = 0;
    bitShiftNum = 0;
    for (cnt=(numBytesSize-1); cnt>=0; cnt--)
    {
      memSize |= (data[2 + numBytesAddr + cnt] << (bitShiftNum));
      bitShiftNum += 8;
    }
    /* write the data to memory, assuming that it is in RAM */
    memcpy((uint8_t *)memAddr, &data[2 + numBytesAddr + numBytesSize], memSize);
    /* prepare the response. start with an empty buffer */
    DiagTxBufferFlush();
    /* set positive response id */
    txByte = DIAG_SERV_POS_RES_ID_WRITE_MEM_BY_ADDR;
    DiagTxBufferWrite(&txByte, 1);
    /* set the format specifier */
    txByte = data[1];
    DiagTxBufferWrite(&txByte, 1);
    /* set the address */
    DiagTxBufferWrite(&data[2], numBytesAddr);
    /* set the size */
    DiagTxBufferWrite(&data[2 + numBytesAddr], numBytesSize);
  }
} /*** end of DiagProcessWriteMemoryByAddress ***/


/************************************************************************************//**
** \brief     Diagnostic service processor for the service indicated by the function
**            name. Note that DiagTpTransmitResponse is automatically called after
**            this function runs so this function must place valid data in the transmit
**            buffer, or flush it to suppress a response.
** \param     data Pointer to byte array with service request data including the
**                 service identifier.
** \param     len Number of bytes in the data array.
** \return    none.
**
****************************************************************************************/
static void DiagProcessClearDiagInfo(uint8_t *data, uint16_t len)
{
  uint8_t txByte;

  /* this service does not support sub-functions, so make sure the positive response
   * is not suppressed.
   */
  diagServerInfo.suppressPosRspMsg = FALSE;

  /* check the message length. it should be 4 (first the service id and then 3 bytes
   * for groupDTC, even thoug this is not used.
   */
  if (len != 4)
  {
    DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_INVALID_FORMAT);
  }
  else
  {
    /* delete stored and active errors */
    ErrCodesDelActiveErrors();
    ErrCodesDelSavedErrors();
    /* prepare the response. start with an empty buffer */
    DiagTxBufferFlush();
    /* set positive response id */
    txByte = DIAG_SERV_POS_RES_ID_CLEAR_DIAG_INFO;
    DiagTxBufferWrite(&txByte, 1);
  }
} /*** end of DiagProcessClearDiagInfo ***/


/************************************************************************************//**
** \brief     Diagnostic service processor for the service indicated by the function
**            name. Note that DiagTpTransmitResponse is automatically called after
**            this function runs so this function must place valid data in the transmit
**            buffer, or flush it to suppress a response.
** \param     data Pointer to byte array with service request data including the
**                 service identifier.
** \param     len Number of bytes in the data array.
** \return    none.
**
****************************************************************************************/
static void DiagProcessReadDTCInfo(uint8_t *data, uint16_t len)
{
  uint8_t txByte;
  uint8_t subFunction;
  uint16_t numErrors;
  uint16_t errorIdx;
  tErrCodeData errorData;

  /* this service supports sub-functions, so filter out the suppress positive response
   * bit.
   */
  if ( (data[1] & DIAG_BITMASK_SUPPRESS_POS_RES) == DIAG_BITMASK_SUPPRESS_POS_RES)
  {
    diagServerInfo.suppressPosRspMsg = TRUE;
  }
  else
  {
    diagServerInfo.suppressPosRspMsg = FALSE;
  }
  /* read out the sub function value */
  subFunction = data[1] & (uint8_t)(~DIAG_BITMASK_SUPPRESS_POS_RES);
  /* check if the subfunction is supported. the following subfunctions are currently
   * supported:
   *  0x01 - reportNumberOfDTCByStatusMask
   *  0x02 - reportDTCByStatusMask
   */
  if ( (subFunction != 0x01) && (subFunction != 0x02) )
  {
    DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_SUBFUNCTION_NOT_SUPPORTED);
  }
  /* check the length of the message. it should be 3 bytes */
  else if (len != 3)
  {
    DiagPrepareNegativeResponse(data[0], DIAG_SERV_NRC_INVALID_FORMAT);
  }
  else
  {
    /* The following statusmask bits are supported by the server:
     *  BIT2 - 0x04 - pendingDTC - Active error codes available in RAM.
     *  BIT3 - 0x08 - confirmedDTC - Stored error codes available in EEPROM.
     */

    /* ---------------------- reportNumberOfDTCByStatusMask -------------------------- */
    if (subFunction == 0x01)
    {
      /* initialize to zero for the case that the DTCStatusMask (data[2]) contains
       * unsupported bits.
       */
      numErrors = 0;
      /* determine number of confirmedDTCs in EEPROM */
      if ((data[2] & 0x08) == 0x08)
      {
        numErrors += ErrCodesGetNumSavedErrors();
      }
      /* determine number of activeDTCs in RAM */
      if ((data[2] & 0x04) == 0x04)
      {
        numErrors += ErrCodesGetNumActiveErrors();
      }
      /* start with an empty buffer */
      DiagTxBufferFlush();
      /* set positive response id */
      txByte = DIAG_SERV_POS_RES_ID_READ_DTC_INFO;
      DiagTxBufferWrite(&txByte, 1);
      /* set the report type which is the same as the subfunction value */
      txByte = subFunction;
      DiagTxBufferWrite(&txByte, 1);
      /* set the DTCStatusAvailabilityMask */
      txByte = 0x08 | 0x04; /* confirmedDTC and pendingDTC */
      DiagTxBufferWrite(&txByte, 1);
      /* set the DTCFormatIdentifier */
      txByte = 0x01; /* ISO14229-1DTCFormat */
      DiagTxBufferWrite(&txByte, 1);
      /* set the DTCcount. first the MSB then the LSB */
      txByte = (uint8_t)(numErrors >> 8);
      DiagTxBufferWrite(&txByte, 1);
      txByte = (uint8_t)numErrors;
      DiagTxBufferWrite(&txByte, 1);
    }
    /* ---------------------- reportDTCByStatusMask ---------------------------------- */
    else if (subFunction == 0x02)
    {
      /* note that our error codes are UINT16 with an additional UINT8 param. UDS DTCs
       * are 3 bytes long so we can squeeze both the error code and the param in there:
       *  --------------------------------
       *  |HIGH_BYTE |MIDDLE_BYTE|LOW_BYTE|
       *  |----------|-----------|--------|
       *  |MSB error | LSB error | param  |
       *  --------------------------------
       */
      /* start with an empty buffer */
      DiagTxBufferFlush();
      /* set positive response id */
      txByte = DIAG_SERV_POS_RES_ID_READ_DTC_INFO;
      DiagTxBufferWrite(&txByte, 1);
      /* set the report type which is the same as the subfunction value */
      txByte = subFunction;
      DiagTxBufferWrite(&txByte, 1);
      /* set the DTCStatusAvailabilityMask */
      txByte = 0x08 | 0x04; /* confirmedDTC and pendingDTC */
      DiagTxBufferWrite(&txByte, 1);
      /* determine number of confirmedDTCs in EEPROM */
      if ((data[2] & 0x08) == 0x08)
      {
        numErrors = ErrCodesGetNumSavedErrors();
        for (errorIdx=0; errorIdx<numErrors; errorIdx++)
        {
          ErrCodesGetSavedErrors(&errorData, errorIdx, 1);
          txByte = (uint8_t)(errorData.code >> 8);
          DiagTxBufferWrite(&txByte, 1);
          txByte = (uint8_t)errorData.code;
          DiagTxBufferWrite(&txByte, 1);
          txByte = errorData.param;
          DiagTxBufferWrite(&txByte, 1);
          txByte = 0x08; /* confirmedDTC */
          DiagTxBufferWrite(&txByte, 1);
        }
      }
      /* determine number of pendingDTCs in RAM */
      if ((data[2] & 0x04) == 0x04)
      {
        numErrors = ErrCodesGetNumActiveErrors();
        for (errorIdx=0; errorIdx<numErrors; errorIdx++)
        {
          ErrCodesGetActiveErrors(&errorData, errorIdx, 1);
          txByte = (uint8_t)(errorData.code >> 8);
          DiagTxBufferWrite(&txByte, 1);
          txByte = (uint8_t)errorData.code;
          DiagTxBufferWrite(&txByte, 1);
          txByte = errorData.param;
          DiagTxBufferWrite(&txByte, 1);
          txByte = 0x04; /* pendingDTC */
          DiagTxBufferWrite(&txByte, 1);
        }
      }
    }
  }
} /*** end of DiagProcessReadDTCInfo ***/


/*********************************** end of diag.c *************************************/
