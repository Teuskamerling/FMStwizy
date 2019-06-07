/*
 * CAN module object for generic microcontroller.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @version     SVN: \$Id: CO_driver.c 57 2014-10-09 15:37:29Z jani22 $
 * @author      Janez Paternoster
 * @copyright   2004 - 2013 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <http://canopennode.sourceforge.net>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * CANopenNode is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */


#include "CANopen.h"
#include "CO_driver.h"
#include "CO_Emergency.h"
#include "can.h"                                      /* CAN driver header file        */
#include "co_node.h"                                  /* for CANopen node              */
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */

/**
 * Function prototypes
 */
static void CO_CANMessageReceived(uint32_t id, uint8_t dlc, uint8_t *data, uint32_t timestamp);
static void CO_CANMessageTransmitted(uint32_t id);
static void CO_CANErrorDetected(tCanErrStatus error);


/**
 * Local data declarations
 */
static uint8_t rxOverrunDetectedFlg = 0u;
static uint8_t busOffDetectedFlg = 0u;


/******************************************************************************/
void CO_CANsetConfigurationMode(uint16_t CANbaseAddress){
  /* config mode is set automatically by CanInit() so nothing special to do here */
}


/******************************************************************************/
void CO_CANsetNormalMode(uint16_t CANbaseAddress){
  /* normal mode is set automatically by CanInit() so nothing special to do here */
}


/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        uint16_t                CANbaseAddress,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{
    uint16_t i;
    uint8_t canChannel;

    /* configure the acceptance filter. CANopen typically works with 11-bit
     * CAN identifiers, so limit the reception to just those messages.
     */
    tCanFilter canFilter = { 0x00000000, 0x00000000, CAN_FILTER_MODE_STDID_ONLY };

    /* store the channel that the CANopen stack is configured to run on */
    canChannel = CoNodeGetChannel();

    /* Configure object variables */
    CANmodule->CANbaseAddress = CANbaseAddress;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    /* bxCan is not a fullcan controller so use them as for generic reception */
    CANmodule->useCANrxFilters = CO_false;
    CANmodule->bufferInhibitFlag = CO_false;
    CANmodule->firstCANtxMessage = CO_true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;
    CANmodule->em = NULL;

    for(i=0U; i<rxSize; i++){
        rxArray[i].ident = 0U;
        rxArray[i].pFunct = NULL;
    }
    for(i=0U; i<txSize; i++){
        txArray[i].bufferFull = CO_false;
    }
    
    /* initialize CAN controller, configure baudrate, configure acceptance filter, 
     * switch to normal mode and enable interrupts. note that the low level CAN driver
     * could be used for other communication as well. therefore, this driver assumes
     * that the CAN driver is already initialized. In case it was already connected,
     * it will be disconnected and re-connected with the baudrate configured for the
     * CANopen node.
     */
    if (CanIsConnected(canChannel))
    {
      CanDisconnect(canChannel);
    }
    if (CanConnect(canChannel, CANbitRate * 1000ul, &canFilter) == FALSE)
    {
      return CO_ERROR_PARAMETERS;
    }
    
    /* reset error flags */
    rxOverrunDetectedFlg = 0u;
    busOffDetectedFlg = 0u;
    
    /* register callback functions */
    CanRegisterGenericTransmittedCallback(canChannel, CO_CANMessageTransmitted);
    CanRegisterGenericReceivedCallback(canChannel, CO_CANMessageReceived);
    CanRegisterErrorCallback(canChannel,CO_CANErrorDetected);

    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule){
    uint8_t canChannel;

    /* store the channel that the CANopen stack is configured to run on */
    canChannel = CoNodeGetChannel();

    /* turn off the module */
    if (CanIsConnected(canChannel))
    {
      CanDisconnect(canChannel);
    }
}


/******************************************************************************/
uint16_t CO_CANrxMsg_readIdent(const CO_CANrxMsg_t *rxMsg){
    return (uint16_t) rxMsg->ident;
}


/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        CO_bool_t               rtr,
        void                   *object,
        void                  (*pFunct)(void *object, const CO_CANrxMsg_t *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if((CANmodule!=NULL) && (object!=NULL) && (pFunct!=NULL) && (index < CANmodule->rxSize)){
        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->pFunct = pFunct;

        /* CAN identifier and CAN mask. bit alignment is taken care of by the underlying
         * CAN driver. note that 29-bit IDs are not supported currently.
         */
        buffer->ident = ident & 0x07FFU;
        if (rtr) {
            buffer->ident |= CAN_RTR_FRAME_MASK;
        }
        buffer->mask = (mask & 0x07FFU) | CAN_RTR_FRAME_MASK;

    }
    else{
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}


/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        CO_bool_t               rtr,
        uint8_t                 noOfBytes,
        CO_bool_t               syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if((CANmodule != NULL) && (index < CANmodule->txSize)){
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier and DLC. bit alignment is taken care of by the underlying
         * CAN driver. Note that RTR frame and 28-bit IDs are not supported currently.
         */
        buffer->ident = ident & 0x07FFU;
        if (rtr) {
            buffer->ident |= CAN_RTR_FRAME_MASK;
        }
        buffer->DLC = noOfBytes;
        buffer->bufferFull = CO_false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}


/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer){
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if(buffer->bufferFull){
        if(!CANmodule->firstCANtxMessage){
            /* don't set error, if bootup message is still on buffers */
            CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, buffer->ident);
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    CO_DISABLE_INTERRUPTS();
    /* attempt to copy the message and start its transmission */
    if (CanTransmit(CoNodeGetChannel(), buffer->ident, buffer->DLC, buffer->data) == TRUE)
    {
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
    }
    else
    {
        /* could not tranmsit. probably no buffer free so keep it in buffer and transmit
         * it the moment a message transmission completed.
         */
        buffer->bufferFull = CO_true;
        CANmodule->CANtxCount++;
    }
    CO_ENABLE_INTERRUPTS();

    return err;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule){
    uint32_t tpdoDeleted = 0U;

    CO_DISABLE_INTERRUPTS();
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if(CANmodule->bufferInhibitFlag){
        /* clear TXREQ */
        CanCancelTransmit(CoNodeGetChannel());
        CANmodule->bufferInhibitFlag = CO_false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if(CANmodule->CANtxCount != 0U){
        uint16_t i;
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        for(i = CANmodule->txSize; i > 0U; i--){
            if(buffer->bufferFull){
                if(buffer->syncFlag){
                    buffer->bufferFull = CO_false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_ENABLE_INTERRUPTS();


    if(tpdoDeleted != 0U){
        CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_TPDO_OUTSIDE_WINDOW, CO_EMC_COMMUNICATION, tpdoDeleted);
    }
}


/******************************************************************************/
static void CO_CANMessageReceived(uint32_t id, uint8_t dlc, uint8_t *data, uint32_t timestamp){
    CO_CANmodule_t *CANmodule = CO->CANmodule[0];
    CO_CANrxMsg_t rcvMsg;      /* pointer to received message in CAN module */
    uint16_t index;             /* index of received message */
    uint16_t rcvMsgIdent;       /* identifier of the received message */
    CO_CANrx_t *buffer = NULL;  /* receive message buffer from CO_CANmodule_t object. */
    CO_bool_t msgMatched = CO_false;

    /* copy the message */
    rcvMsg.ident = id;
    rcvMsg.DLC = dlc;
    for (index=0; index<dlc; index++)
    {
      rcvMsg.data[index] = data[index];
    }
    rcvMsgIdent = rcvMsg.ident;

    /* CAN module filters are not used, message with any standard 11-bit identifier */
    /* has been received. Search rxArray form CANmodule for the same CAN-ID. */
    buffer = &CANmodule->rxArray[0];
    for(index = CANmodule->rxSize; index > 0U; index--){
        if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U){
            msgMatched = CO_true;
            break;
        }
        buffer++;
    }

    /* Call specific function, which will process the message */
    if(msgMatched && (buffer != NULL) && (buffer->pFunct != NULL)){
        buffer->pFunct(buffer->object, &rcvMsg);
    }

}


/******************************************************************************/
static void CO_CANMessageTransmitted(uint32_t id){
    CO_CANmodule_t *CANmodule = CO->CANmodule[0];

    /* First CAN message (bootup) was sent successfully */
    CANmodule->firstCANtxMessage = CO_false;
    /* clear flag from previous message */
    CANmodule->bufferInhibitFlag = CO_false;
    /* Are there any new messages waiting to be send */
    if(CANmodule->CANtxCount > 0U){
        uint16_t i;             /* index of transmitting message */

        /* first buffer */
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        /* search through whole array of pointers to transmit message buffers. */
        for(i = CANmodule->txSize; i > 0U; i--){
            /* if message buffer is full, send it. */
            if(buffer->bufferFull){
                buffer->bufferFull = CO_false;
                CANmodule->CANtxCount--;
                
                /* attempt to copy the message and start its transmission */
                if (CanTransmit(CoNodeGetChannel(), buffer->ident, buffer->DLC, buffer->data) == TRUE)
                {
                    CANmodule->bufferInhibitFlag = buffer->syncFlag;
                }
                else
                {
                    /* could not tranmsit. probably no buffer free so keep it in buffer and transmit
                     * it the moment a message transmission completed.
                     */
                    buffer->bufferFull = CO_true;
                    CANmodule->CANtxCount++;
                }
                break;                      /* exit for loop */
            }
            buffer++;
        }/* end of for loop */

        /* Clear counter if no more messages */
        if(i == 0U){
            CANmodule->CANtxCount = 0U;
        }
    }

}


/******************************************************************************/
static void CO_CANErrorDetected(tCanErrStatus error){
    if (error == CAN_ERR_STAT_RX_OVERFLOW)
    {
        /* set error flag */
        rxOverrunDetectedFlg = CO_true;
    }
    else if (error == CAN_ERR_STAT_BUS_OFF)
    {
        /* set error flag */
        busOffDetectedFlg = CO_true;
    }
}


/******************************************************************************/
void CO_CANverifyErrors(CO_CANmodule_t *CANmodule){
    uint16_t rxErrors, txErrors, overflow, busOff;
    CO_EM_t* em = (CO_EM_t*)CANmodule->em;
    uint32_t err;
    uint8_t canChannel;

    /* get the channel that we are operating on */
    canChannel = CoNodeGetChannel();

    /* get error counters from module. Id possible, function may use different way to
     * determine errors. */
    rxErrors = CanGetErrorCount(canChannel, CAN_ERR_COUNT_RECEPTION);
    txErrors = CanGetErrorCount(canChannel, CAN_ERR_COUNT_TRANSMISSION);
    overflow = rxOverrunDetectedFlg;
    busOff = busOffDetectedFlg;
    

    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | ((uint32_t)(overflow << 1)) | busOff;

    if(CANmodule->errOld != err){
        CANmodule->errOld = err;

        if(busOff != 0U){                               /* bus off */
            CO_errorReport(em, CO_EM_CAN_TX_BUS_OFF, CO_EMC_BUS_OFF_RECOVERED, err);
            /* event detected so reset the global flag */
            busOffDetectedFlg = 0u;
        }
        else{                                               /* not bus off */
            CO_errorReset(em, CO_EM_CAN_TX_BUS_OFF, err);

            if((rxErrors >= 96U) || (txErrors >= 96U)){     /* bus warning */
                CO_errorReport(em, CO_EM_CAN_BUS_WARNING, CO_EMC_NO_ERROR, err);
            }

            if(rxErrors >= 128U){                           /* RX bus passive */
                CO_errorReport(em, CO_EM_CAN_RX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
            }
            else{
                CO_errorReset(em, CO_EM_CAN_RX_BUS_PASSIVE, err);
            }

            if(txErrors >= 128U){                           /* TX bus passive */
                if(!CANmodule->firstCANtxMessage){
                    CO_errorReport(em, CO_EM_CAN_TX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
                }
            }
            else{
                CO_bool_t isError = CO_isError(em, CO_EM_CAN_TX_BUS_PASSIVE);
                if(isError){
                    CO_errorReset(em, CO_EM_CAN_TX_BUS_PASSIVE, err);
                    CO_errorReset(em, CO_EM_CAN_TX_OVERFLOW, err);
                }
            }

            if((rxErrors < 96U) && (txErrors < 96U)){       /* no error */
                CO_errorReset(em, CO_EM_CAN_BUS_WARNING, err);
            }
        }

        if(overflow != 0U){                                 /* CAN RX bus overflow */
            CO_errorReport(em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_CAN_OVERRUN, err);
            /* event detected so reset the global flag */
            rxOverrunDetectedFlg = 0u;
        }
    }
}


