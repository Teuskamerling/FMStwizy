/*
 * CANopen NMT and Heartbeat producer object.
 *
 * @file        CO_NMT_Heartbeat.c
 * @ingroup     CO_NMT_Heartbeat
 * @version     SVN: \$Id: CO_NMT_Heartbeat.c 42 2013-07-09 11:14:12Z jani22 $
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


#include "CO_driver.h"
#include "CO_SDO.h"
#include "CO_Emergency.h"
#include "CO_NMT_Heartbeat.h"

/*
 * Read received message from CAN module.
 *
 * Function will be called (by CAN receive interrupt) every time, when CAN
 * message with correct identifier will be received. For more information and
 * description of parameters see file CO_driver.h.
 */
static void CO_NMT_receive(void *object, const CO_CANrxMsg_t *msg){
    CO_NMT_t *NMT;
    uint8_t nodeId;

    NMT = (CO_NMT_t*)object;   /* this is the correct pointer type of the first argument */

    nodeId = msg->data[1];

    if((msg->DLC == 2) && ((nodeId == 0) || (nodeId == NMT->nodeId))){
        uint8_t command = msg->data[0];

        switch(command){
            case CO_NMT_ENTER_OPERATIONAL:
                if((*NMT->emPr->errorRegister) == 0U){
                    NMT->operatingState = CO_NMT_OPERATIONAL;
                }
                break;
            case CO_NMT_ENTER_STOPPED:
                NMT->operatingState = CO_NMT_STOPPED;
                break;
            case CO_NMT_ENTER_PRE_OPERATIONAL:
                NMT->operatingState = CO_NMT_PRE_OPERATIONAL;
                break;
            case CO_NMT_RESET_NODE:
                NMT->resetCommand = CO_RESET_APP;
                break;
            case CO_NMT_RESET_COMMUNICATION:
                NMT->resetCommand = CO_RESET_COMM;
                break;
        }
    }
}


/******************************************************************************/
int16_t CO_NMT_init(
        CO_NMT_t               *NMT,
        CO_EMpr_t              *emPr,
        uint8_t                 nodeId,
        uint16_t                firstHBTime,
        CO_CANmodule_t         *NMT_CANdev,
        uint16_t                NMT_rxIdx,
        uint16_t                CANidRxNMT,
        CO_CANmodule_t         *HB_CANdev,
        uint16_t                HB_txIdx,
        uint16_t                CANidTxHB)
{

    /* blinking bytes */
    NMT->LEDflickering          = 0;
    NMT->LEDblinking            = 0;
    NMT->LEDsingleFlash         = 0;
    NMT->LEDdoubleFlash         = 0;
    NMT->LEDtripleFlash         = 0;
    NMT->LEDquadrupleFlash      = 0;

    /* Configure object variables */
    NMT->operatingState         = CO_NMT_INITIALIZING;
    NMT->LEDgreenRun            = -1;
    NMT->LEDredError            = 1;
    NMT->nodeId                 = nodeId;
    NMT->firstHBTime            = firstHBTime;
    NMT->resetCommand           = 0;
    NMT->HBproducerTimer        = 0xFFFF;
    NMT->emPr                   = emPr;

    /* configure NMT CAN reception */
    CO_CANrxBufferInit(
            NMT_CANdev,         /* CAN device */
            NMT_rxIdx,          /* rx buffer index */
            CANidRxNMT,         /* CAN identifier */
            0x7FF,              /* mask */
            0,                  /* rtr */
            (void*)NMT,         /* object passed to receive function */
            CO_NMT_receive);    /* this function will process received message */

    /* configure HB CAN transmission */
    NMT->HB_CANdev = HB_CANdev;
    NMT->HB_TXbuff = CO_CANtxBufferInit(
            HB_CANdev,          /* CAN device */
            HB_txIdx,           /* index of specific buffer inside CAN module */
            CANidTxHB,          /* CAN identifier */
            0,                  /* rtr */
            1,                  /* number of data bytes */
            0);                 /* synchronous message flag bit */

    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_NMT_blinkingProcess50ms(CO_NMT_t *NMT){

    if(++NMT->LEDflickering >= 1) NMT->LEDflickering = -1;

    if(++NMT->LEDblinking >= 4) NMT->LEDblinking = -4;

    if(++NMT->LEDsingleFlash >= 4) NMT->LEDsingleFlash = -20;

    switch(++NMT->LEDdoubleFlash){
        case    4:  NMT->LEDdoubleFlash = -104; break;
        case -100:  NMT->LEDdoubleFlash =  100; break;
        case  104:  NMT->LEDdoubleFlash =  -20; break;
    }

    switch(++NMT->LEDtripleFlash){
        case    4:  NMT->LEDtripleFlash = -104; break;
        case -100:  NMT->LEDtripleFlash =  100; break;
        case  104:  NMT->LEDtripleFlash = -114; break;
        case -110:  NMT->LEDtripleFlash =  110; break;
        case  114:  NMT->LEDtripleFlash =  -20; break;
    }

    switch(++NMT->LEDquadrupleFlash){
        case    4:  NMT->LEDquadrupleFlash = -104; break;
        case -100:  NMT->LEDquadrupleFlash =  100; break;
        case  104:  NMT->LEDquadrupleFlash = -114; break;
        case -110:  NMT->LEDquadrupleFlash =  110; break;
        case  114:  NMT->LEDquadrupleFlash = -124; break;
        case -120:  NMT->LEDquadrupleFlash =  120; break;
        case  124:  NMT->LEDquadrupleFlash =  -20; break;
    }
}


/******************************************************************************/
CO_NMT_reset_cmd_t CO_NMT_process(
        CO_NMT_t               *NMT,
        uint16_t                timeDifference_ms,
        uint16_t                HBtime,
        uint32_t                NMTstartup,
        uint8_t                 errorRegister,
        const uint8_t           errorBehavior[])
{
    uint8_t CANpassive;

    NMT->HBproducerTimer += timeDifference_ms;

    /* Heartbeat producer message & Bootup message */
    if((HBtime && NMT->HBproducerTimer >= HBtime) || NMT->operatingState == CO_NMT_INITIALIZING){

        NMT->HBproducerTimer = NMT->HBproducerTimer - HBtime;

        NMT->HB_TXbuff->data[0] = NMT->operatingState;
        CO_CANsend(NMT->HB_CANdev, NMT->HB_TXbuff);

        if(NMT->operatingState == CO_NMT_INITIALIZING){
            if(HBtime > NMT->firstHBTime) NMT->HBproducerTimer = HBtime - NMT->firstHBTime;

            if((NMTstartup & 0x04) == 0) NMT->operatingState = CO_NMT_OPERATIONAL;
            else                         NMT->operatingState = CO_NMT_PRE_OPERATIONAL;
        }
    }


    /* CAN passive flag */
    CANpassive = 0;
    if(CO_isError(NMT->emPr->em, CO_EM_CAN_TX_BUS_PASSIVE) || CO_isError(NMT->emPr->em, CO_EM_CAN_RX_BUS_PASSIVE))
        CANpassive = 1;


    /* CANopen green RUN LED (DR 303-3) */
    switch(NMT->operatingState){
        case CO_NMT_STOPPED:          NMT->LEDgreenRun = NMT->LEDsingleFlash;   break;
        case CO_NMT_PRE_OPERATIONAL:  NMT->LEDgreenRun = NMT->LEDblinking;      break;
        case CO_NMT_OPERATIONAL:      NMT->LEDgreenRun = 1;                     break;
    }


    /* CANopen red ERROR LED (DR 303-3) */
    if(CO_isError(NMT->emPr->em, CO_EM_CAN_TX_BUS_OFF))
        NMT->LEDredError = 1;

    else if(CO_isError(NMT->emPr->em, CO_EM_SYNC_TIME_OUT))
        NMT->LEDredError = NMT->LEDtripleFlash;

    else if(CO_isError(NMT->emPr->em, CO_EM_HEARTBEAT_CONSUMER) || CO_isError(NMT->emPr->em, CO_EM_HB_CONSUMER_REMOTE_RESET))
        NMT->LEDredError = NMT->LEDdoubleFlash;

    else if(CANpassive || CO_isError(NMT->emPr->em, CO_EM_CAN_BUS_WARNING))
        NMT->LEDredError = NMT->LEDsingleFlash;

    else if(errorRegister)
        NMT->LEDredError = (NMT->LEDblinking>=0)?-1:1;

    else
        NMT->LEDredError = -1;


    /* in case of error enter pre-operational state */
    if(errorBehavior && (NMT->operatingState == CO_NMT_OPERATIONAL)){
        if(CANpassive && (errorBehavior[2] == 0 || errorBehavior[2] == 2)) errorRegister |= 0x10;

        if(errorRegister){
            /* Communication error */
            if(errorRegister & CO_ERR_REG_COMM_ERR){
                if(errorBehavior[1] == 0){
                  NMT->operatingState = CO_NMT_PRE_OPERATIONAL;
                }
                else if(errorBehavior[1] == 2){
                    NMT->operatingState = CO_NMT_STOPPED;
                }
                else if(CO_isError(NMT->emPr->em, CO_EM_CAN_TX_BUS_OFF)
                     || CO_isError(NMT->emPr->em, CO_EM_HEARTBEAT_CONSUMER)
                     || CO_isError(NMT->emPr->em, CO_EM_HB_CONSUMER_REMOTE_RESET))
                {
                    if(errorBehavior[0] == 0){
                        NMT->operatingState = CO_NMT_PRE_OPERATIONAL;
                    }
                    else if(errorBehavior[0] == 2){
                        NMT->operatingState = CO_NMT_STOPPED;
                    }
                }
            }

            /* Generic error */
            if(errorRegister & CO_ERR_REG_GENERIC_ERR){
                if      (errorBehavior[3] == 0) NMT->operatingState = CO_NMT_PRE_OPERATIONAL;
                else if (errorBehavior[3] == 2) NMT->operatingState = CO_NMT_STOPPED;
            }

            /* Device profile error */
            if(errorRegister & CO_ERR_REG_DEV_PROFILE){
                if      (errorBehavior[4] == 0) NMT->operatingState = CO_NMT_PRE_OPERATIONAL;
                else if (errorBehavior[4] == 2) NMT->operatingState = CO_NMT_STOPPED;
            }

            /* Manufacturer specific error */
            if(errorRegister & CO_ERR_REG_MANUFACTURER){
                if      (errorBehavior[5] == 0) NMT->operatingState = CO_NMT_PRE_OPERATIONAL;
                else if (errorBehavior[5] == 2) NMT->operatingState = CO_NMT_STOPPED;
            }

            /* if operational state is lost, send HB immediatelly. */
            if(NMT->operatingState != CO_NMT_OPERATIONAL)
                NMT->HBproducerTimer = HBtime;
        }
    }


    return NMT->resetCommand;
}
