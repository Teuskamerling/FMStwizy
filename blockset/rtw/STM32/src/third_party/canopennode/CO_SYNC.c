/*
 * CANopen SYNC object.
 *
 * @file        CO_SYNC.c
 * @ingroup     CO_SYNC
 * @version     SVN: \$Id: CO_SYNC.c 46 2013-08-24 09:18:16Z jani22 $
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
#include "CO_SYNC.h"


/*
 * Read received message from CAN module.
 *
 * Function will be called (by CAN receive interrupt) every time, when CAN
 * message with correct identifier will be received. For more information and
 * description of parameters see file CO_driver.h.
 */
static void CO_SYNC_receive(void *object, const CO_CANrxMsg_t *msg){
    CO_SYNC_t *SYNC;
    uint8_t operState;
    CO_bool_t err = CO_false;

    SYNC = (CO_SYNC_t*)object;   /* this is the correct pointer type of the first argument */
    operState = *SYNC->operatingState;

    if((operState == CO_NMT_OPERATIONAL) || (operState == CO_NMT_PRE_OPERATIONAL)){
        if(SYNC->counterOverflowValue != 0){
            if(msg->DLC != 1U){
                SYNC->receiveError = (uint16_t)msg->DLC | 0x0100U;
                err = CO_true;
            }
            else{
                SYNC->counter = msg->data[0];
            }
        }
        else{
            if(msg->DLC != 0U){
                SYNC->receiveError = (uint16_t)msg->DLC | 0x0200U;
                err = CO_true;
            }
        }

        if(!err){
            if(operState == CO_NMT_OPERATIONAL){
                SYNC->running = CO_true;
            }
            SYNC->timer = 0;
        }
    }
}


/*
 * Function for accessing _COB ID SYNC Message_ (index 0x1005) from SDO server.
 *
 * For more information see file CO_SDO.h.
 */
static CO_SDO_abortCode_t CO_ODF_1005(CO_ODF_arg_t *ODF_arg){
    CO_SYNC_t *SYNC;
    uint32_t value;
    CO_SDO_abortCode_t ret = CO_SDO_AB_NONE;

    SYNC = (CO_SYNC_t*) ODF_arg->object;
    value = CO_getUint32(ODF_arg->data);

    if(!ODF_arg->reading){
        uint8_t configureSyncProducer = 0;

        /* only 11-bit CAN identifier is supported */
        if(value & 0x20000000UL){
            ret = CO_SDO_AB_INVALID_VALUE;
        }
        else{
            /* is 'generate Sync messge' bit set? */
            if(value & 0x40000000UL){
                /* if bit was set before, value can not be changed */
                if(SYNC->isProducer){
                    ret = CO_SDO_AB_DATA_DEV_STATE;
                }
                else{
                    configureSyncProducer = 1;
                }
            }
        }

        /* configure sync producer and consumer */
        if(ret == CO_SDO_AB_NONE){
            SYNC->COB_ID = (uint16_t)(value & 0x7FFU);

            if(configureSyncProducer){
                uint8_t len = 0U;
                if(SYNC->counterOverflowValue != 0U){
                    len = 1U;
                    SYNC->counter = 0U;
                    SYNC->running = CO_false;
                    SYNC->timer = 0U;
                }
                SYNC->CANtxBuff = CO_CANtxBufferInit(
                        SYNC->CANdevTx,         /* CAN device */
                        SYNC->CANdevTxIdx,      /* index of specific buffer inside CAN module */
                        SYNC->COB_ID,           /* CAN identifier */
                        0,                      /* rtr */
                        len,                    /* number of data bytes */
                        0);                     /* synchronous message flag bit */
                SYNC->isProducer = CO_true;
            }
            else{
                SYNC->isProducer = CO_false;
            }

            CO_CANrxBufferInit(
                    SYNC->CANdevRx,         /* CAN device */
                    SYNC->CANdevRxIdx,      /* rx buffer index */
                    SYNC->COB_ID,           /* CAN identifier */
                    0x7FF,                  /* mask */
                    0,                      /* rtr */
                    (void*)SYNC,            /* object passed to receive function */
                    CO_SYNC_receive);       /* this function will process received message */
        }
    }

    return ret;
}


/*
 * Function for accessing _Communication cycle period_ (index 0x1006) from SDO server.
 *
 * For more information see file CO_SDO.h.
 */
static CO_SDO_abortCode_t CO_ODF_1006(CO_ODF_arg_t *ODF_arg){
    CO_SYNC_t *SYNC;
    uint32_t value;
    CO_SDO_abortCode_t ret = CO_SDO_AB_NONE;

    SYNC = (CO_SYNC_t*) ODF_arg->object;
    value = CO_getUint32(ODF_arg->data);

    if(!ODF_arg->reading){
        /* period transition from 0 to something */
        if((SYNC->periodTime == 0) && (value != 0)){
            SYNC->counter = 0;
        }

        SYNC->periodTime = value;
        SYNC->periodTimeoutTime = (value / 2U) * 3U;
        /* overflow? */
        if(SYNC->periodTimeoutTime < value){
            SYNC->periodTimeoutTime = 0xFFFFFFFFUL;
        }

        SYNC->running = CO_false;
        SYNC->timer = 0;
    }

    return ret;
}


/**
 * Function for accessing _Synchronous counter overflow value_ (index 0x1019) from SDO server.
 *
 * For more information see file CO_SDO.h.
 */
static CO_SDO_abortCode_t CO_ODF_1019(CO_ODF_arg_t *ODF_arg){
    CO_SYNC_t *SYNC;
    uint8_t value;
    CO_SDO_abortCode_t ret = CO_SDO_AB_NONE;

    SYNC = (CO_SYNC_t*) ODF_arg->object;
    value = ODF_arg->data[0];

    if(!ODF_arg->reading){
        uint8_t len = 0U;

        if(SYNC->periodTime){
            ret = CO_SDO_AB_DATA_DEV_STATE;
        }
        else{
            SYNC->counterOverflowValue = value;
            if(value != 0){
                len = 1U;
            }

            SYNC->CANtxBuff = CO_CANtxBufferInit(
                    SYNC->CANdevTx,         /* CAN device */
                    SYNC->CANdevTxIdx,      /* index of specific buffer inside CAN module */
                    SYNC->COB_ID,           /* CAN identifier */
                    0,                      /* rtr */
                    len,                    /* number of data bytes */
                    0);                     /* synchronous message flag bit */
        }
    }

    return ret;
}


/******************************************************************************/
int16_t CO_SYNC_init(
        CO_SYNC_t              *SYNC,
        CO_EM_t                *em,
        CO_SDO_t               *SDO,
        uint8_t                *operatingState,
        uint32_t                COB_ID_SYNCMessage,
        uint32_t                communicationCyclePeriod,
        uint8_t                 synchronousCounterOverflowValue,
        CO_CANmodule_t         *CANdevRx,
        uint16_t                CANdevRxIdx,
        CO_CANmodule_t         *CANdevTx,
        uint16_t                CANdevTxIdx)
{
    uint8_t len = 0;

    /* Configure object variables */
    SYNC->isProducer = (COB_ID_SYNCMessage&0x40000000L) ? CO_true : CO_false;
    SYNC->COB_ID = COB_ID_SYNCMessage&0x7FF;

    SYNC->periodTime = communicationCyclePeriod;
    SYNC->periodTimeoutTime = communicationCyclePeriod / 2 * 3;
    /* overflow? */
    if(SYNC->periodTimeoutTime < communicationCyclePeriod) SYNC->periodTimeoutTime = 0xFFFFFFFFL;

    SYNC->counterOverflowValue = synchronousCounterOverflowValue;
    if(synchronousCounterOverflowValue) len = 1;

    SYNC->curentSyncTimeIsInsideWindow = CO_true;

    SYNC->running = CO_false;
    SYNC->timer = 0;
    SYNC->counter = 0;
    SYNC->receiveError = 0U;

    SYNC->em = em;
    SYNC->operatingState = operatingState;
    SYNC->CANdevRx = CANdevRx;
    SYNC->CANdevRxIdx = CANdevRxIdx;

    /* Configure Object dictionary entry at index 0x1005, 0x1006 and 0x1019 */
    CO_OD_configure(SDO, OD_H1005_COBID_SYNC,        CO_ODF_1005, (void*)SYNC, 0, 0);
    CO_OD_configure(SDO, OD_H1006_COMM_CYCL_PERIOD,  CO_ODF_1006, (void*)SYNC, 0, 0);
    CO_OD_configure(SDO, OD_H1019_SYNC_CNT_OVERFLOW, CO_ODF_1019, (void*)SYNC, 0, 0);

    /* configure SYNC CAN reception */
    CO_CANrxBufferInit(
            CANdevRx,               /* CAN device */
            CANdevRxIdx,            /* rx buffer index */
            SYNC->COB_ID,           /* CAN identifier */
            0x7FF,                  /* mask */
            0,                      /* rtr */
            (void*)SYNC,            /* object passed to receive function */
            CO_SYNC_receive);       /* this function will process received message */

    /* configure SYNC CAN transmission */
    SYNC->CANdevTx = CANdevTx;
    SYNC->CANdevTxIdx = CANdevTxIdx;
    SYNC->CANtxBuff = CO_CANtxBufferInit(
            CANdevTx,               /* CAN device */
            CANdevTxIdx,            /* index of specific buffer inside CAN module */
            SYNC->COB_ID,           /* CAN identifier */
            0,                      /* rtr */
            len,                    /* number of data bytes */
            0);                     /* synchronous message flag bit */

    return CO_ERROR_NO;
}


/******************************************************************************/
uint8_t CO_SYNC_process(
        CO_SYNC_t              *SYNC,
        uint32_t                timeDifference_us,
        uint32_t                ObjDict_synchronousWindowLength)
{
    uint8_t ret = 0;
    uint32_t timerNew;

    if(*SYNC->operatingState == CO_NMT_OPERATIONAL || *SYNC->operatingState == CO_NMT_PRE_OPERATIONAL){
        /* was SYNC just received */
        if(SYNC->running && SYNC->timer == 0)
            ret = 1;

        /* update sync timer, no overflow */
        CO_DISABLE_INTERRUPTS();
        timerNew = SYNC->timer + timeDifference_us;
        if(timerNew > SYNC->timer) SYNC->timer = timerNew;
        CO_ENABLE_INTERRUPTS();

        /* SYNC producer */
        if(SYNC->isProducer && SYNC->periodTime){
            if(SYNC->timer >= SYNC->periodTime){
                if(++SYNC->counter > SYNC->counterOverflowValue) SYNC->counter = 1;
                SYNC->running = CO_true;
                SYNC->timer = 0;
                SYNC->CANtxBuff->data[0] = SYNC->counter;
                CO_CANsend(SYNC->CANdevTx, SYNC->CANtxBuff);
                ret = 1;
            }
        }

        /* Synchronous PDOs are allowed only inside time window */
        if(ObjDict_synchronousWindowLength){
            if(SYNC->timer > ObjDict_synchronousWindowLength){
                if(SYNC->curentSyncTimeIsInsideWindow){
                    ret = 2;
                }
                SYNC->curentSyncTimeIsInsideWindow = CO_false;
            }
            else{
                SYNC->curentSyncTimeIsInsideWindow = CO_true;
            }
        }
        else{
            SYNC->curentSyncTimeIsInsideWindow = CO_true;
        }

        /* Verify timeout of SYNC */
        if(SYNC->periodTime && SYNC->timer > SYNC->periodTimeoutTime && *SYNC->operatingState == CO_NMT_OPERATIONAL)
            CO_errorReport(SYNC->em, CO_EM_SYNC_TIME_OUT, CO_EMC_COMMUNICATION, SYNC->timer);
    }

    if(*SYNC->operatingState != CO_NMT_OPERATIONAL){
        SYNC->running = CO_false;
    }

    /* verify error from receive function */
    if(SYNC->receiveError != 0U){
        CO_errorReport(SYNC->em, CO_EM_SYNC_LENGTH, CO_EMC_SYNC_DATA_LENGTH, (uint32_t)SYNC->receiveError);
        SYNC->receiveError = 0U;
    }

    return ret;
}
