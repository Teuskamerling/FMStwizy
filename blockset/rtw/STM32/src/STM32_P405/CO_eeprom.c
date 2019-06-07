/**
 * Microcontroller specific code for CANopenNode nonvolatile variables.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_eeprom.c
 * @ingroup     CO_eeprom
 * @version     SVN: \$Id: eeprom.c 46 2013-08-24 09:18:16Z jani22 $
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
#include "CO_eeprom.h"
#include "crc16-ccitt.h"
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */
#include "eeprom_sim.h"                               /* simulated EEPROM  driver      */


/**
 * Macros to define how the CANopen EEPROM is mapped to the STM32F4xx simulated EEPROM.
 * The simulated EEPROM is 2kb in total. The first 512 bytes are reserved by eeprom.c
 * and the last 768 bytes is reserved by errorcodes.c. This leaves a total of 768 bytes
 * starting at 0x200 in simulated EEPROM for the CANopen EEPROM. This is divided up into
 * a space for storing OD_ROM (including checksum) and a space for storing OD_EEPROM.
 */
#define CO_EE_OD_ROM_CRC_BASE_ADDRESS     (0x200)
#define CO_EE_OD_ROM_CRC_SIZE             (2)
#define CO_EE_OD_ROM_BASE_ADDRESS         (CO_EE_OD_ROM_CRC_BASE_ADDRESS + CO_EE_OD_ROM_CRC_SIZE)
#define CO_EE_OD_ROM_SIZE                 (702)
#define CO_EE_OD_EEPROM_BASE_ADDRESS      (CO_EE_OD_ROM_BASE_ADDRESS + CO_EE_OD_ROM_SIZE)
#define CO_EE_OD_EEPROM_SIZE              (64)



/**
 * OD function for accessing _Store parameters_ (index 0x1010) from SDO server.
 *
 * For more information see file CO_SDO.h.
 */
static CO_SDO_abortCode_t CO_ODF_1010(CO_ODF_arg_t *ODF_arg);
static CO_SDO_abortCode_t CO_ODF_1010(CO_ODF_arg_t *ODF_arg){
    CO_EE_t *ee;
    uint32_t value;
    CO_SDO_abortCode_t ret = CO_SDO_AB_NONE;
    uint16_t cnt;
    uint16_t current_write_address;
    uint16_t CRCbuf;

    ee = (CO_EE_t*) ODF_arg->object;
    value = CO_getUint32(ODF_arg->data);

    if(!ODF_arg->reading){
        /* don't change the old value */
        CO_memcpy(ODF_arg->data, (const uint8_t*)ODF_arg->ODdataStorage, 4U);

        if(ODF_arg->subIndex == 1U){
            if(value == 0x65766173UL){
                /* check if all OD_ROM data actually fits in the simulated EEPROM */
                if (ee->OD_ROMSize > CO_EE_OD_ROM_SIZE) {
                    ret = CO_SDO_AB_HW;
                }
                else {
                    /* write ee->OD_ROMAddress, ee->OD_ROMSize to eeprom (blocking function) */
                    current_write_address = CO_EE_OD_ROM_BASE_ADDRESS;
                    for (cnt=0; cnt < ee->OD_ROMSize; cnt++) {
                        EepromSimUINT8Set(current_write_address, ee->OD_ROMAddress[cnt]);
                        current_write_address++;
                    }
                    /* calculate and write the checksum */
                    CRCbuf = crc16_ccitt((uint8_t*)ee->OD_ROMAddress, ee->OD_ROMSize, 0);
                    EepromSimUINT16Set(CO_EE_OD_ROM_CRC_BASE_ADDRESS, CRCbuf);
                    /* synchronize the EEPROM to make sure it is non-volatile */
                    EepromSimSynchronize();
                }
                  
            }
            else{
                ret = CO_SDO_AB_DATA_TRANSF;
            }
        }
    }

    return ret;
}


/**
 * OD function for accessing _Restore default parameters_ (index 0x1011) from SDO server.
 *
 * For more information see file CO_SDO.h.
 */
static CO_SDO_abortCode_t CO_ODF_1011(CO_ODF_arg_t *ODF_arg);
static CO_SDO_abortCode_t CO_ODF_1011(CO_ODF_arg_t *ODF_arg){
    CO_EE_t *ee;
    uint32_t value;
    CO_SDO_abortCode_t ret = CO_SDO_AB_NONE;

    ee = (CO_EE_t*) ODF_arg->object;
    value = CO_getUint32(ODF_arg->data);

    if(!ODF_arg->reading){
        /* don't change the old value */
        CO_memcpy(ODF_arg->data, (const uint8_t*)ODF_arg->ODdataStorage, 4U);

        if(ODF_arg->subIndex >= 1U){
            if(value == 0x64616F6CUL){
                /* Clear the eeprom */
                EepromSimErase(CO_EE_OD_ROM_BASE_ADDRESS, ee->OD_ROMSize);
                /* synchronize the EEPROM to make sure it is non-volatile */
                EepromSimSynchronize();
                
            }
            else{
                ret = CO_SDO_AB_DATA_TRANSF;
            }
        }
    }

    return ret;
}


/******************************************************************************/
CO_ReturnError_t CO_EE_init_1(
        CO_EE_t                *ee,
        uint8_t                *OD_EEPROMAddress,
        uint32_t                OD_EEPROMSize,
        uint8_t                *OD_ROMAddress,
        uint32_t                OD_ROMSize)
{
    uint32_t firstWordRAM, firstWordEE, lastWordEE;
    uint16_t cnt;
    uint16_t CRCread, CRCcalculated;

    /* configure object variables */
    ee->OD_EEPROMAddress = OD_EEPROMAddress;
    ee->OD_EEPROMSize = OD_EEPROMSize;
    ee->OD_ROMAddress = OD_ROMAddress;
    ee->OD_ROMSize = OD_ROMSize;
    ee->OD_EEPROMCurrentIndex = 0U;
    ee->OD_EEPROMWriteEnable = CO_false;

    /* check sizes */
    if ( (ee->OD_EEPROMSize > CO_EE_OD_EEPROM_SIZE) || (ee->OD_ROMSize > CO_EE_OD_ROM_SIZE) ) {
        /* either the OD_ROM or OD_EEPROM tables are to large for the simulated EEPROM */
        return CO_ERROR_OUT_OF_MEMORY;
    }
    
    /* read the CO_OD_EEPROM from EEPROM, first verify, if data are OK */
    firstWordRAM = *(ee->OD_EEPROMAddress);
    firstWordEE = EepromSimUINT32Get(CO_EE_OD_EEPROM_BASE_ADDRESS);
    lastWordEE = EepromSimUINT32Get(CO_EE_OD_EEPROM_BASE_ADDRESS + ee->OD_EEPROMSize - 4);
    if(firstWordRAM == firstWordEE && firstWordRAM == lastWordEE){
        for(cnt=0; cnt<ee->OD_EEPROMSize; cnt++)
            (ee->OD_EEPROMAddress)[cnt] = EepromSimUINT8Get(CO_EE_OD_EEPROM_BASE_ADDRESS + cnt);
    }
    ee->OD_EEPROMWriteEnable = CO_true;
    
    /* read the CRC from EEPROM */
    CRCread = EepromSimUINT16Get(CO_EE_OD_ROM_CRC_BASE_ADDRESS);
    /* calculate the CRC as stored in EEPROM */
    CRCcalculated = crc16_ccitt((uint8_t*)EepromSimGetPtr(CO_EE_OD_ROM_BASE_ADDRESS), ee->OD_ROMSize, 0);
    /* verify CRC to determine if OD_ROM data stored in EEPROM is actually valid */
    if (CRCread == CRCcalculated) {
        /* continue by copying data into object dictionary */
        for (cnt=0; cnt<ee->OD_ROMSize; cnt++) {
            (ee->OD_ROMAddress)[cnt] = EepromSimUINT8Get(CO_EE_OD_ROM_BASE_ADDRESS + cnt);
        }
    }
    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_EE_init_2(
        CO_EE_t                *ee,
        CO_ReturnError_t        eeStatus,
        CO_SDO_t               *SDO,
        CO_EM_t                *em)
{
    CO_OD_configure(SDO, OD_H1010_STORE_PARAM_FUNC, CO_ODF_1010, (void*)ee, 0, 0U);
    CO_OD_configure(SDO, OD_H1011_REST_PARAM_FUNC, CO_ODF_1011, (void*)ee, 0, 0U);
    if(eeStatus != CO_ERROR_NO){
        CO_errorReport(em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, (uint32_t)eeStatus);
    }
}


/******************************************************************************/
void CO_EE_process(CO_EE_t *ee){
    if((ee != 0) && (ee->OD_EEPROMWriteEnable)/* && !isWriteInProcess()*/){
        uint32_t i;
        uint8_t RAMdata, eeData;

        /* verify next word */
        if(++ee->OD_EEPROMCurrentIndex == ee->OD_EEPROMSize){
            ee->OD_EEPROMCurrentIndex = 0U;
            /* full EEPROM data check and updated. now make sure to commit changes
             * to non volatile memory.
             */
            EepromSimSynchronize();
        }
        i = ee->OD_EEPROMCurrentIndex;

        /* read eeprom */
        RAMdata = ee->OD_EEPROMAddress[i];
        eeData = EepromSimUINT8Get(CO_EE_OD_EEPROM_BASE_ADDRESS + i);

        /* if bytes in EEPROM and in RAM are different, then write to EEPROM */
        if(eeData != RAMdata){
          EepromSimUINT8Set(CO_EE_OD_EEPROM_BASE_ADDRESS + i, RAMdata);
        }
    }
}
