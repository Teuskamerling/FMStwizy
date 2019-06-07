/************************************************************************************//**
* \file         eeprom.c
* \brief        EEPROM driver source file.
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
#include "eeprom.h"                                   /* EEPROM driver                 */
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */
#include "eeprom_sim.h"                               /* simulated EEPROM  driver      */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Maximum identifier value for a EEPROM parameter. */
#define EEPROM_PARAM_ID_MAX    (255)

/** \brief For storing data, 512 bytes of simulated EEPROM is reserved at the start of
 *         the 2kb simulated EEPROM module. This defines the start address.
 */
#define EEPROM_START_ADDRESS (0x000)

/** \brief The number of bytes reserved for the EEPROM module. */
#define EEPROM_SIZE          (512)


/************************************************************************************//**
** \brief     Initializes the EEPROM driver.
** \return    none.
**
****************************************************************************************/
void EepromInit(void)
{
  /* initialize the simulated EEPROM driver */
  EepromSimInit();
} /*** end of EepromInit ***/


/************************************************************************************//**
** \brief     Stores a parameter value with the specified identifier in EEPROM.
** \param     id Identifier of the parameter. Can be 0..EEPROM_PARAM_ID_MAX.
** \param     value Parameter value to store in EEPROM.
** \return    none.
**
****************************************************************************************/
void EepromWrite(uint8_t id, uint16_t value)
{
  uint16_t baseAddr;

  /* return if the identifier is not in the valid range */
  if (id > EEPROM_PARAM_ID_MAX)
  {
    return;
  }
  /* determine the base address */
  baseAddr = EEPROM_START_ADDRESS + (id * sizeof(uint16_t));
  /* write the identifier value */
  EepromSimUINT16Set(baseAddr, value);
} /*** end of EepromWrite ***/


/************************************************************************************//**
** \brief     Reads a parameter value with the specified identifier from EEPROM.
** \param     id Identifier of the parameter. Can be 0..EEPROM_PARAM_ID_MAX.
** \return    The parameter value or 0 if the identifier is not valid.
**
****************************************************************************************/
uint16_t EepromRead(uint8_t id)
{
  uint16_t baseAddr;

  /* return 0 if the identifier is not in the valid range */
  if (id > EEPROM_PARAM_ID_MAX)
  {
    return 0;
  }
  /* determine the base address */
  baseAddr = EEPROM_START_ADDRESS + (id * sizeof(uint16_t));
  /* read and return the identifier value */
  return EepromSimUINT16Get(baseAddr);
} /*** end of EepromRead ***/


/************************************************************************************//**
** \brief     Erases all stored parameters from EEPROM.
** \return    none.
**
****************************************************************************************/
void EepromErase(void)
{
  /* erase the data in the part of the simulated EEPROM */
  EepromSimErase(EEPROM_START_ADDRESS, EEPROM_SIZE);
} /*** end of EepromErase ***/


/************************************************************************************//**
** \brief     This module is based on the simulated EEPROM module because the target
**            does not have dedicated EEPROM. This means all EEPROM reads and writes
**            operate on a RAM buffer. Use this function to store the RAM buffer contents
**            in non-volatile flash memory so that the data is still available after
**            power-off.
** \return    none.
**
****************************************************************************************/
void EepromSave(void)
{
  EepromSimSynchronize();
}/*** end of EepromSave ***/


/************************************ end of eeprom.c **********************************/


