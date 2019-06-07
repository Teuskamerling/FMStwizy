/************************************************************************************//**
* \file         eeprom_sim.c
* \brief        Simulated EEPROM driver source file.
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
#include "eeprom_sim.h"                               /* simulated EEPROM  driver      */
#include "os.h"                                       /* for operating system          */
#include "stm32f10x.h"                                /* STM32 registers               */
#include "stm32f10x_conf.h"                           /* STM32 peripheral drivers      */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief The simulated EEPROM is mapped to flash. This macro defines its start address
 *         in flash.
 */
#define EEPROMSIM_FLASH_BASE_ADDRESS   (0x0801D800)

/** \brief The total size of the simulated EEPROM in bytes. */
#define EEPROMSIM_TOTAL_SIZE           (2048)

/** \brief Flash page size. This is also the minimal erase size. Note that this is valid
 *         for STM32 devices with 128kb flash EEPROM.
 */
#define EEPROMSIM_FLASH_PAGE_SIZE     (0x400)

/** \brief The number of pages in the reserved flash EEPROM for the error codes. */
#define EEPROMSIM_FLASH_NUM_PAGES     (EEPROMSIM_TOTAL_SIZE / EEPROMSIM_FLASH_PAGE_SIZE)


/****************************************************************************************
* Local data declarations
****************************************************************************************/
static uint8_t eeprom_overlay_buffer[EEPROMSIM_TOTAL_SIZE];
static uint8_t eeprom_out_of_sync;


/************************************************************************************//**
** \brief     Initializes the simulated EEPROM driver.
** \return    none.
**
****************************************************************************************/
void EepromSimInit(void)
{
  uint8_t *src, *dst;
  uint16_t cnt;

  /* set source and destination pointers */
  src = (uint8_t *)EEPROMSIM_FLASH_BASE_ADDRESS;
  dst = (uint8_t *)&eeprom_overlay_buffer[0];
  /* copy flash EEPROM contents to RAM */
  for (cnt=0; cnt<EEPROMSIM_TOTAL_SIZE; cnt++)
  {
    *dst = *src;
    dst++;
    src++;
  }
  /* RAM and flash EEPROM are now synchronized */
  eeprom_out_of_sync = FALSE;
} /*** end of EepromSimInit ***/


/************************************************************************************//**
** \brief     Synchronized the values in the RAM buffer to flash memory. During read,
**            write and erase operations the data is only updated in the RAM buffer.
**            These changes will be lost upon power off. At one point in time, ideally
**            before power off, the changes should be stored in flash memory. This is
**            achieved by calling this function.
** \return    none.
**
****************************************************************************************/
void EepromSimSynchronize(void)
{
  uint8_t pageCounter;
  volatile FLASH_Status flashStatus = FLASH_COMPLETE;
  uint16_t  wordCnt;
  uint32_t programBaseAddr;
  uint32_t *programDataPtr;
  uint32_t saved_cs_state;

  /* check if there is a difference between RAM and flash EEPROM */
  if (eeprom_out_of_sync == FALSE)
  {
    /* no need to update the flash sector */
    return;
  }

  /* enter critical section to make sure flash operation is not interrupted */
  saved_cs_state = OsEnterCriticalSection();
  /* unlock the flash bank1 program erase controller */
  FLASH_UnlockBank1();

  /* clear all pending flags */
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

  /* erase the FLASH pages */
  for (pageCounter = 0; (pageCounter < EEPROMSIM_FLASH_NUM_PAGES) && (flashStatus == FLASH_COMPLETE); pageCounter++)
  {
    flashStatus = FLASH_ErasePage(EEPROMSIM_FLASH_BASE_ADDRESS + (EEPROMSIM_FLASH_PAGE_SIZE * pageCounter));
  }

  /* clear all pending flags */
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

  /* determine base address */
  programBaseAddr = EEPROMSIM_FLASH_BASE_ADDRESS;

  /* program the data word-by-word */
  for (wordCnt=0; wordCnt<(EEPROMSIM_TOTAL_SIZE/sizeof(uint32_t)); wordCnt++)
  {
    programDataPtr = (uint32_t *)(&eeprom_overlay_buffer[wordCnt*sizeof(uint32_t)]);
    FLASH_ProgramWord(programBaseAddr, *programDataPtr);
    programBaseAddr += sizeof(uint32_t);
  }

  /* lock the flash bank1 program erase controller */
  FLASH_LockBank1();
  /* leave critical section */
  OsLeaveCriticalSection(saved_cs_state);

  /* RAM and flash EEPROM are now synchronized */
  eeprom_out_of_sync = FALSE;
} /*** end of EepromSimSynchronize ***/


/************************************************************************************//**
** \brief     Erases the simulated EEPROM by setting all values in the RAM buffer to
**            0xff, which is the default value of an erased EEPROM byte. Use function
**            EepromSimSynchronize to actually process the changes in non-volatile flash.
** \return    none.
**
****************************************************************************************/
void EepromSimBulkErase(void)
{
  uint16_t cnt;

  /* sets all bytes to the default EEPROM erased value */
  for (cnt=0; cnt<EEPROMSIM_TOTAL_SIZE; cnt++)
  {
    /* keep track of dirty flag */
    if (eeprom_overlay_buffer[cnt] != 0xff)
    {
      eeprom_overlay_buffer[cnt] = 0xff;
      /* RAM buffer and flash are now out of sync */
      eeprom_out_of_sync = TRUE;
    }
  }
} /*** end of EepromSimBulkErase ***/


/************************************************************************************//**
** \brief     Obtains a pointer to the EEPROM data at the specified address.
** \param     address Memory address to get the pointer from
** \return    The requested pointer if successfull, otherwise NULL.
**
****************************************************************************************/
uint8_t *EepromSimGetPtr(uint16_t address)
{
    /* check the address */
  if (address >= EEPROMSIM_TOTAL_SIZE)
  {
    /* invalid address, cannot continue */
    return NULL;
  }
  return &eeprom_overlay_buffer[address];
} /*** end of EepromSimGetPtr ***/


/************************************************************************************//**
** \brief     Erases a part of the simulated EEPROM by setting the values in the RAM
**            buffer to 0xff, which is the default value of an erased EEPROM byte. Use
**            function EepromSimSynchronize to actually process the changes in
**            non-volatile flash.
** \param     address Memory pointer to read from. Must be a valid value in the range
**                    0 and EEPROMSIM_TOTAL_SIZE.
** \param     len Number of bytes to erase.
** \return    none.
**
****************************************************************************************/
void EepromSimErase(uint16_t address, uint16_t len)
{
  uint16_t idx;

  /* check the address */
  if (address >= (EEPROMSIM_TOTAL_SIZE - len) + 1)
  {
    /* invalid address, cannot continue */
    return;
  }

  /* sets bytes in memory range to the default EEPROM erased value */
  for (idx=address; idx<(address+len); idx++)
  {
    /* keep track of dirty flag */
    if (eeprom_overlay_buffer[idx] != 0xff)
    {
      eeprom_overlay_buffer[idx] = 0xff;
      /* RAM buffer and flash are now out of sync */
      eeprom_out_of_sync = TRUE;
    }
  }
} /*** end of EepromSimErase ***/


/************************************************************************************//**
** \brief     Reads an unsigned 8-bit value from the specified virtual memory address.
** \param     address Memory pointer to read from. Must be a valid value in the range
**                    0 and EEPROMSIM_TOTAL_SIZE.
** \return    the value read from the memory address.
**
****************************************************************************************/
uint8_t EepromSimUINT8Get(uint16_t address)
{
  uint8_t  data_val = 0;
  uint8_t *data_val_ptr;
  uint32_t saved_cs_state;

  /* check the address */
  if (address >= (EEPROMSIM_TOTAL_SIZE - sizeof(uint8_t)) + 1)
  {
	  /* invalid address, return a dummy value */
    return data_val;
  }
  /* start critical section */
  saved_cs_state = OsEnterCriticalSection();
  /* determine the pointer of the data to read */
  data_val_ptr = (uint8_t*)(&eeprom_overlay_buffer[address]);
  /* read the value from the RAM buffer */
  data_val = *data_val_ptr;
  /* end critical section */
  OsLeaveCriticalSection(saved_cs_state);
  /* return the results */
  return data_val;
} /*** end of EepromSimUINT8Get ***/


/************************************************************************************//**
** \brief     Reads a signed 8-bit value from the specified virtual memory address.
** \param     address Memory pointer to read from. Must be a valid value in the range
**                    0 and EEPROMSIM_TOTAL_SIZE.
** \return    the value read from the memory address.
**
****************************************************************************************/
int8_t EepromSimSINT8Get(uint16_t address)
{
  int8_t  data_val = 0;
  int8_t *data_val_ptr;
  uint32_t saved_cs_state;

  /* check the address */
  if (address >= (EEPROMSIM_TOTAL_SIZE - sizeof(int8_t)) + 1)
  {
	/* invalid address, return a dummy value */
    return data_val;
  }
  /* start critical section */
  saved_cs_state = OsEnterCriticalSection();
  /* determine the pointer of the data to read */
  data_val_ptr = (int8_t*)(&eeprom_overlay_buffer[address]);
  /* read the value from the RAM buffer */
  data_val = *data_val_ptr;
  /* end critical section */
  OsLeaveCriticalSection(saved_cs_state);
  /* return the results */
  return data_val;
} /*** end of EepromSimSINT8Get ***/


/************************************************************************************//**
** \brief     Reads an unsigned 16-bit value from the specified virtual memory address.
** \param     address Memory pointer to read from. Must be a valid value in the range
**                    0 and EEPROMSIM_TOTAL_SIZE.
** \return    the value read from the memory address.
**
****************************************************************************************/
uint16_t EepromSimUINT16Get(uint16_t address)
{
  uint16_t  data_val = 0;
  uint16_t *data_val_ptr;
  uint32_t saved_cs_state;

  /* check the address */
  if (address >= (EEPROMSIM_TOTAL_SIZE - sizeof(uint16_t)) + 1)
  {
	/* invalid address, return a dummy value */
    return data_val;
  }
  /* start critical section */
  saved_cs_state = OsEnterCriticalSection();
    /* determine the pointer of the data to read */
  data_val_ptr = (uint16_t*)(&eeprom_overlay_buffer[address]);
  /* read the value from the RAM buffer */
  data_val = *data_val_ptr;
  /* end critical section */
  OsLeaveCriticalSection(saved_cs_state);
  /* return the results */
  return data_val;
} /*** end of EepromSimUINT16Get ***/


/************************************************************************************//**
** \brief     Reads a signed 16-bit value from the specified virtual memory address.
** \param     address Memory pointer to read from. Must be a valid value in the range
**                    0 and EEPROMSIM_TOTAL_SIZE.
** \return    the value read from the memory address.
**
****************************************************************************************/
int16_t EepromSimSINT16Get(uint16_t address)
{
  int16_t  data_val = 0;
  int16_t *data_val_ptr;
  uint32_t saved_cs_state;

  /* check the address */
  if (address >= (EEPROMSIM_TOTAL_SIZE - sizeof(int16_t)) + 1)
  {
	/* invalid address, return a dummy value */
    return data_val;
  }
  /* start critical section */
  saved_cs_state = OsEnterCriticalSection();
  /* determine the pointer of the data to read */
  data_val_ptr = (int16_t*)(&eeprom_overlay_buffer[address]);
  /* read the value from the RAM buffer */
  data_val = *data_val_ptr;
  /* end critical section */
  OsLeaveCriticalSection(saved_cs_state);
  /* return the results */
  return data_val;
} /*** end of EepromSimSINT16Get ***/


/************************************************************************************//**
** \brief     Reads an unsigned 32-bit value from the specified virtual memory address.
** \param     address Memory pointer to read from. Must be a valid value in the range
**                    0 and EEPROMSIM_TOTAL_SIZE.
** \return    the value read from the memory address.
**
****************************************************************************************/
uint32_t EepromSimUINT32Get(uint16_t address)
{
  uint32_t  data_val = 0;
  uint32_t *data_val_ptr;
  uint32_t saved_cs_state;

  /* check the address */
  if (address >= (EEPROMSIM_TOTAL_SIZE - sizeof(uint32_t)) + 1)
  {
	/* invalid address, return a dummy value */
    return data_val;
  }
  /* start critical section */
  saved_cs_state = OsEnterCriticalSection();
  /* determine the pointer of the data to read */
  data_val_ptr = (uint32_t*)(&eeprom_overlay_buffer[address]);
  /* read the value from the RAM buffer */
  data_val = *data_val_ptr;
  /* end critical section */
  OsLeaveCriticalSection(saved_cs_state);
  /* return the results */
  return data_val;
} /*** end of EepromSimUINT32Get ***/


/************************************************************************************//**
** \brief     Reads a signed 32-bit value from the specified virtual memory address.
** \param     address Memory pointer to read from. Must be a valid value in the range
**                    0 and EEPROMSIM_TOTAL_SIZE.
** \return    the value read from the memory address.
**
****************************************************************************************/
int32_t EepromSimSINT32Get(uint16_t address)
{
  int32_t  data_val = 0;
  int32_t *data_val_ptr;
  uint32_t saved_cs_state;

  /* check the address */
  if (address >= (EEPROMSIM_TOTAL_SIZE - sizeof(int32_t)) + 1)
  {
	/* invalid address, return a dummy value */
    return data_val;
  }
  /* start critical section */
  saved_cs_state = OsEnterCriticalSection();
  /* determine the pointer of the data to read */
  data_val_ptr = (int32_t*)(&eeprom_overlay_buffer[address]);
  /* read the value from the RAM buffer */
  data_val = *data_val_ptr;
  /* end critical section */
  OsLeaveCriticalSection(saved_cs_state);
  /* return the results */
  return data_val;
} /*** end of EepromSimSINT32Get ***/


/************************************************************************************//**
** \brief     Writes an unsigned 8-bit value to the specified memory address. Use function
**            EepromSimSynchronize to actually process the changes in non-volatile flash.
** \param     address Memory pointer to write to. Must be a valid value in the range
**                    0 and EEPROMSIM_TOTAL_SIZE.
** \param     value Value to write to the memory address.
** \return    none.
**
****************************************************************************************/
void EepromSimUINT8Set(uint16_t address, uint8_t value)
{
  uint8_t *data_val_ptr;
  uint32_t saved_cs_state;

  /* check the address */
  if (address >= (EEPROMSIM_TOTAL_SIZE - sizeof(uint8_t)) + 1)
  {
	/* cannot continue because the address in invalid */
	return;
  }

  /* only continue if the same value is not already there */
  if (EepromSimUINT8Get(address) == value)
  {
    /* already done */
    return;
  }

  /* start critical section */
  saved_cs_state = OsEnterCriticalSection();
  /* determine the pointer of the data to write */
  data_val_ptr = (uint8_t*)(&eeprom_overlay_buffer[address]);
  /* write the data */
  *data_val_ptr = value;
  /* end critical section */
  OsLeaveCriticalSection(saved_cs_state);
  /* set flag that RAM and flash EEPROM are now out of sync */
  eeprom_out_of_sync = TRUE;
} /*** end of EepromSimUINT8Set ***/


/************************************************************************************//**
** \brief     Writes an signed 8-bit value to the specified memory address. Use function
**            EepromSimSynchronize to actually process the changes in non-volatile flash.
** \param     address Memory pointer to write to. Must be a valid value in the range
**                    0 and EEPROMSIM_TOTAL_SIZE.
** \param     value Value to write to the memory address.
** \return    none.
**
****************************************************************************************/
void EepromSimSINT8Set(uint16_t address, int8_t value)
{
  int8_t *data_val_ptr;
  uint32_t saved_cs_state;

  /* check the address */
  if (address >= (EEPROMSIM_TOTAL_SIZE - sizeof(int8_t)) + 1)
  {
	/* cannot continue because the address in invalid */
	return;
  }

  /* only continue if the same value is not already there */
  if (EepromSimSINT8Get(address) == value)
  {
    /* already done */
    return;
  }

  /* start critical section */
  saved_cs_state = OsEnterCriticalSection();
  /* determine the pointer of the data to write */
  data_val_ptr = (int8_t*)(&eeprom_overlay_buffer[address]);
  /* write the data */
  *data_val_ptr = value;
  /* end critical section */
  OsLeaveCriticalSection(saved_cs_state);
  /* set flag that RAM and flash EEPROM are now out of sync */
  eeprom_out_of_sync = TRUE;
} /*** end of EepromSimSINT8Set ***/


/************************************************************************************//**
** \brief     Writes an unsigned 16-bit value to the specified memory address. Use function
**            EepromSimSynchronize to actually process the changes in non-volatile flash.
** \param     address Memory pointer to write to. Must be a valid value in the range
**                    0 and EEPROMSIM_TOTAL_SIZE.
** \param     value Value to write to the memory address.
** \return    none.
**
****************************************************************************************/
void EepromSimUINT16Set(uint16_t address, uint16_t value)
{
  uint16_t *data_val_ptr;
  uint32_t saved_cs_state;

  /* check the address */
  if (address >= (EEPROMSIM_TOTAL_SIZE - sizeof(uint16_t)) + 1)
  {
	/* cannot continue because the address in invalid */
	return;
  }

  /* only continue if the same value is not already there */
  if (EepromSimUINT16Get(address) == value)
  {
    /* already done */
    return;
  }

  /* start critical section */
  saved_cs_state = OsEnterCriticalSection();
  /* determine the pointer of the data to write */
  data_val_ptr = (uint16_t*)(&eeprom_overlay_buffer[address]);
  /* write the data */
  *data_val_ptr = value;
  /* end critical section */
  OsLeaveCriticalSection(saved_cs_state);
  /* set flag that RAM and flash EEPROM are now out of sync */
  eeprom_out_of_sync = TRUE;
} /*** end of EepromSimUINT16Set ***/


/************************************************************************************//**
** \brief     Writes an signed 16-bit value to the specified memory address. Use function
**            EepromSimSynchronize to actually process the changes in non-volatile flash.
** \param     address Memory pointer to write to. Must be a valid value in the range
**                    0 and EEPROMSIM_TOTAL_SIZE.
** \param     value Value to write to the memory address.
** \return    none.
**
****************************************************************************************/
void EepromSimSINT16Set(uint16_t address, int16_t value)
{
  int16_t *data_val_ptr;
  uint32_t saved_cs_state;

  /* check the address */
  if (address >= (EEPROMSIM_TOTAL_SIZE - sizeof(int16_t)) + 1)
  {
	/* cannot continue because the address in invalid */
	return;
  }

  /* only continue if the same value is not already there */
  if (EepromSimSINT16Get(address) == value)
  {
    /* already done */
    return;
  }

  /* start critical section */
  saved_cs_state = OsEnterCriticalSection();
  /* determine the pointer of the data to write */
  data_val_ptr = (int16_t*)(&eeprom_overlay_buffer[address]);
  /* write the data */
  *data_val_ptr = value;
  /* end critical section */
  OsLeaveCriticalSection(saved_cs_state);
  /* set flag that RAM and flash EEPROM are now out of sync */
  eeprom_out_of_sync = TRUE;
} /*** end of EepromSimSINT16Set ***/


/************************************************************************************//**
** \brief     Writes an unsigned 32-bit value to the specified memory address. Use function
**            EepromSimSynchronize to actually process the changes in non-volatile flash.
** \param     address Memory pointer to write to. Must be a valid value in the range
**                    0 and EEPROMSIM_TOTAL_SIZE.
** \param     value Value to write to the memory address.
** \return    none.
**
****************************************************************************************/
void EepromSimUINT32Set(uint16_t address, uint32_t value)
{
  uint32_t *data_val_ptr;
  uint32_t saved_cs_state;

  /* check the address */
  if (address >= (EEPROMSIM_TOTAL_SIZE - sizeof(uint32_t)) + 1)
  {
	/* cannot continue because the address in invalid */
	return;
  }

  /* only continue if the same value is not already there */
  if (EepromSimUINT32Get(address) == value)
  {
    /* already done */
    return;
  }

  /* start critical section */
  saved_cs_state = OsEnterCriticalSection();
  /* determine the pointer of the data to write */
  data_val_ptr = (uint32_t*)(&eeprom_overlay_buffer[address]);
  /* write the data */
  *data_val_ptr = value;
  /* end critical section */
  OsLeaveCriticalSection(saved_cs_state);
  /* set flag that RAM and flash EEPROM are now out of sync */
  eeprom_out_of_sync = TRUE;
} /*** end of EepromSimUINT32Set ***/


/************************************************************************************//**
** \brief     Writes an signed 32-bit value to the specified memory address. Use function
**            EepromSimSynchronize to actually process the changes in non-volatile flash.
** \param     address Memory pointer to write to. Must be a valid value in the range
**                    0 and EEPROMSIM_TOTAL_SIZE.
** \param     value Value to write to the memory address.
** \return    none.
**
****************************************************************************************/
void EepromSimSINT32Set(uint16_t address, int32_t value)
{
  int32_t *data_val_ptr;
  uint32_t saved_cs_state;

  /* check the address */
  if (address >= (EEPROMSIM_TOTAL_SIZE - sizeof(int32_t)) + 1)
  {
	/* cannot continue because the address in invalid */
	return;
  }

  /* only continue if the same value is not already there */
  if (EepromSimSINT32Get(address) == value)
  {
    /* already done */
    return;
  }

  /* start critical section */
  saved_cs_state = OsEnterCriticalSection();
  /* determine the pointer of the data to write */
  data_val_ptr = (int32_t*)(&eeprom_overlay_buffer[address]);
  /* write the data */
  *data_val_ptr = value;
  /* end critical section */
  OsLeaveCriticalSection(saved_cs_state);
  /* set flag that RAM and flash EEPROM are now out of sync */
  eeprom_out_of_sync = TRUE;
} /*** end of EepromSimSINT32Set ***/


/************************************ end of eeprom_sim.c ******************************/


